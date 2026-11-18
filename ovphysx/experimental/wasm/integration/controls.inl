// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Named controls borrow the original OVPhysX attachment. Public sessions provide
// live state and kinematic writes; transient stage columns use the original USD
// property-update path for joint drives. No native SDK address is dereferenced.
namespace
{
constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
uint32_t controlsStageHandle = 0;
int64_t controlsMetadataOrdinal = 0;
int64_t controlsPendingOrdinal = 0;
bool controlsPending = false;
double controlsMetersPerUnit = 0.0;
bool controlsPartial = false;
std::string controlsCleanupError;

struct ControlRejected : std::runtime_error
{
    using std::runtime_error::runtime_error;
};
struct ControlFailed : std::runtime_error
{
    using std::runtime_error::runtime_error;
};
struct ControlBody
{
    ovx_primpath_t path{};
    ovphysx_query_target_t target = OVPHYSX_QUERY_TARGET_RIGID_ACTOR;
    bool kinematic = false;
    std::array<float, 13> fixedState{};
};
struct ControlJoint
{
    ovx_primpath_t path{};
    bool articulation = false;
    bool velocityDrive = false;
    std::string bodies[2];
    pxr::GfQuatd localRotations[2]{pxr::GfQuatd(1.0), pxr::GfQuatd(1.0)};
    pxr::GfQuatd axisRotation{1.0};
};
std::unordered_map<std::string, ControlBody> controlsBodies;
std::unordered_map<std::string, ControlJoint> controlsJoints;
std::unordered_map<std::string, std::string> controlsJointErrors;

void resetControls()
{
    controlsStageHandle = 0;
    controlsMetadataOrdinal = 0;
    controlsPendingOrdinal = 0;
    controlsPending = false;
    controlsMetersPerUnit = 0.0;
    controlsBodies.clear();
    controlsJoints.clear();
    controlsJointErrors.clear();
}
void advanceControlsMetadata(int64_t ordinal)
{
    controlsMetadataOrdinal = ordinal;
}
void controlRequire(bool condition, const char* message)
{
    if (!condition)
        throw ControlRejected(message);
}
void controlComplete(ovphysx_enqueue_result_t result)
{
    if (complete(result))
        throw ControlFailed(errorText);
}
// Cleanup must not replace the primary rejection or let an unhealthy owner
// appear reusable. A rejected release remains instance-owned until teardown.
template<class Function>
void cleanupControl(Function&& function)
{
    const std::string saved = errorText;
    try
    {
        function();
    }
    catch (const std::exception& error)
    {
        controlsPartial = true;
        if (!controlsCleanupError.empty())
            controlsCleanupError += "; ";
        controlsCleanupError += error.what();
    }
    catch (...)
    {
        controlsPartial = true;
        if (!controlsCleanupError.empty())
            controlsCleanupError += "; ";
        controlsCleanupError += "Unexpected control cleanup failure";
    }
    errorText = saved;
}
int failControl(const char* message)
{
    const std::string diagnostic = controlsCleanupError.empty() ? message :
        std::string(message) + "; cleanup also failed: " + controlsCleanupError;
    return failure(diagnostic.c_str());
}
ovx_token_or_string_t controlAttribute(const char* name)
{
    return {OVX_INVALID_TOKEN, {name, std::strlen(name)}};
}

// Scratch is copied immediately. This paired main-module ABI owns the original
// typed stage read and dictionary decoding; it does not parse USD a second time.
bool controlReadStage(const std::string& path, const char* attribute)
{
    const int status = ovs_wasm_read_attribute(controlsStageHandle, path.data(), path.size(),
        attribute, std::strlen(attribute), controlsMetadataOrdinal);
    if (status == OVSTAGE_ERROR_NOT_FOUND)
        return false;
    if (status)
        throw ControlFailed("Read control metadata " + path + "." + attribute + ": " + ovs_wasm_error());
    return true;
}
std::vector<double> controlStageNumbers(const std::string& path, const char* attribute)
{
    if (!controlReadStage(path, attribute))
        return {};
    const uint32_t count = ovs_wasm_result_count();
    const unsigned char* data = reinterpret_cast<const unsigned char*>(ovs_wasm_result_data());
    std::vector<double> result(count);
    for (uint32_t index = 0; index < count; ++index)
    {
        if (ovs_wasm_result_code() == kDLFloat && ovs_wasm_result_bits() == 32)
        {
            float value;
            std::memcpy(&value, data + index * sizeof(value), sizeof(value));
            result[index] = value;
        }
        else if (ovs_wasm_result_code() == kDLFloat && ovs_wasm_result_bits() == 64)
            std::memcpy(&result[index], data + index * sizeof(double), sizeof(double));
        else if ((ovs_wasm_result_code() == kDLUInt || ovs_wasm_result_code() == kDLBool) && ovs_wasm_result_bits() == 8)
            result[index] = data[index];
        else
            throw ControlRejected("Control metadata " + path + "." + attribute +
                " requires float32, float64 or bool8; received code " + std::to_string(ovs_wasm_result_code()) +
                ", bits " + std::to_string(ovs_wasm_result_bits()));
    }
    return result;
}
std::vector<std::string> controlStageStrings(const std::string& path, const char* attribute)
{
    if (!controlReadStage(path, attribute))
        return {};
    controlRequire(ovs_wasm_result_is_string(), "Control metadata must contain strings or paths");
    std::vector<std::string> result;
    for (uint32_t index = 0; index < ovs_wasm_result_string_count(); ++index)
        result.emplace_back(ovs_wasm_result_string_data(index), ovs_wasm_result_string_size(index));
    return result;
}
double controlScalar(const std::string& path, const char* attribute, double fallback)
{
    const std::vector<double> result = controlStageNumbers(path, attribute);
    controlRequire(result.empty() || result.size() == 1, "Control metadata requires one scalar");
    return result.empty() ? fallback : result[0];
}
void prepareControls()
{
    controlRequire(instance && attached && controlsStageHandle, "Physics has no attached stage");
    controlRequire(!controlsPending, "Finish the pending physics control ordinal first");
    if (!controlsMetersPerUnit)
    {
        const double scale = controlScalar("/", "usd-metadata:metersPerUnit", 0.01);
        controlRequire(std::isfinite(scale) && scale > 0.0, "Invalid stage length units");
        controlsMetersPerUnit = scale;
    }
}
std::vector<std::string> controlPaths(const char* packed, uint32_t bytes, uint32_t count, bool unique)
{
    controlRequire((!count && !bytes) || (packed && bytes && count <= bytes), "Invalid packed control paths");
    std::vector<std::string> result;
    std::unordered_set<std::string> seen;
    size_t offset = 0;
    for (uint32_t index = 0; index < count; ++index)
    {
        controlRequire(offset < bytes, "Missing packed control path");
        const char* end = static_cast<const char*>(std::memchr(packed + offset, 0, bytes - offset));
        controlRequire(end && end > packed + offset && packed[offset] == '/', "Invalid absolute control path");
        std::string path(packed + offset, end);
        controlRequire(!unique || seen.insert(path).second, "Duplicate control target path");
        result.push_back(std::move(path));
        offset = static_cast<size_t>(end - packed) + 1;
    }
    controlRequire(offset == bytes, "Unexpected packed control path bytes");
    return result;
}
ovx_primpath_t controlPathId(const std::string& path)
{
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(ovphysx_wasm_borrow_stage(controlsStageHandle));
    const ovx_string_t text{path.data(), path.size()};
    ovx_primpath_t result{};
    const ovx_api_result_t status = path_dictionary_create_paths_from_strings(dictionary, &text, 1, &result);
    if (status.status)
    {
        if (status.error.ptr)
            path_dictionary_release_error(dictionary, status.error);
        throw ControlRejected("Invalid physics prim path");
    }
    return result;
}
struct ControlQuery
{
    ovphysx_query_handle_t handle = 0;
    ovstage_query_result_t summary{};
    ControlQuery(ovphysx_query_target_t target, ovx_primpath_t path)
    {
        const ovphysx_query_selection_t selection{{}, &path, 1};
        try
        {
            controlComplete(ovphysx_query_selection(instance, target, &selection, &handle, &summary));
        }
        catch (...)
        {
            if (handle)
                cleanupControl([&] { close(); });
            throw;
        }
    }
    explicit ControlQuery(ovphysx_query_target_t target)
    {
        try
        {
            controlComplete(ovphysx_query(instance, target, OVPHYSX_SCOPE_ALL, &handle, &summary));
        }
        catch (...)
        {
            if (handle)
                cleanupControl([&] { close(); });
            throw;
        }
    }
    ~ControlQuery()
    {
        if (handle)
            cleanupControl([&] { close(); });
    }
    void close()
    {
        const ovphysx_query_handle_t value = std::exchange(handle, 0);
        controlComplete(ovphysx_release_query(instance, value));
    }
};
bool controlHasObject(ovphysx_query_target_t target, ovx_primpath_t path)
{
    ControlQuery query(target, path);
    const bool found = query.summary.total_prim_count == 1;
    query.close();
    return found;
}
void controlStageComplete(ovstage_instance_t* stage, ovstage_enqueue_result_t operation)
{
    if (operation.status != OVSTAGE_OK)
        throw ControlFailed(ovstage_get_error_string(stage, operation.status));
    ovstage_op_wait_result_t waited{};
    const ovstage_api_status_t status = ovstage_wait_op(stage, operation.op_index, OVSTAGE_TIMEOUT_INFINITE, &waited);
    std::string error;
    if (waited.error_op_id_count)
    {
        const ovx_string_t detail = ovstage_get_last_op_error(stage, waited.error_op_ids[0]);
        error = detail.ptr ? std::string(detail.ptr, detail.length) : "OVStage control write failed";
    }
    else if (status != OVSTAGE_OK)
        error = ovstage_get_error_string(stage, status);
    const ovstage_api_status_t released = ovstage_release_op(stage, operation.op_index);
    if (!error.empty())
        throw ControlFailed(error);
    if (released != OVSTAGE_OK)
        throw ControlFailed(ovstage_get_error_string(stage, released));
}
bool controlHasSchema(const std::string& path, const char* schema)
{
    ovstage_instance_t* stage = ovphysx_wasm_borrow_stage(controlsStageHandle);
    const ovx_string_t pathValue{path.data(), path.size()};
    const ovx_string_t schemaValue{schema, std::strlen(schema)};
    const ovstage_predicate_t predicates[]{
        {controlAttribute("usd-path"), OVSTAGE_FILTER_OP_IN, &pathValue, 1},
        {controlAttribute("usd-schemas"), OVSTAGE_FILTER_OP_CONTAINS, &schemaValue, 1}
    };
    const ovstage_filter_t filter{predicates, 2};
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_query_result_t summary{};
    bool ownsSummary = false;
    const auto close = [&] {
        if (ownsSummary)
        {
            ownsSummary = false;
            const ovstage_api_status_t released = ovstage_release_query_result(stage, &summary);
            if (released != OVSTAGE_OK)
                throw ControlFailed(ovstage_get_error_string(stage, released));
        }
        if (query != OVSTAGE_INVALID_QUERY_HANDLE)
            controlStageComplete(stage, ovstage_release_query(stage, std::exchange(query, OVSTAGE_INVALID_QUERY_HANDLE)));
    };
    try
    {
        controlStageComplete(stage, ovstage_query(stage, &filter, nullptr, 0, &query));
        const ovstage_api_status_t fetched = ovstage_fetch_query_result(stage, query, OVSTAGE_TIMEOUT_INFINITE, &summary);
        if (fetched != OVSTAGE_OK)
            throw ControlFailed(ovstage_get_error_string(stage, fetched));
        ownsSummary = true;
        const bool found = summary.total_prim_count == 1;
        close();
        return found;
    }
    catch (...)
    {
        cleanupControl(close);
        throw;
    }
}
struct ControlStageWrite
{
    ovstage_instance_t* stage;
    path_dictionary_instance_t* dictionary;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    float value;
    explicit ControlStageWrite(float input) : stage(ovphysx_wasm_borrow_stage(controlsStageHandle)),
        dictionary(ovstage_get_path_dictionary(stage)), value(input) {}
    ~ControlStageWrite()
    {
        cleanupControl([&] { close(); });
    }
    void prepare(ovx_primpath_t path)
    {
        const ovx_api_result_t status = path_dictionary_create_path_list_from_paths(dictionary, &path, 1, &list);
        if (status.status)
        {
            if (status.error.ptr)
                path_dictionary_release_error(dictionary, status.error);
            throw ControlFailed("Could not prepare joint control paths");
        }
        const ovstage_api_status_t queried = ovstage_query_from_path_list(stage, list, &query);
        if (queried != OVSTAGE_OK)
            throw ControlFailed(ovstage_get_error_string(stage, queried));
    }
    void commit(int64_t ordinal)
    {
        int64_t shape = 1;
        DLTensor tensor{};
        tensor.data = &value;
        tensor.device = {kDLCPU, 0};
        tensor.dtype = {kDLFloat, 32, 1};
        tensor.ndim = 1;
        tensor.shape = &shape;
        ovstage_write_data_t data{};
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.is_array = false;
        controlStageComplete(stage, ovstage_write_attribute(stage, query,
            controlAttribute("drive:angular:physics:targetVelocity"), ordinal, data, OVSTAGE_PRIM_MODE_UPSERT));
    }
    void close()
    {
        if (query != OVSTAGE_INVALID_QUERY_HANDLE)
            controlStageComplete(stage, ovstage_release_query(stage, std::exchange(query, OVSTAGE_INVALID_QUERY_HANDLE)));
        if (list != OVX_INVALID_PRIMPATH_LIST)
        {
            const ovx_api_result_t status = path_dictionary_release_path_list_reference(dictionary,
                std::exchange(list, OVX_INVALID_PRIMPATH_LIST));
            if (status.status)
            {
                if (status.error.ptr)
                    path_dictionary_release_error(dictionary, status.error);
                throw ControlFailed("Could not release joint control paths");
            }
        }
    }
};

std::vector<float> controlReadColumn(ovphysx_query_target_t target, ovx_primpath_t path,
                                    const char* name, uint16_t width)
{
    ControlQuery query(target, path);
    controlRequire(query.summary.total_prim_count == 1, "No compatible named physics object");
    ovphysx_read_handle_t read = 0;
    const ovx_token_or_string_t attribute = controlAttribute(name);
    std::vector<float> output;
    try
    {
        controlComplete(ovphysx_read(instance, query.handle, &attribute, 1, {}, &read, nullptr));
        for (;;)
        {
            const ovstage_read_group_t* group = nullptr;
            controlComplete(ovphysx_fetch_read_next(instance, read, &group));
            if (!group)
                break;
            try
            {
                if (group->column_count != 1 || !group->columns || group->columns[0].is_array ||
                    group->columns[0].data.tensor_count != 1 || !group->columns[0].data.tensors)
                    throw ControlFailed("Unexpected named physics column layout");
                const DLTensor& tensor = group->columns[0].data.tensors[0];
                if (tensor.device.device_type != kDLCPU || tensor.dtype.code != kDLFloat || tensor.dtype.bits != 32 ||
                    tensor.dtype.lanes != width || tensor.ndim != 1 || !tensor.shape || tensor.shape[0] != 1 || !tensor.data ||
                    !output.empty())
                    throw ControlFailed("Expected one finite CPU physics row");
                output.resize(width);
                std::memcpy(output.data(), static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset,
                    width * sizeof(float));
                for (float value : output)
                    if (!std::isfinite(value))
                        throw ControlFailed("Physics returned nonfinite named state");
            }
            catch (...)
            {
                cleanupControl([&] { controlComplete(ovphysx_release_group(instance, read, group->read_group_id)); });
                throw;
            }
            controlComplete(ovphysx_release_group(instance, read, group->read_group_id));
        }
        controlComplete(ovphysx_release_read(instance, std::exchange(read, 0)));
        query.close();
    }
    catch (...)
    {
        if (read)
            cleanupControl([&] { controlComplete(ovphysx_release_read(instance, read)); });
        throw;
    }
    if (output.size() != width)
        throw ControlFailed("Requested physics state is unavailable");
    return output;
}
pxr::GfQuatd controlQuaternion(const std::vector<double>& values)
{
    controlRequire(values.size() == 4, "Expected a quaternion control frame");
    pxr::GfQuatd result(values[3], pxr::GfVec3d(values[0], values[1], values[2]));
    const double length = result.GetLength();
    controlRequire(std::isfinite(length) && length > 1e-12, "Invalid quaternion control frame");
    return result / length;
}
ControlBody& controlBody(const std::string& path)
{
    const auto old = controlsBodies.find(path);
    if (old != controlsBodies.end())
        return old->second;
    ControlBody body;
    body.path = controlPathId(path);
    if (controlHasObject(OVPHYSX_QUERY_TARGET_RIGID_BODY, body.path))
    {
        body.target = OVPHYSX_QUERY_TARGET_RIGID_BODY;
        body.kinematic = controlScalar(path, "physics:kinematicEnabled", 0.0) != 0.0;
    }
    else if (controlHasObject(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, body.path))
        body.target = OVPHYSX_QUERY_TARGET_ARTICULATION_LINK;
    else
    {
        controlRequire(controlHasObject(OVPHYSX_QUERY_TARGET_RIGID_ACTOR, body.path), "No physics body at requested path");
        const std::vector<double> matrix = controlStageNumbers(path, "omni:fabric:worldMatrix");
        controlRequire(matrix.size() == 16, "Static actor has no populated world matrix");
        pxr::GfMatrix4d world;
        std::memcpy(world.GetArray(), matrix.data(), 16 * sizeof(double));
        const pxr::GfQuatd rotation = pxr::GfTransform(world).GetRotation().GetQuat();
        // OvstageWalker fills static actor position from this matrix and uses
        // Gf-compatible rotation decomposition. The native query proves actor
        // ownership; statics have no solver motion and no writable body session.
        for (size_t axis = 0; axis < 3; ++axis)
        {
            body.fixedState[axis] = static_cast<float>(matrix[12 + axis] * controlsMetersPerUnit);
            body.fixedState[3 + axis] = static_cast<float>(rotation.GetImaginary()[axis]);
        }
        body.fixedState[6] = static_cast<float>(rotation.GetReal());
    }
    return controlsBodies.emplace(path, std::move(body)).first->second;
}
std::array<float, 13> controlBodyState(const std::string& path)
{
    if (path.empty())
    {
        std::array<float, 13> world{};
        world[6] = 1.0f;
        return world;
    }
    ControlBody& body = controlBody(path);
    if (body.target == OVPHYSX_QUERY_TARGET_RIGID_ACTOR)
        return body.fixedState;
    std::array<float, 13> result{};
    const char* names[]{OVPHYSX_ATTR_POSITION, OVPHYSX_ATTR_ORIENTATION,
        OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_ATTR_ANGULAR_VELOCITY};
    const uint16_t widths[]{3, 4, 3, 3};
    size_t offset = 0;
    for (size_t column = 0; column < 4; ++column)
    {
        const std::vector<float> data = controlReadColumn(body.target, body.path, names[column], widths[column]);
        for (float value : data)
            result[offset++] = (column == 0 || column == 2) ? static_cast<float>(value * controlsMetersPerUnit) : value;
    }
    return result;
}
pxr::GfMatrix4d controlWorldMatrix(const std::string& path)
{
    const std::vector<double> data = controlStageNumbers(path, "omni:fabric:worldMatrix");
    controlRequire(data.size() == 16, "Joint endpoint has no populated world matrix");
    pxr::GfMatrix4d result;
    std::memcpy(result.GetArray(), data.data(), 16 * sizeof(double));
    return result;
}
std::string controlEndpointBody(const std::string& target)
{
    std::string path = target;
    while (!path.empty() && path != "/")
    {
        const ovx_primpath_t id = controlPathId(path);
        if (controlHasObject(OVPHYSX_QUERY_TARGET_RIGID_BODY, id) ||
            controlHasObject(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, id) ||
            controlHasObject(OVPHYSX_QUERY_TARGET_RIGID_ACTOR, id))
        {
            controlBody(path);
            return path;
        }
        path.resize(path.find_last_of('/'));
    }
    return {};
}
ControlJoint& controlJoint(const std::string& path)
{
    const auto rejected = controlsJointErrors.find(path);
    if (rejected != controlsJointErrors.end())
        throw ControlRejected(rejected->second);
    const auto old = controlsJoints.find(path);
    if (old != controlsJoints.end())
        return old->second;
    const std::vector<std::string> type = controlStageStrings(path, "usd-prim-type");
    controlRequire(type.size() == 1 && type[0] == "PhysicsRevoluteJoint", "Only revolute joints support named joint controls");
    ControlJoint joint;
    joint.path = controlPathId(path);
    joint.articulation = controlHasObject(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, joint.path);
    controlRequire(joint.articulation || controlHasObject(OVPHYSX_QUERY_TARGET_JOINT, joint.path),
        "No native revolute joint at requested path");
    const double damping = controlScalar(path, "drive:angular:physics:damping", 0.0);
    const double force = controlScalar(path, "drive:angular:physics:maxForce", std::numeric_limits<double>::infinity());
    joint.velocityDrive = damping > 0.0 && force > 0.0 && controlHasSchema(path, "PhysicsDriveAPI:angular");
    if (!joint.articulation)
    {
        for (size_t side = 0; side < 2; ++side)
        {
            const std::vector<std::string> bodies = controlStageStrings(path, side ? "physics:body1" : "physics:body0");
            controlRequire(bodies.size() <= 1, "A controlled joint endpoint must name at most one body");
            const std::vector<double> rotation = controlStageNumbers(path, side ? "physics:localRot1" : "physics:localRot0");
            if (!rotation.empty())
                joint.localRotations[side] = controlQuaternion(rotation);
            if (!bodies.empty())
            {
                joint.bodies[side] = controlEndpointBody(bodies[0]);
                if (joint.bodies[side] != bodies[0])
                {
                    const std::vector<double> position = controlStageNumbers(path, side ? "physics:localPos1" : "physics:localPos0");
                    controlRequire(position.empty() || position.size() == 3, "Invalid joint anchor position");
                    pxr::GfMatrix4d anchor(1.0);
                    anchor.SetRotate(joint.localRotations[side]);
                    if (!position.empty())
                        anchor.SetTranslateOnly(pxr::GfVec3d(position[0], position[1], position[2]));
                    const pxr::GfMatrix4d bodyWorld = joint.bodies[side].empty() ? pxr::GfMatrix4d(1.0) :
                        controlWorldMatrix(joint.bodies[side]);
                    // Native transformJointFrameToBody uses this row-vector
                    // composition, including a non-body target's world frame.
                    const pxr::GfMatrix4d local = anchor * controlWorldMatrix(bodies[0]) * bodyWorld.GetInverse();
                    joint.localRotations[side] = local.RemoveScaleShear().ExtractRotationQuat().GetNormalized();
                }
            }
        }
        const std::vector<std::string> axis = controlStageStrings(path, "physics:axis");
        const std::string value = axis.empty() ? "X" : axis[0];
        const double h = std::sqrt(0.5);
        if (value == "Y")
            joint.axisRotation = pxr::GfQuatd(h, pxr::GfVec3d(0, 0, h));
        else if (value == "Z")
            joint.axisRotation = pxr::GfQuatd(h, pxr::GfVec3d(0, -h, 0));
        else
            controlRequire(value == "X", "Invalid revolute axis");
    }
    return controlsJoints.emplace(path, std::move(joint)).first->second;
}
void controlDictionaryComplete(path_dictionary_instance_t* dictionary, ovx_api_result_t result)
{
    if (result.status == OVX_API_SUCCESS)
        return;
    const std::string message = result.error.ptr ? std::string(result.error.ptr, result.error.length) :
        "Physics control path dictionary failed";
    if (result.error.ptr)
        path_dictionary_release_error(dictionary, result.error);
    throw ControlFailed(message);
}
std::string controlPathName(path_dictionary_instance_t* dictionary, ovx_primpath_t path)
{
    std::vector<ovx_token_t> buffer(16);
    ovx_token_t* tokens = nullptr;
    size_t count = 0;
    size_t processed = 0;
    for (;;)
    {
        controlDictionaryComplete(dictionary, path_dictionary_get_tokens_from_paths(dictionary, &path, 1,
            buffer.data(), buffer.size(), &tokens, &count, &processed));
        if (processed == 1)
            break;
        if (buffer.size() > buffer.max_size() / 2)
            throw ControlFailed("Physics control path exceeds dictionary buffer capacity");
        buffer.resize(buffer.size() * 2);
    }
    if (!count)
        return "/";
    if (!tokens)
        throw ControlFailed("Physics control path has no components");
    std::vector<ovx_string_t> components(count);
    controlDictionaryComplete(dictionary, path_dictionary_get_strings_from_tokens(dictionary, tokens, count, components.data()));
    std::string result;
    for (const ovx_string_t& component : components)
    {
        if (!component.ptr || !component.length)
            throw ControlFailed("Physics control path has an empty component");
        result += '/';
        result.append(component.ptr, component.length);
    }
    return result;
}
std::vector<std::string> controlJointPaths(ovphysx_query_target_t target)
{
    ControlQuery query(target);
    std::vector<std::string> result;
    if (!query.summary.total_prim_count)
    {
        query.close();
        return result;
    }
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(ovphysx_wasm_borrow_stage(controlsStageHandle));
    ovphysx_read_handle_t read = 0;
    // This is the public identity-bearing column for standalone joints. Only
    // the group's prim list is consumed; pointer values are never inspected.
    const ovx_token_or_string_t attribute = controlAttribute(OVPHYSX_ATTR_PHYSX_POINTER);
    try
    {
        controlComplete(ovphysx_read(instance, query.handle, &attribute, 1, {}, &read, nullptr));
        for (;;)
        {
            const ovstage_read_group_t* group = nullptr;
            controlComplete(ovphysx_fetch_read_next(instance, read, &group));
            if (!group)
                break;
            try
            {
                for (uint32_t row = 0; row < group->prims.count; ++row)
                {
                    const size_t index = group->prims.offset + static_cast<size_t>(group->prims.index_map ?
                        group->prims.index_map[row] : row);
                    ovx_primpath_t path{};
                    size_t count = 0;
                    controlDictionaryComplete(dictionary, path_dictionary_get_paths_from_path_list(dictionary,
                        group->prims.list, index, 1, &path, &count));
                    if (count != 1)
                        throw ControlFailed("Physics joint inventory returned an incomplete path list");
                    result.push_back(controlPathName(dictionary, path));
                }
            }
            catch (...)
            {
                cleanupControl([&] { controlComplete(ovphysx_release_group(instance, read, group->read_group_id)); });
                throw;
            }
            controlComplete(ovphysx_release_group(instance, read, group->read_group_id));
        }
        controlComplete(ovphysx_release_read(instance, std::exchange(read, 0)));
        query.close();
    }
    catch (...)
    {
        if (read)
            cleanupControl([&] { controlComplete(ovphysx_release_read(instance, read)); });
        throw;
    }
    if (result.size() != query.summary.total_prim_count)
        throw ControlFailed("Physics joint inventory did not expose every native joint");
    return result;
}
void snapshotControlJoints()
{
    prepareControls();
    for (const ovphysx_query_target_t target : {OVPHYSX_QUERY_TARGET_JOINT, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT})
    {
        for (const std::string& path : controlJointPaths(target))
        {
            try
            {
                // Snapshot authored frames while body/descendant world matrices
                // still describe the same pose, before any physics publication.
                controlJoint(path);
            }
            catch (const ControlRejected& error)
            {
                if (!controlsCleanupError.empty())
                    throw ControlFailed(error.what());
                controlsJointErrors.emplace(path, error.what());
            }
        }
    }
}
std::array<float, 2> controlJointState(const ControlJoint& joint)
{
    if (joint.articulation)
    {
        return {static_cast<float>(controlReadColumn(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, joint.path,
                    OVPHYSX_ATTR_JOINT_POSITION, 1)[0] * kRadiansPerDegree),
                static_cast<float>(controlReadColumn(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, joint.path,
                    OVPHYSX_ATTR_JOINT_VELOCITY, 1)[0] * kRadiansPerDegree)};
    }
    const std::array<float, 13> state0 = controlBodyState(joint.bodies[0]);
    const std::array<float, 13> state1 = controlBodyState(joint.bodies[1]);
    const pxr::GfQuatd q0 = pxr::GfQuatd(state0[6], pxr::GfVec3d(state0[3], state0[4], state0[5])) *
        joint.localRotations[0] * joint.axisRotation;
    const pxr::GfQuatd q1 = pxr::GfQuatd(state1[6], pxr::GfVec3d(state1[3], state1[4], state1[5])) *
        joint.localRotations[1] * joint.axisRotation;
    const pxr::GfQuatd relative = q0.GetInverse() * q1;
    const double x = relative.GetImaginary()[0];
    const double w = relative.GetReal();
    const double length = std::sqrt(x * x + w * w);
    // Match the native standalone twist branch, including rotations beyond pi.
    const double angle = x != 0.0 && length > 0.0 ? std::copysign(2 * std::acos(std::clamp(w / length, -1.0, 1.0)), x) : 0.0;
    const pxr::GfVec3d axis = q0.Transform(pxr::GfVec3d(1, 0, 0));
    const pxr::GfVec3d relativeVelocity(state1[10] - state0[10], state1[11] - state0[11], state1[12] - state0[12]);
    return {static_cast<float>(angle), static_cast<float>(pxr::GfDot(axis, relativeVelocity))};
}

// A write session is prepared and populated before any group is committed.
// Session storage stays borrowed until commit/release; no mapped buffer escapes.
struct ControlKinematicWrite
{
    std::unique_ptr<ControlQuery> query;
    ovphysx_write_handle_t write = 0;
    std::vector<const ovstage_map_group_t*> groups;
    ~ControlKinematicWrite()
    {
        if (write)
            cleanupControl([&] { controlComplete(ovphysx_release_write(instance, std::exchange(write, 0))); });
    }
    void close()
    {
        controlComplete(ovphysx_release_write(instance, std::exchange(write, 0)));
        query->close();
    }
};

void prepareKinematicWrite(ControlKinematicWrite& owner, const ControlBody& body, const float* target)
{
    owner.query.reset(new ControlQuery(OVPHYSX_QUERY_TARGET_RIGID_BODY, body.path));
    const ovx_token_or_string_t attributes[]{controlAttribute(OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION),
        controlAttribute(OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION)};
    controlComplete(ovphysx_write(instance, owner.query->handle, attributes, 2, &owner.write, nullptr));
    size_t columns = 0;
    for (;;)
    {
        ovphysx_fetched_write_group_t fetched{};
        controlComplete(ovphysx_fetch_write_next(instance, owner.write, &fetched));
        if (!fetched.group)
            break;
        const ovstage_map_group_t* group = fetched.group;
        if (group->data.tensor_count != 1 || !group->data.tensors)
            throw ControlFailed("Unexpected kinematic write tensor");
        const DLTensor& tensor = group->data.tensors[0];
        const uint16_t width = tensor.dtype.lanes;
        if (fetched.attribute_index > 1 || width != (fetched.attribute_index == 0 ? 3 : 4))
            throw ControlFailed("Unexpected kinematic write attribute");
        if (tensor.device.device_type != kDLCPU || tensor.dtype.code != kDLFloat || tensor.dtype.bits != 32 ||
            tensor.ndim != 1 || !tensor.shape || tensor.shape[0] != 1 || !tensor.data || (width != 3 && width != 4))
            throw ControlFailed("Unexpected kinematic write layout");
        float* output = reinterpret_cast<float*>(static_cast<unsigned char*>(tensor.data) + tensor.byte_offset);
        for (uint16_t lane = 0; lane < width; ++lane)
            output[lane] = width == 3 ? static_cast<float>(target[lane] / controlsMetersPerUnit) : target[3 + lane];
        columns |= width == 3 ? 1 : 2;
        owner.groups.push_back(group);
    }
    if (columns != 3)
        throw ControlFailed("Kinematic target columns are unavailable");
}

template<class Function>
int invokeControl(Function&& function)
{
    errorText.clear();
    controlsPartial = false;
    controlsCleanupError.clear();
    values.clear();
    try
    {
        function();
        return controlsCleanupError.empty() ? 0 : failControl("Physics control cleanup failed");
    }
    catch (const ControlRejected& error)
    {
        return failControl(error.what());
    }
    catch (const std::exception& error)
    {
        controlsPartial = true;
        return failControl(error.what());
    }
    catch (...)
    {
        controlsPartial = true;
        return failControl("Unexpected native physics control failure");
    }
}
int initializeControls(uint32_t handle, int64_t ordinal)
{
    return invokeControl([&] {
        resetControls();
        controlsStageHandle = handle;
        controlsMetadataOrdinal = ordinal;
        snapshotControlJoints();
    });
}
} // namespace

extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_control_error_is_partial()
{
    return controlsPartial ? 1 : 0;
}
extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_read_body_states(const char* paths, uint32_t bytes, uint32_t count)
{
    return invokeControl([&] {
        prepareControls();
        const std::vector<std::string> names = controlPaths(paths, bytes, count, false);
        for (const std::string& name : names)
            controlBody(name);
        for (const std::string& name : names)
        {
            const std::array<float, 13> state = controlBodyState(name);
            values.insert(values.end(), state.begin(), state.end());
        }
    });
}
extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_read_joint_states(const char* paths, uint32_t bytes, uint32_t count)
{
    return invokeControl([&] {
        prepareControls();
        const std::vector<std::string> names = controlPaths(paths, bytes, count, false);
        for (const std::string& name : names)
            controlJoint(name);
        for (const std::string& name : names)
        {
            const std::array<float, 2> state = controlJointState(controlJoint(name));
            values.insert(values.end(), state.begin(), state.end());
        }
    });
}
extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_set_joint_velocity_targets(uint32_t stageHandle, int64_t ordinal,
    const char* paths, uint32_t bytes, uint32_t count, const float* targets)
{
    return invokeControl([&] {
        prepareControls();
        controlRequire(stageHandle == controlsStageHandle && ordinal > controlsMetadataOrdinal, "Invalid physics control ordinal");
        const std::vector<std::string> names = controlPaths(paths, bytes, count, true);
        controlRequire(!count || targets, "Missing joint velocity targets");
        std::vector<float> degrees;
        for (size_t index = 0; index < names.size(); ++index)
        {
            controlRequire(controlJoint(names[index]).velocityDrive, "Joint needs an authored positive damping and force limit");
            const float value = static_cast<float>(targets[index] / kRadiansPerDegree);
            controlRequire(std::isfinite(value), "Invalid joint velocity target");
            degrees.push_back(value);
        }
        if (names.empty())
            return;
        std::vector<std::unique_ptr<ControlStageWrite>> writes;
        for (size_t index = 0; index < names.size(); ++index)
        {
            std::unique_ptr<ControlStageWrite> write(new ControlStageWrite(degrees[index]));
            write->prepare(controlJoint(names[index]).path);
            writes.push_back(std::move(write));
        }
        controlsPartial = true;
        for (const std::unique_ptr<ControlStageWrite>& write : writes)
            write->commit(ordinal);
        for (const std::unique_ptr<ControlStageWrite>& write : writes)
            write->close();
        controlsPendingOrdinal = ordinal;
        controlsPending = true;
    });
}
extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_apply_controls(int64_t ordinal)
{
    return invokeControl([&] {
        controlRequire(instance && attached && controlsPending && controlsPendingOrdinal == ordinal,
            "No matching sealed physics control ordinal");
        controlsPartial = true;
        ovstage_ordinal_range_t range{};
        range.start_ordinal = ordinal;
        range.end_ordinal = ordinal;
        range.has_start_ordinal = true;
        controlComplete(ovphysx_update_from_ovstage(instance, range));
        controlsPending = false;
        advanceControlsMetadata(ordinal);
    });
}
extern "C" EMSCRIPTEN_KEEPALIVE int ovp_wasm_set_kinematic_targets(const char* paths,
    uint32_t bytes, uint32_t count, const float* targets)
{
    return invokeControl([&] {
        prepareControls();
        const std::vector<std::string> names = controlPaths(paths, bytes, count, true);
        controlRequire(!count || targets, "Missing kinematic targets");
        std::vector<std::unique_ptr<ControlKinematicWrite>> writes;
        for (size_t index = 0; index < names.size(); ++index)
        {
            const ControlBody& body = controlBody(names[index]);
            controlRequire(body.kinematic && body.target == OVPHYSX_QUERY_TARGET_RIGID_BODY, "No kinematic body at requested path");
            double norm = 0;
            for (size_t lane = 0; lane < 7; ++lane)
            {
                const float value = targets[index * 7 + lane];
                controlRequire(std::isfinite(value) && (lane >= 3 || std::isfinite(static_cast<float>(value / controlsMetersPerUnit))),
                    "Invalid kinematic target value");
                if (lane >= 3)
                    norm += static_cast<double>(value) * value;
            }
            controlRequire(std::abs(norm - 1.0) < 1e-4, "Kinematic orientation must be normalized");
        }
        for (size_t index = 0; index < names.size(); ++index)
        {
            std::unique_ptr<ControlKinematicWrite> write(new ControlKinematicWrite);
            prepareKinematicWrite(*write, controlBody(names[index]), targets + index * 7);
            writes.push_back(std::move(write));
        }
        if (writes.empty())
            return;
        controlsPartial = true;
        for (const std::unique_ptr<ControlKinematicWrite>& write : writes)
            for (const ovstage_map_group_t* group : write->groups)
                controlComplete(ovphysx_commit_group(instance, write->write, group, {}));
        for (const std::unique_ptr<ControlKinematicWrite>& write : writes)
            write->close();
    });
}
