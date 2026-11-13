// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PvdDomDocumentInternal.h"
#include "PvdDomUtils.h"
#include "PvdDomHierarchy.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <string.h>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace PvdDomInspect
{
namespace
{
class FileStream : public OmniPvdReadStream
{
public:
    enum class ReadFailure { None, PhysicalEof, Cap, Io };
    explicit FileStream(const char* path, uint64_t cap) : file(path, std::ios::binary), cap(cap)
    {
        if (!file) return;
        file.seekg(0, std::ios::end);
        const std::streamoff end = file.tellg();
        if (end < 0) { file.setstate(std::ios::badbit); return; }
        size = static_cast<uint64_t>(end);
        file.seekg(0);
    }
    uint64_t OMNI_PVD_CALL readBytes(uint8_t* bytes, uint64_t count) override
    {
        failure = ReadFailure::None;
        const uint64_t remaining = offset <= cap ? cap - offset : 0;
        const uint64_t allowed = (std::min)(count, (std::min)(remaining,
            static_cast<uint64_t>((std::numeric_limits<std::streamsize>::max)())));
        uint64_t n = 0;
        if (allowed)
        {
            file.read(reinterpret_cast<char*>(bytes), static_cast<std::streamsize>(allowed));
            n = static_cast<uint64_t>(file.gcount());
        }
        offset += n;
        if (file.bad() || (n < allowed && offset != size)) failure = ReadFailure::Io;
        else if (n != count)
            failure = offset == size ? ReadFailure::PhysicalEof : ReadFailure::Cap;
        return n;
    }
    bool sameSize()
    {
        if (file.bad()) return false;
        file.clear(file.rdstate() & ~(std::ios::eofbit | std::ios::failbit));
        file.seekg(0, std::ios::end);
        const std::streamoff end = file.tellg();
        return !file.fail() && end >= 0 && static_cast<uint64_t>(end) == size;
    }
    uint64_t OMNI_PVD_CALL skipBytes(uint64_t count) override
    {
        uint8_t scratch[256];
        uint64_t total = 0;
        while (total < count)
        {
            const uint64_t n = readBytes(scratch, (std::min)(count - total, uint64_t(sizeof(scratch))));
            total += n;
            if (!n) break;
        }
        return total;
    }
    bool OMNI_PVD_CALL openStream() override { return file.is_open(); }
    bool OMNI_PVD_CALL closeStream() override { file.close(); return true; }
    std::ifstream file;
    uint64_t size = 0;
    uint64_t offset = 0;
    uint64_t cap;
    ReadFailure failure = ReadFailure::None;
};

struct ReaderDeleter
{
    void operator()(OmniPvdReader* reader) const { if (reader) destroyOmniPvdReader(*reader); }
};

bool fail(Error& error, Error::Code code, const char* message, Error::LimitKind limitKind = Error::LimitKind::None)
{
    error.code = code;
    error.limitKind = limitKind;
    error.message = message;
    return false;
}

// Charges include container growth and map nodes. Released samples remain charged for the load.
class Budget
{
public:
    Budget(const LoadLimits& limits, LoadStats& stats, Error& error) : limits(limits), stats(stats), error(error) {}
    bool charge(uint64_t count, uint64_t unit = 1)
    {
        const uint64_t remaining = limits.maxRetainedBytes - stats.accountedRetainedBytes;
        if (unit && count > remaining / unit)
            return fail(error, Error::Code::Limit, "Retained-data limit exceeded", Error::LimitKind::RetainedBytes);
        stats.accountedRetainedBytes += count * unit;
        return true;
    }
    bool increment(uint64_t& count, uint64_t limit, const char* message, Error::LimitKind limitKind)
    {
        if (count >= limit) return fail(error, Error::Code::Limit, message, limitKind);
        ++count;
        return true;
    }
    bool handlerSteps(uint64_t steps)
    {
        if (stats.handlerSteps > limits.maxHandlerSteps || steps > limits.maxHandlerSteps - stats.handlerSteps)
            return fail(error, Error::Code::Limit, "Handler-step limit exceeded", Error::LimitKind::HandlerSteps);
        stats.handlerSteps += steps;
        return true;
    }
    const LoadLimits& limits;
    LoadStats& stats;
    Error& error;
};

bool named(const std::string& name, const char* text) { return isSameString(name.c_str(), text); }

uint32_t handlerWidth(OmniPvdPhysXClassEnum type, const std::string& name)
{
    // Stored samples of these fields are also read on referenced objects.
    if (named(name, "ovdIntegrationVersionMajor") || named(name, "ovdIntegrationVersionMinor")) return 4;
    if (named(name, "parentLink") || named(name, "articulation")) return 8;
    switch (type)
    {
    case ePxActor:
    case ePxDeformableVolume:
    case ePxDeformableSurface:
        if (named(name, "type")) return 4;
        if (named(name, "shapes") || named(name, "inboundJoint") || named(name, "particleBuffers") ||
            named(name, "simulationMeshShapes")) return 8;
        break;
    case ePxScene:
        if (named(name, "actors") || named(name, "articulations") || named(name, "constraints")) return 8;
        break;
    case ePxPhysics: if (named(name, "scenes")) return 8; break;
    case ePxJoint: if (named(name, "constraint")) return 8; break;
    case ePxArticulationJoint: if (named(name, "childLink")) return 8; break;
    case ePxShape:
        if (named(name, "isExclusive")) return 1;
        if (named(name, "geom")) return 8;
        break;
    case ePxGeomConvexMesh: if (named(name, "convexmesh")) return 8; break;
    case ePxGeomHeightfield: if (named(name, "heightfield")) return 8; break;
    case ePxGeomTriangleMesh: if (named(name, "trianglemesh")) return 8; break;
    case ePxGeomTetMesh: if (named(name, "tetrahedronMesh")) return 8; break;
    case ePxConvexCoreGeometry: if (named(name, "core")) return 8; break;
    default: break;
    }
    return 0;
}

using ClassAttributeLabels = std::unordered_map<OmniPvdClassHandle, std::unordered_set<std::string>>;

bool validateSchema(OmniPvdReader& reader, OmniPvdCommand::Enum command, OmniPvdDOMState& dom,
                    std::unordered_set<OmniPvdAttributeHandle>& attributes, ClassAttributeLabels& names, Budget& budget)
{
    const auto cls = dom.mClassHandleToClassMap.find(reader.getClassHandle());
    if (command == OmniPvdCommand::eREGISTER_CLASS)
    {
        if (!reader.getClassHandle() || cls != dom.mClassHandleToClassMap.end())
            return fail(budget.error, Error::Code::Invalid, "Invalid or duplicate class handle");
        uint64_t depth = 1;
        if (reader.getBaseClassHandle())
        {
            const auto base = dom.mClassHandleToClassMap.find(reader.getBaseClassHandle());
            if (base == dom.mClassHandleToClassMap.end())
                return fail(budget.error, Error::Code::Invalid, "Unregistered base class");
            depth += base->second->mInheritanceChain.size();
        }
        if (depth > budget.limits.maxInheritanceDepth || depth > uint64_t(INT32_MAX))
            return fail(budget.error, Error::Code::Limit, "Inheritance-depth limit exceeded",
                budget.limits.maxInheritanceDepth < uint64_t(INT32_MAX) ? Error::LimitKind::InheritanceDepth : Error::LimitKind::None);
        return budget.increment(budget.stats.classes, budget.limits.maxClasses, "Class limit exceeded", Error::LimitKind::Classes) &&
            budget.charge(512 + 4 * strnlen(reader.getClassName(), OMNI_PVD_MAX_STRING_LENGTH)) && budget.charge(depth, 96);
    }
    if (command == OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE)
        return fail(budget.error, Error::Code::Unsupported, "Nested class attributes are unsupported");
    if (!reader.getAttributeHandle() || attributes.count(reader.getAttributeHandle()) || cls == dom.mClassHandleToClassMap.end())
        return fail(budget.error, Error::Code::Invalid, "Invalid attribute handle or owning class");
    OmniPvdClass& owner = *cls->second;
    std::string sanitized;
    getSanitizedName(sanitized, reader.getAttributeName());
    for (char& c : sanitized) if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + ('a' - 'A'));
    const auto type = reader.getAttributeDataType();
    for (const OmniPvdClass* base : owner.mInheritanceChain)
    {
        if (!budget.handlerSteps(1)) return false;
        // Enum entries are identified by numeric value, not by their display labels.
        if (type == OmniPvdDataType::eENUM_VALUE && base->mIsEnumClass) continue;
        const auto found = names.find(base->mOmniClassHandle);
        if (found != names.end() && found->second.count(sanitized))
            return fail(budget.error, Error::Code::Invalid, "Ambiguous attribute name");
    }
    for (const auto& entry : dom.mClassHandleToClassMap)
    {
        if (!budget.handlerSteps(1)) return false;
        const OmniPvdClass* descendant = entry.second;
        if (descendant == &owner) continue;
        if (!budget.handlerSteps(descendant->mInheritanceChain.size())) return false;
        if (std::find(descendant->mInheritanceChain.begin(), descendant->mInheritanceChain.end(), &owner) ==
            descendant->mInheritanceChain.end()) continue;
        const auto found = names.find(entry.first);
        if (found != names.end() && found->second.count(sanitized))
            return fail(budget.error, Error::Code::Invalid, "Ambiguous attribute name");
    }
    if (owner.mAttributeDefinitions.size() >= 65536)
        return fail(budget.error, Error::Code::Limit, "Attribute index exceeds DOM representation");
    if (type < OmniPvdDataType::eINT8 || type > OmniPvdDataType::eFLAGS_WORD)
        return fail(budget.error, Error::Code::Invalid, "Unknown attribute data type");
    if (type == OmniPvdDataType::eENUM_VALUE)
    {
        if (command != OmniPvdCommand::eREGISTER_ATTRIBUTE || (!owner.mIsEnumClass && !owner.mAttributeDefinitions.empty()))
            return fail(budget.error, Error::Code::Invalid, "Mixed enum and value attributes");
    }
    else if (owner.mIsEnumClass)
        return fail(budget.error, Error::Code::Invalid, "Value attribute on enum class");
    if (type == OmniPvdDataType::eFLAGS_WORD)
    {
        const auto enumeration = dom.mClassHandleToClassMap.find(reader.getEnumClassHandle());
        if (command != OmniPvdCommand::eREGISTER_ATTRIBUTE || enumeration == dom.mClassHandleToClassMap.end() ||
            !enumeration->second->mIsEnumClass)
            return fail(budget.error, Error::Code::Invalid, "Unregistered flags enum");
    }
    if (!budget.increment(budget.stats.attributes, budget.limits.maxAttributes, "Attribute limit exceeded", Error::LimitKind::Attributes) ||
        !budget.charge(768 + 4 * strnlen(reader.getAttributeName(), OMNI_PVD_MAX_STRING_LENGTH))) return false;
    attributes.insert(reader.getAttributeHandle());
    names[owner.mOmniClassHandle].insert(sanitized);
    return true;
}

bool isKind(const OmniPvdClass& type, const char* name)
{
    const auto kind = type.mPhysXBaseProcessingClassId;
    if (!strcmp(name, "PxActor") && kind != ePxUndefined)
        return kind == ePxActor || kind == ePxDeformableVolume || kind == ePxDeformableSurface;
    if (!strcmp(name, "PxGeometry") && kind != ePxUndefined)
        return (kind >= ePxGeomSphere && kind <= ePxGeomTriangleMesh) || kind == ePxGeomTetMesh || kind == ePxCustomGeometry;
    const struct { const char* name; OmniPvdPhysXClassEnum kind; } expected[] = {
        {"PxArticulationLink", ePxActor}, {"PxArticulationJointReducedCoordinate", ePxArticulationJoint},
        {"PxArticulationReducedCoordinate", ePxArticulation}, {"PxShape", ePxShape}, {"PxScene", ePxScene},
        {"PxParticleBuffer", ePxParticleBuffer}, {"PxTetrahedronMesh", ePxTetrahedronMesh},
        {"PxTriangleMesh", ePxTriangleMesh}, {"PxConvexMesh", ePxConvexMesh}, {"PxHeightField", ePxHeightfield}
    };
    for (const auto& entry : expected)
        if (!strcmp(entry.name, name) && kind != ePxUndefined && kind != entry.kind) return false;
    for (const OmniPvdClass* cls : type.mInheritanceChain)
        if (cls->mClassName == name) return true;
    return false;
}

struct PendingEdge
{
    const char* expected;
    uint64_t offset;
    uint64_t command;
    bool optional;
};
using PendingEdges = std::map<uint64_t, std::vector<PendingEdge>>;

const char* edgeTarget(OmniPvdPhysXClassEnum kind, const std::string& name, bool& list)
{
    list = false;
    if (kind == ePxActor || kind == ePxDeformableVolume || kind == ePxDeformableSurface)
    {
        if (named(name, "shapes")) { list = true; return "PxShape"; }
        if (named(name, "particleBuffers")) { list = true; return "PxParticleBuffer"; }
        if (named(name, "simulationMeshShapes")) { list = true; return "PxTetrahedronMesh"; }
        if (named(name, "articulation")) return "PxArticulationReducedCoordinate";
        if (named(name, "inboundJoint")) return "PxArticulationJointReducedCoordinate";
    }
    if (kind == ePxScene)
    {
        list = true;
        if (named(name, "actors")) return "PxActor";
        if (named(name, "articulations")) return "PxArticulationReducedCoordinate";
        if (named(name, "constraints")) return "PxConstraint";
    }
    if (kind == ePxPhysics && named(name, "scenes")) { list = true; return "PxScene"; }
    list = false;
    if (kind == ePxArticulationJoint && (named(name, "parentLink") || named(name, "childLink"))) return "PxArticulationLink";
    if (kind == ePxJoint && named(name, "constraint")) return "PxConstraint";
    if (kind == ePxShape && named(name, "geom")) return "PxGeometry";
    if (kind == ePxGeomConvexMesh && named(name, "convexmesh")) return "PxConvexMesh";
    if (kind == ePxGeomHeightfield && named(name, "heightfield")) return "PxHeightField";
    if (kind == ePxGeomTriangleMesh && named(name, "trianglemesh")) return "PxTriangleMesh";
    if (kind == ePxGeomTetMesh && named(name, "tetrahedronMesh")) return "PxTetrahedronMesh";
    return nullptr;
}

bool canParent(OmniPvdObject* parent, OmniPvdObject* child, const OmniPvdDOMState& dom, Error& error)
{
    return detail::canParent(parent, child, dom.mObjectCreations.size()) ||
        fail(error, Error::Code::Invalid, "Cyclic object hierarchy");
}

bool cachedParent(OmniPvdObject& object, OmniPvdDOMState& dom, OmniPvdObject*& parent, Error& error)
{
    int32_t attribute = -1, cls = -1;
    OmniPvdAttributeInstList* list = getAttribList(attribute, cls, "parentLink", &object);
    parent = nullptr;
    if (!list || !list->mFirst || list->mAttributeDef->mIsUniqueList) return true;
    auto* sample = static_cast<OmniPvdAttributeSample*>(list->mFirst);
    if (sample->mDataLen != sizeof(OmniPvdObjectHandle)) return true;
    OmniPvdObjectHandle handle = 0;
    memcpy(&handle, sample->mData, sizeof(handle));
    parent = findOmniPvdObject(getInternalHandle(handle, dom.mExternalToInternalHandleMap), dom.mObjectHandleToObjectMap);
    return !parent || isKind(*parent->mOmniPvdClass, "PxArticulationLink") ||
        fail(error, Error::Code::Invalid, "Incompatible cached parent link target");
}

bool validateArticulation(OmniPvdObject& joint, OmniPvdObject& child, OmniPvdDOMState& dom, Error& error)
{
    OmniPvdObject* parent = nullptr;
    if (!cachedParent(joint, dom, parent, error)) return false;
    return detail::canParentArticulation(parent, &child, &joint, dom.mObjectCreations.size()) ||
        fail(error, Error::Code::Invalid, "Cyclic object hierarchy");
}

bool validateEdge(OmniPvdReader& reader, OmniPvdCommand::Enum command, OmniPvdObject& object,
                  const OmniPvdAttributeDef& attribute, OmniPvdDOMState& dom, Budget& budget, PendingEdges& pending)
{
    const auto kind = object.mOmniPvdClass->mPhysXBaseProcessingClassId;
    const auto& name = attribute.mAttributeName;
    bool list = false;
    const char* expected = edgeTarget(kind, name, list);
    if (!expected) return true;
    if (attribute.mDataType != OmniPvdDataType::eOBJECT_HANDLE || list != (attribute.mIsUniqueList != 0))
        return fail(budget.error, Error::Code::Invalid, "Incorrect schema for object edge");
    // Constraint handles correlate scenes and joints without requiring a streamed constraint object.
    if ((kind == ePxScene && named(name, "constraints")) || (kind == ePxJoint && named(name, "constraint"))) return true;
    uint64_t handle = 0;
    memcpy(&handle, reader.getAttributeDataPointer(), sizeof(handle));
    if (!handle) return !list || fail(budget.error, Error::Code::Invalid, "Null unique-list edge");
    OmniPvdObject* target = findOmniPvdObject(getInternalHandle(handle, dom.mExternalToInternalHandleMap), dom.mObjectHandleToObjectMap);
    if (!target)
    {
        if (command == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE) return true;
        if (!budget.charge(192)) return false;
        const bool required = kind == ePxActor && (named(name, "inboundJoint") || named(name, "articulation"));
        pending[handle].push_back({expected, budget.error.byteOffset, budget.error.commandIndex, !required});
        return true;
    }
    if (!isKind(*target->mOmniPvdClass, expected))
        return fail(budget.error, Error::Code::Invalid, "Incompatible object edge target");
    if (command == OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE) return true;
    if (kind == ePxArticulationJoint && named(name, "parentLink")) return true;
    if (kind == ePxArticulationJoint && named(name, "childLink"))
        return validateArticulation(object, *target, dom, budget.error);
    if (kind == ePxActor && named(name, "inboundJoint"))
        return validateArticulation(*target, object, dom, budget.error);
    if (kind == ePxActor && named(name, "articulation")) return canParent(target, &object, dom, budget.error);
    if (named(name, "shapes") && target->mIsShared) return true;
    if (named(name, "constraint") || named(name, "constraints") ||
        kind == ePxGeomConvexMesh || kind == ePxGeomHeightfield || kind == ePxGeomTriangleMesh || kind == ePxGeomTetMesh) return true;
    return canParent(&object, target, dom, budget.error);
}

bool chargeReopenedLifespans(OmniPvdObject& root, Budget& budget)
{
    const uint64_t closedAt = root.mLifeSpans.empty() ? 0 : root.mLifeSpans.back().mFrameStop;
    if (root.mLifeSpans.empty() || closedAt)
        if (!budget.charge(2 * sizeof(OmniPvdObjectLifeSpan))) return false;
    if (!closedAt) return true;
    // Match reopenDescendantsClosedAt, including its pruning of detached descendants.
    OmniPvdObject* node = root.mFirstChild;
    while (node)
    {
        const bool reopen = node->mReopenable && !node->mLifeSpans.empty() && node->mLifeSpans.back().mFrameStop == closedAt;
        if (reopen && !budget.charge(2 * sizeof(OmniPvdObjectLifeSpan))) return false;
        if (reopen && node->mFirstChild) node = node->mFirstChild;
        else
        {
            while (node != &root && !node->mNextSibling) node = node->mAncestor;
            if (node == &root) break;
            node = node->mNextSibling;
        }
    }
    return true;
}

bool chargeSubtreeReopening(OmniPvdReader& reader, OmniPvdCommand::Enum command, const OmniPvdObject& object,
                           const OmniPvdAttributeDef& attribute, OmniPvdDOMState& dom, Budget& budget)
{
    if (command != OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE) return true;
    const auto kind = object.mOmniPvdClass->mPhysXBaseProcessingClassId;
    const auto& name = attribute.mAttributeName;
    const bool shapes = named(name, "shapes") && (kind == ePxActor || kind == ePxDeformableVolume || kind == ePxDeformableSurface);
    if (!shapes && !(kind == ePxActor && named(name, "particleBuffers")) &&
        !(kind == ePxScene && (named(name, "actors") || named(name, "articulations")))) return true;
    uint64_t handle = 0;
    memcpy(&handle, reader.getAttributeDataPointer(), sizeof(handle));
    OmniPvdObject* target = findOmniPvdObject(getInternalHandle(handle, dom.mExternalToInternalHandleMap), dom.mObjectHandleToObjectMap);
    // Shared shapes reopen one reference node, covered by the per-handle allowance.
    return !target || (shapes && target->mIsShared) || chargeReopenedLifespans(*target, budget);
}

bool validateSamplePayload(OmniPvdReader& reader, OmniPvdCommand::Enum command, OmniPvdDOMState& dom, Budget& budget, PendingEdges& pending, bool& ignored)
{
    if (getOmniPvdReaderStatus(reader).attributePathDepth != 1)
        return fail(budget.error, Error::Code::Unsupported, "Nested attribute paths are unsupported");
    OmniPvdObject* object = findOmniPvdObject(getInternalHandle(reader.getObjectHandle(), dom.mExternalToInternalHandleMap),
                                            dom.mObjectHandleToObjectMap);
    OmniPvdAttributeDef* attribute = findOmniPvdAttribute(reader.getAttributeHandle(), dom.mAttributeHandleToAttributeMap);
    if (!object || !attribute)
    {
        ignored = true;
        return true;
    }
    const int32_t index = findOmniPvdClassOffset(object->mOmniPvdClass->mOmniClassHandle,
                                                attribute->mClass->mOmniClassHandle, dom.mInheritedClassToClassOffset);
    if (index < 0 || static_cast<size_t>(index) >= object->mInheritedClassInstances.size())
        return fail(budget.error, Error::Code::Invalid, "Attribute does not belong to object");
    const bool list = attribute->mIsUniqueList != 0;
    if (list == (command == OmniPvdCommand::eSET_ATTRIBUTE) || attribute->mDataType == OmniPvdDataType::eENUM_VALUE)
        return fail(budget.error, Error::Code::Invalid, "Command does not match attribute kind");
    const uint64_t size = reader.getAttributeDataLength();
    const auto type = static_cast<OmniPvdDataType::Enum>(attribute->mDataType);
    // FLAGS_WORD omits storage width. SDK PxFlags payloads use one, two or four bytes.
    const uint64_t width = type == OmniPvdDataType::eFLAGS_WORD ? size : getOmniPvdDataTypeSizeFromEnum(type);
    if (type == OmniPvdDataType::eFLAGS_WORD && size != 1 && size != 2 && size != 4)
        return fail(budget.error, Error::Code::Invalid, "Invalid flags storage width");
    const uint64_t fields = type == OmniPvdDataType::eFLAGS_WORD || list ? 1 : attribute->mNbrFields;
    if (type != OmniPvdDataType::eSTRING && (!width || size % width || (fields && size != fields * width)))
        return fail(budget.error, Error::Code::Invalid, "Attribute payload does not match schema");
    const uint32_t required = handlerWidth(object->mOmniPvdClass->mPhysXBaseProcessingClassId, attribute->mAttributeName);
    if (required && (size != required || width != required || (!list && fields != 1)))
        return fail(budget.error, Error::Code::Invalid, "Invalid schema or payload for fixed-width DOM handler");
    const auto kind = object->mOmniPvdClass->mPhysXBaseProcessingClassId;
    if (required == 4 && (list || (type != OmniPvdDataType::eUINT32 && type != OmniPvdDataType::eFLAGS_WORD)))
        return fail(budget.error, Error::Code::Invalid, "Incorrect scalar schema for DOM handler");
    if (kind == ePxShape && named(attribute->mAttributeName, "isExclusive") &&
        (list || type != OmniPvdDataType::eUINT8 || reader.getAttributeDataPointer()[0] > 1))
        return fail(budget.error, Error::Code::Invalid, "Invalid shape boolean");
    if (list && (named(attribute->mAttributeName, "parentLink") || named(attribute->mAttributeName, "articulation") ||
                 named(attribute->mAttributeName, "ovdIntegrationVersionMajor") || named(attribute->mAttributeName, "ovdIntegrationVersionMinor")))
        return fail(budget.error, Error::Code::Invalid, "Scalar DOM field declared as a list");
    if (named(attribute->mAttributeName, "ovdIntegrationVersionMajor") && size == 4)
    {
        uint32_t version = 0;
        memcpy(&version, reader.getAttributeDataPointer(), 4);
        if (version > PVDDOM_ACCEPTED_OVD_INTEG_VERSION_MAJOR)
            return fail(budget.error, Error::Code::Unsupported, "Unsupported PhysX integration version");
    }
    if (!validateEdge(reader, command, *object, *attribute, dom, budget, pending)) return false;
    const auto& slots = object->mInheritedClassInstances[static_cast<size_t>(index)].mClassAttributeLists;
    if (attribute->mClassOffset >= slots.size() && !budget.charge(attribute->mClassOffset + 1ull - slots.size(), 2 * sizeof(void*)))
        return false;
    if (!budget.charge(256) || !budget.charge(size, 2)) return false;
    if (required == 8 || (type == OmniPvdDataType::eOBJECT_HANDLE && object->mOmniPvdClass->mIsDefaultParsed))
    {
        if (!budget.charge(8192) || !budget.charge(size / 8, 2048) ||
            !chargeSubtreeReopening(reader, command, *object, *attribute, dom, budget)) return false;
    }
    ++budget.stats.samples;
    return true;
}

void collectMessage(OmniPvdReader& reader, OmniPvdDOMState& dom, OmniPvdMessages& messages)
{
    OmniPvdMessage message = {};
    const char* text = nullptr;
    const char* file = nullptr;
    reader.getMessageData(text, file, message.line, message.type, message.handle);
    snprintf(message.message, sizeof(message.message), "%s", text);
    snprintf(message.file, sizeof(message.file), "%s", file);
    message.frameId = dom.mLatestStartedFrame;
    const auto cls = dom.mClassHandleToClassMap.find(message.handle);
    if (cls != dom.mClassHandleToClassMap.end())
        for (const OmniPvdAttributeDef* attribute : cls->second->mAttributeDefinitions)
            if (attribute->mNbrFields == message.type)
            {
                snprintf(message.typeName, sizeof(message.typeName), "%s", attribute->mAttributeName.c_str());
                break;
            }
    messages.push_back(message);
}

using SceneClocks = std::map<std::pair<uint64_t, uint64_t>, uint64_t>;

OmniPvdObject* objectByHandle(uint64_t handle, OmniPvdDOMState& dom)
{
    return findOmniPvdObject(getInternalHandle(handle, dom.mExternalToInternalHandleMap), dom.mObjectHandleToObjectMap);
}

void membershipRoots(OmniPvdReader& reader, OmniPvdCommand::Enum command, OmniPvdDOMState& dom,
                     const OmniPvdObject* (&roots)[3])
{
    OmniPvdObject* object = objectByHandle(reader.getObjectHandle(), dom);
    if (command == OmniPvdCommand::eCREATE_OBJECT || command == OmniPvdCommand::eDESTROY_OBJECT)
    { roots[0] = object; return; }
    if (!object || (command != OmniPvdCommand::eSET_ATTRIBUTE &&
        command != OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE && command != OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE)) return;
    const auto* attribute = findOmniPvdAttribute(reader.getAttributeHandle(), dom.mAttributeHandleToAttributeMap);
    const auto kind = object->mOmniPvdClass->mPhysXBaseProcessingClassId;
    const auto& name = attribute->mAttributeName;
    if (kind == ePxShape && named(name, "isExclusive"))
    { if (!reader.getAttributeDataPointer()[0]) roots[0] = object; return; }
    if (attribute->mDataType != OmniPvdDataType::eOBJECT_HANDLE || reader.getAttributeDataLength() != 8) return;
    uint64_t handle = 0;
    memcpy(&handle, reader.getAttributeDataPointer(), sizeof(handle));
    OmniPvdObject* target = objectByHandle(handle, dom);
    const bool list = command != OmniPvdCommand::eSET_ATTRIBUTE;
    if (kind == ePxScene && list)
    {
        if (named(name, "actors") || named(name, "articulations")) roots[0] = target;
        else if (named(name, "constraints"))
        {
            const auto joint = dom.mConstraintToJointMap.find(handle);
            if (joint != dom.mConstraintToJointMap.end()) roots[0] = joint->second;
        }
    }
    else if (kind == ePxActor || kind == ePxDeformableVolume || kind == ePxDeformableSurface)
    {
        if (list && ((named(name, "shapes") && target && !target->mIsShared) ||
            (kind == ePxActor && named(name, "particleBuffers")))) roots[0] = target;
        else if (kind == ePxActor && !list && named(name, "articulation")) roots[0] = object;
        else if (kind == ePxActor && !list && named(name, "inboundJoint"))
        { roots[0] = object; roots[1] = target; }
    }
    else if (kind == ePxShape && !list && named(name, "geom")) roots[0] = target;
    else if (kind == ePxArticulationJoint && !list && named(name, "childLink"))
    { roots[0] = target; roots[1] = object; }
    else if (kind == ePxJoint && !list && named(name, "constraint")) roots[0] = object;
}

bool appendMembershipId(std::vector<uint64_t>& ids, uint64_t id, Budget& budget)
{
    if (ids.size() == ids.capacity())
    {
        const size_t growth = (std::max)(size_t(8), ids.capacity());
        if (growth > ids.max_size() - ids.capacity())
            return fail(budget.error, Error::Code::Limit, "Membership scratch limit exceeded");
        if (!budget.charge(growth, 2 * sizeof(uint64_t))) return false;
        ids.reserve(ids.capacity() + growth);
    }
    ids.push_back(id);
    return true;
}

bool collectMembershipSubtree(const OmniPvdObject* root, const OmniPvdDOMState& dom,
                              std::vector<uint64_t>& affected, Budget& budget)
{
    // The ownership list includes synthetic branches and references. Do not follow reference targets.
    const size_t bound = dom.mObjectCreations.size();
    size_t visits = 0, ascents = 0;
    const OmniPvdObject* node = root;
    while (node)
    {
        if (++visits > bound) return fail(budget.error, Error::Code::Invalid, "Cyclic membership subtree");
        if (!budget.increment(budget.stats.membershipVisits, UINT64_MAX, "Membership visit limit exceeded", Error::LimitKind::None)) return false;
        if (node->mUID && !appendMembershipId(affected, node->mUID, budget)) return false;
        if (node->mFirstMembershipChild) node = node->mFirstMembershipChild;
        else
        {
            while (node != root)
            {
                if (node->mNextMembershipSibling) { node = node->mNextMembershipSibling; break; }
                if (++ascents > bound || !node->mAncestor)
                    return fail(budget.error, Error::Code::Invalid, "Invalid membership ancestry");
                node = node->mAncestor;
            }
            if (node == root) break;
        }
    }
    return true;
}

struct GenericCandidate
{
    uint64_t id;
    bool previouslyReferenced;
};

bool collectGenericMembership(OmniPvdReader& reader, OmniPvdCommand::Enum command, OmniPvdDOMState& dom,
                              std::vector<uint64_t>& roots, std::vector<GenericCandidate>& candidates,
                              std::vector<uint64_t>& affected, Budget& budget)
{
    if (command != OmniPvdCommand::eSET_ATTRIBUTE && command != OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE) return true;
    const auto* object = objectByHandle(reader.getObjectHandle(), dom);
    if (!object->mOmniPvdClass->mIsDefaultParsed) return true;
    const auto* attribute = findOmniPvdAttribute(reader.getAttributeHandle(), dom.mAttributeHandleToAttributeMap);
    if (attribute->mDataType != OmniPvdDataType::eOBJECT_HANDLE) return true;
    const uint64_t key = (object->mOmniObjectHandle << 32) | reader.getAttributeHandle();
    roots.clear();
    candidates.clear();
    for (uint32_t offset = 0; offset < reader.getAttributeDataLength(); offset += sizeof(uint64_t))
    {
        uint64_t handle = 0;
        memcpy(&handle, reader.getAttributeDataPointer() + offset, sizeof(handle));
        const auto* target = objectByHandle(handle, dom);
        if (!target || !target->mOmniPvdClass->mIsDefaultParsed) continue;
        // Generic reference entries retain the streamed target, including an older incarnation.
        const auto mapped = dom.mObjectAttributeRefMap.find({key, handle});
        const bool previouslyReferenced = mapped != dom.mObjectAttributeRefMap.end() && mapped->second;
        if (previouslyReferenced) target = mapped->second;
        if (target->mUID && (target->mLifeSpans.empty() || target->mLifeSpans.back().mFrameStop))
        {
            if (candidates.size() == candidates.capacity())
            {
                const size_t growth = (std::max)(size_t(8), candidates.capacity());
                if (growth > candidates.max_size() - candidates.capacity())
                    return fail(budget.error, Error::Code::Limit, "Membership scratch limit exceeded");
                if (!budget.charge(growth, 2 * sizeof(GenericCandidate))) return false;
                candidates.reserve(candidates.capacity() + growth);
            }
            candidates.push_back({target->mUID, previouslyReferenced});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const GenericCandidate& a, const GenericCandidate& b)
    { return a.id < b.id; });
    for (size_t i = 0; i < candidates.size();)
    {
        size_t end = i + 1;
        bool previouslyReferenced = candidates[i].previouslyReferenced;
        while (end < candidates.size() && candidates[end].id == candidates[i].id)
        { previouslyReferenced = previouslyReferenced || candidates[end].previouslyReferenced; ++end; }
        if ((previouslyReferenced || end - i > 1) && !appendMembershipId(roots, candidates[i].id, budget)) return false;
        i = end;
    }
    std::sort(roots.begin(), roots.end());
    roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
    for (uint64_t id : roots)
    {
        const auto* root = findOmniPvdObject(id, dom.mObjectHandleToObjectMap);
        const auto* ancestor = root->mAncestor;
        size_t depth = 0;
        while (ancestor && !std::binary_search(roots.begin(), roots.end(), ancestor->mUID))
        {
            if (++depth > dom.mObjectCreations.size())
                return fail(budget.error, Error::Code::Invalid, "Cyclic membership ancestry");
            ancestor = ancestor->mAncestor;
        }
        // Reopening changes closed state, not stream-object parentage. Collect each subtree once before apply.
        if (!ancestor && !collectMembershipSubtree(root, dom, affected, budget)) return false;
    }
    return true;
}

const OmniPvdObject* currentScene(const OmniPvdObject* object, const OmniPvdDOMState& dom)
{
    size_t depth = 0;
    for (const OmniPvdObject* node = object; node && ++depth <= dom.mObjectCreations.size(); node = node->mAncestor)
    {
        if (node->mUID && (!node->mLifeSpans.empty() && node->mLifeSpans.back().mFrameStop)) return nullptr;
        if (node->mOmniPvdClass && node->mOmniPvdClass->mIsSceneClass) return node;
    }
    return nullptr;
}

uint64_t sceneClock(const SceneClocks& clocks, uint64_t segment, uint64_t context)
{
    const auto found = clocks.find({segment, context});
    return found == clocks.end() ? 0 : found->second;
}

bool updateMembership(ObjectHistory& record, OmniPvdDOMState& dom, const SceneClocks& clocks, Budget& budget)
{
    if (!budget.increment(budget.stats.membershipVisits, UINT64_MAX, "Membership visit limit exceeded", Error::LimitKind::None)) return false;
    const uint64_t segment = dom.mCurrentRecordingSegmentId;
    const OmniPvdObject* scene = record.object->mRecordingSegmentId == segment ? currentScene(record.object, dom) : nullptr;
    const uint64_t context = scene ? scene->mOmniAPIHandle : 0;
    if (!record.memberships.empty() && !record.memberships.back().stopOrdinal)
    {
        SceneMembership& last = record.memberships.back();
        if (last.recordingSegment == segment && last.context == context) return true;
        last.stopOrdinal = budget.stats.commands;
        last.endFrame = sceneClock(clocks, last.recordingSegment, last.context);
        if (last.recordingSegment != segment) ++last.endFrame;
    }
    if (context)
    {
        if (!budget.charge(2 * sizeof(SceneMembership))) return false;
        SceneMembership member;
        member.recordingSegment = segment;
        member.context = context;
        member.startFrame = sceneClock(clocks, segment, context);
        member.startCommand = budget.stats.commands;
        record.memberships.push_back(member);
    }
    return true;
}

SampleWrite captureWrite(OmniPvdReader& reader, OmniPvdDOMState& dom, const SceneClocks& clocks, uint64_t command)
{
    SampleWrite write;
    write.recordingSegment = dom.mCurrentRecordingSegmentId;
    write.rawContext = reader.getContextHandle();
    write.command = command;
    const auto raw = clocks.find({write.recordingSegment, write.rawContext});
    write.hasRawFrame = raw != clocks.end();
    if (write.hasRawFrame) write.rawFrame = raw->second;
    OmniPvdObject* object = findOmniPvdObject(getInternalHandle(reader.getObjectHandle(), dom.mExternalToInternalHandleMap),
                                             dom.mObjectHandleToObjectMap);
    const OmniPvdObject* scene = currentScene(object, dom);
    write.context = scene ? scene->mOmniAPIHandle : 0;
    const auto effective = clocks.find({write.recordingSegment, write.context});
    write.frameKnown = scene && effective != clocks.end();
    write.frame = write.frameKnown ? effective->second : 0;
    write.attribute = findOmniPvdAttribute(reader.getAttributeHandle(), dom.mAttributeHandleToAttributeMap);
    return write;
}
}
struct Document::Storage
{
    std::unique_ptr<OmniPvdDOMState> dom;
    std::vector<RecordedFrame> frames;
    LoadStats stats;
    OmniPvdMessages messages;
    std::map<uint64_t, ObjectHistory> history;
    std::map<const ObjectHistory*, size_t> openMembershipIndices;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t, uint64_t>, std::vector<size_t>> scopedWrites;
    std::map<std::pair<uint64_t, uint64_t>, FrameIndex> frameIndex;
};
const std::vector<size_t>* DocumentAccess::scopedWrites(const Document& document, const ObjectHistory& record,
    uint64_t attribute, uint64_t recordingSegment, uint64_t context)
{
    const auto& index = document.mStorage->scopedWrites;
    const auto found = index.find(std::make_tuple(record.object->mUID, attribute, recordingSegment, context));
    return found == index.end() ? nullptr : &found->second;
}
Document::Document() : mStorage(new Storage) {}
Document::~Document() = default;
const OmniPvdDOMState* Document::dom() const { return mStorage->dom.get(); }
const std::vector<RecordedFrame>& Document::frames() const { return mStorage->frames; }
const LoadStats& Document::stats() const { return mStorage->stats; }
const OmniPvdMessages& Document::messages() const { return mStorage->messages; }
const std::map<uint64_t, ObjectHistory>& Document::history() const { return mStorage->history; }
const std::map<std::pair<uint64_t, uint64_t>, FrameIndex>& Document::frameIndex() const { return mStorage->frameIndex; }
const LoadInfo& Document::loadInfo() const { return mLoadInfo; }
bool Document::load(const char* path, const LoadLimits& limits, Error& error)
{
    return load(path, limits, error, LoadMode::Strict, OmniPvdReaderDiagnostics());
}
bool Document::load(const char* path, const LoadLimits& limits, Error& error, LoadMode mode)
{
    return load(path, limits, error, mode, OmniPvdReaderDiagnostics());
}
bool Document::load(const char* path, const LoadLimits& limits, Error& error,
                    const OmniPvdReaderDiagnostics& diagnostics)
{
    return load(path, limits, error, LoadMode::Strict, diagnostics);
}
bool Document::load(const char* path, const LoadLimits& limits, Error& error, LoadMode mode,
                    const OmniPvdReaderDiagnostics& diagnostics)
{
    mStorage->dom.reset();
    mStorage->history.clear();
    mStorage->openMembershipIndices.clear();
    mStorage->scopedWrites.clear();
    mStorage->frameIndex.clear();
    std::vector<RecordedFrame>().swap(mStorage->frames);
    OmniPvdMessages().swap(mStorage->messages);
    mStorage->stats = LoadStats();
    mLoadInfo = LoadInfo();
    error = Error();
    if (!path || !*path) return fail(error, Error::Code::Io, "Missing file path");
    try
    {
        FileStream stream(path, limits.maxStreamBytes);
        if (!stream.file) return fail(error, Error::Code::Io, "Cannot open or size file");
        if (stream.size > limits.maxStreamBytes)
            return fail(error, Error::Code::Limit, "Stream-byte limit exceeded", Error::LimitKind::StreamBytes);
        std::unique_ptr<Storage> storage(new Storage);
        Budget budget(limits, storage->stats, error);
        if (!budget.charge(32768)) return false;
        storage->dom.reset(new OmniPvdDOMState);
        OmniPvdDOMState& dom = *storage->dom;
        const uint64_t payloadCap = (std::min)((std::min)(limits.maxAttributeBytes, limits.maxRetainedBytes), uint64_t(UINT32_MAX));
        std::unique_ptr<OmniPvdReader, ReaderDeleter> reader(createOmniPvdReaderWithLimits(
            static_cast<uint32_t>(payloadCap)));
        if (!reader)
            return fail(error, Error::Code::Limit, "Reader allocation failed");
        setOmniPvdReaderDiagnostics(*reader, diagnostics);
        reader->setReadStream(stream);
        OmniPvdVersionType major = 0, minor = 0, patch = 0;
        if (!reader->startReading(major, minor, patch))
        {
            const bool io = stream.failure == FileStream::ReadFailure::Io || stream.file.bad();
            return fail(error, io ? Error::Code::Io : getOmniPvdReaderStatus(*reader).code == OmniPvdReaderStatus::eUNSUPPORTED ?
                        Error::Code::Unsupported : Error::Code::Invalid,
                        io ? "File read failed while reading the stream header" : "Invalid or unsupported stream header");
        }
        std::unordered_set<OmniPvdAttributeHandle> attributes;
        ClassAttributeLabels names;
        PendingEdges pending;
        std::map<std::pair<uint64_t, uint64_t>, size_t> openFrames;
        SceneClocks clocks;
        std::vector<uint64_t> affected;
        std::vector<uint64_t> genericRoots;
        std::vector<GenericCandidate> genericCandidates;
        std::vector<ObjectHistory*> openMemberships;
        size_t maxOpenMemberships = 0;
        auto refreshMembership = [&](ObjectHistory& record)
        {
            if (!updateMembership(record, dom, clocks, budget)) return false;
            const bool open = !record.memberships.empty() && !record.memberships.back().stopOrdinal;
            auto position = storage->openMembershipIndices.find(&record);
            if (open && position == storage->openMembershipIndices.end())
            {
                if (openMemberships.size() == maxOpenMemberships)
                {
                    // Reserve the map node and vector growth for this peak open slot.
                    if (!budget.charge(256)) return false;
                    ++maxOpenMemberships;
                }
                openMemberships.push_back(&record);
                storage->openMembershipIndices.emplace(&record, openMemberships.size() - 1);
            }
            else if (!open && position != storage->openMembershipIndices.end())
            {
                ObjectHistory* moved = openMemberships.back();
                openMemberships[position->second] = moved;
                storage->openMembershipIndices.at(moved) = position->second;
                openMemberships.pop_back();
                storage->openMembershipIndices.erase(position);
            }
            return true;
        };
        bool recoveredTail = false;
        Error terminalError;
        while (stream.offset < stream.size)
        {
            error.byteOffset = stream.offset;
            error.commandIndex = storage->stats.commands + 1;
            if (!budget.increment(storage->stats.commands, limits.maxCommands, "Command limit exceeded", Error::LimitKind::Commands)) return false;
            const auto command = reader->getNextCommand();
            const auto status = getOmniPvdReaderStatus(*reader);
            SampleWrite write;
            if (command == OmniPvdCommand::eINVALID)
            {
                // Path rejection precedes payload length decoding. Allocation failures do not exceed the cap.
                const bool payloadOverCap = status.code == OmniPvdReaderStatus::eLIMIT &&
                    status.attributePathDepth > 0 && status.attributePathDepth <= 32 &&
                    reader->getAttributeDataLength() > payloadCap;
                const bool oversizeTail = payloadOverCap && stream.offset <= stream.size &&
                    reader->getAttributeDataLength() > stream.size - stream.offset;
                if (oversizeTail)
                {
                    const uint64_t remaining = stream.size - stream.offset;
                    if (stream.skipBytes(remaining) == remaining && stream.failure == FileStream::ReadFailure::None)
                    {
                        if (!stream.sameSize())
                            return fail(error, stream.file.fail() ? Error::Code::Io : Error::Code::Invalid,
                                        stream.file.fail() ? "File size check failed" : "File changed while loading");
                        stream.failure = FileStream::ReadFailure::PhysicalEof;
                    }
                }
                const Error::Code code = stream.failure == FileStream::ReadFailure::Io || stream.file.bad() ? Error::Code::Io :
                    oversizeTail ? Error::Code::Invalid :
                    status.code == OmniPvdReaderStatus::eLIMIT ? Error::Code::Limit :
                    status.code == OmniPvdReaderStatus::eUNSUPPORTED ? Error::Code::Unsupported : Error::Code::Invalid;
                const char* message = code == Error::Code::Io ? "File read failed before the measured end of file" :
                    oversizeTail ? "Attribute payload extends past end of file" :
                    code == Error::Code::Unsupported ? "Unsupported command" :
                    code == Error::Code::Limit ? (status.attributePathDepth > 32 ? "Attribute path depth exceeds 32" :
                        payloadOverCap ? "Attribute payload exceeds the payload limit" : "Attribute payload allocation failed") :
                    stream.failure == FileStream::ReadFailure::PhysicalEof ? "Command truncated at end of file" :
                    "Invalid command or field";
                fail(error, code, message);
                if (code == Error::Code::Limit && payloadOverCap)
                    error.limitKind = limits.maxAttributeBytes <= limits.maxRetainedBytes ?
                        Error::LimitKind::AttributeBytes : Error::LimitKind::RetainedBytes;
                if (mode != LoadMode::RecoverTruncatedTail || code != Error::Code::Invalid ||
                    (status.code != OmniPvdReaderStatus::eINVALID && !oversizeTail) || stream.failure != FileStream::ReadFailure::PhysicalEof ||
                    stream.offset != stream.size ||
                    storage->stats.commands <= 1 || stream.offset <= error.byteOffset) return false;
                if (!stream.sameSize())
                    return fail(error, stream.file.fail() ? Error::Code::Io : Error::Code::Invalid,
                                stream.file.fail() ? "File size check failed" : "File changed while loading");
                terminalError = error;
                --storage->stats.commands;
                recoveredTail = true;
                break;
            }
            switch (command)
            {
            case OmniPvdCommand::eREGISTER_CLASS:
            case OmniPvdCommand::eREGISTER_ATTRIBUTE:
            case OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE:
            case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:
                if (!validateSchema(*reader, command, dom, attributes, names, budget)) return false;
                break;
            case OmniPvdCommand::eCREATE_OBJECT:
            {
                const auto cls = dom.mClassHandleToClassMap.find(reader->getClassHandle());
                if (!reader->getObjectHandle() || cls == dom.mClassHandleToClassMap.end() || cls->second->mIsEnumClass)
                    return fail(error, Error::Code::Invalid, "Invalid object handle or class");
                const auto deferred = pending.find(reader->getObjectHandle());
                if (deferred != pending.end())
                {
                    for (const PendingEdge& edge : deferred->second)
                        if (!isKind(*cls->second, edge.expected))
                            return fail(error, Error::Code::Invalid, "Incompatible forward-reference target");
                    pending.erase(deferred);
                }
                if (!budget.increment(storage->stats.creates, (std::min)(limits.maxCreates, uint64_t(UINT32_MAX - 1)), "Object-create limit exceeded",
                    limits.maxCreates < uint64_t(UINT32_MAX - 1) ? Error::LimitKind::Creates : Error::LimitKind::None) ||
                    !budget.charge(4096 + 4 * strnlen(reader->getObjectName(), OMNI_PVD_MAX_STRING_LENGTH))) return false;
                for (const OmniPvdClass* base : cls->second->mInheritanceChain)
                    if (!budget.charge(64) || !budget.charge(base->mAttributeDefinitions.size(), 2 * sizeof(void*))) return false;
                // The handler appends this articulation before scanning the list.
                if (cls->second == dom.mPxArticulationReducedCoordinateClass && dom.mPxArticulationLinkClass &&
                    (!budget.handlerSteps(dom.mObjectCreations.size()) || !budget.handlerSteps(1))) return false;
                break;
            }
            case OmniPvdCommand::eSET_ATTRIBUTE:
            case OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE:
            case OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE:
            {
                bool ignored = false;
                if (!validateSamplePayload(*reader, command, dom, budget, pending, ignored)) return false;
                if (ignored)
                {
                    ++storage->stats.ignoredCommands;
                    continue;
                }
                if (command == OmniPvdCommand::eSET_ATTRIBUTE)
                {
                    if (!budget.charge(2 * sizeof(SampleWrite))) return false;
                    write = captureWrite(*reader, dom, clocks, storage->stats.commands);
                }
                break;
            }
            case OmniPvdCommand::eDESTROY_OBJECT:
                if (!dom.mExternalToInternalHandleMap.count(reader->getObjectHandle()))
                {
                    ++storage->stats.ignoredCommands;
                    continue;
                }
                break;
            case OmniPvdCommand::eSTART_FRAME:
            {
                if (reader->getFrameTimeStart() == UINT64_MAX)
                    return fail(error, Error::Code::Invalid, "Frame number cannot represent a closing boundary");
                if (storage->frames.size() >= limits.maxFrameRecords)
                    return fail(error, Error::Code::Limit, "Frame-record limit exceeded", Error::LimitKind::FrameRecords);
                if (!budget.charge(192)) return false;
                if ((dom.mStreamOvdIntegVersionMajor < 1 ||
                     (dom.mStreamOvdIntegVersionMajor == 1 && dom.mStreamOvdIntegVersionMinor < 4)) &&
                    !budget.handlerSteps(dom.mSceneCreations.size())) return false;
                const auto key = std::make_pair(dom.mCurrentRecordingSegmentId, reader->getContextHandle());
                if (!clocks.count(key) && !budget.charge(128)) return false;
                clocks[key] = reader->getFrameTimeStart();
                if ((!storage->frameIndex.count(key) && !budget.charge(128)) || !budget.charge(2 * sizeof(uint64_t))) return false;
                FrameIndex& frameIndex = storage->frameIndex[key];
                if (!frameIndex.recordedFrames.empty() && reader->getFrameTimeStart() <= frameIndex.recordedFrames.back())
                    frameIndex.monotonic = false;
                frameIndex.recordedFrames.push_back(reader->getFrameTimeStart());
                const auto* contextObject = findOmniPvdObject(getInternalHandle(key.second, dom.mExternalToInternalHandleMap),
                                                              dom.mObjectHandleToObjectMap);
                frameIndex.sceneMapped = frameIndex.sceneMapped || (contextObject && contextObject->mOmniPvdClass->mIsSceneClass &&
                    contextObject->mRecordingSegmentId == key.first);
                openFrames[key] = storage->frames.size();
                storage->frames.push_back({key.first, key.second, reader->getFrameTimeStart(), reader->getFrameTimeStart(), false});
                break;
            }
            case OmniPvdCommand::eSTOP_FRAME:
            {
                const auto key = std::make_pair(dom.mCurrentRecordingSegmentId, reader->getContextHandle());
                const auto frame = openFrames.find(key);
                if (frame == openFrames.end()) return fail(error, Error::Code::Invalid, "Frame stop without matching start");
                storage->frames[frame->second].completed = true;
                openFrames.erase(frame);
                break;
            }
            case OmniPvdCommand::eRECORD_MESSAGE:
                if (!budget.charge(2 * sizeof(OmniPvdMessage))) return false;
                collectMessage(*reader, dom, storage->messages);
                continue;
            default: return fail(error, Error::Code::Unsupported, "Unsupported command");
            }
            const uint64_t segmentBeforeCommand = dom.mCurrentRecordingSegmentId;
            const OmniPvdObject* roots[3] = {};
            membershipRoots(*reader, command, dom, roots);
            affected.clear();
            for (const auto* root : roots)
                if (!collectMembershipSubtree(root, dom, affected, budget)) return false;
            if (!collectGenericMembership(*reader, command, dom, genericRoots, genericCandidates, affected, budget)) return false;
            const uint64_t jointSteps = dom.mJointLookupSteps;
            applyPvdDomCommand(*reader, dom, command);
            if (!budget.handlerSteps(dom.mJointLookupSteps - jointSteps)) return false;
            if (command == OmniPvdCommand::eCREATE_OBJECT)
            {
                if (!budget.charge(256)) return false;
                const uint64_t id = getInternalHandle(reader->getObjectHandle(), dom.mExternalToInternalHandleMap);
                ObjectHistory& record = storage->history[id];
                record.object = findOmniPvdObject(id, dom.mObjectHandleToObjectMap);
                if (record.object->mOmniPvdClass->mIsSceneClass)
                {
                    const auto key = std::make_pair(record.object->mRecordingSegmentId, record.object->mOmniAPIHandle);
                    auto index = storage->frameIndex.find(key);
                    if (index != storage->frameIndex.end())
                    {
                        if (index->second.sceneMapped)
                        {
                            index->second.sceneContextRecreated = true;
                            // Superseded memberships must close before the old scene clock is discarded.
                            for (uint64_t oldId : affected)
                            {
                                const auto old = storage->history.find(oldId);
                                if (old != storage->history.end() && !refreshMembership(old->second)) return false;
                            }
                            clocks.erase(key);
                            affected.clear();
                            roots[0] = nullptr;
                        }
                        index->second.sceneMapped = true;
                    }
                }
                // New articulations can adopt links that were previously floating roots.
                roots[2] = record.object;
            }
            if (command == OmniPvdCommand::eSET_ATTRIBUTE)
            {
                const uint64_t id = getInternalHandle(reader->getObjectHandle(), dom.mExternalToInternalHandleMap);
                ObjectHistory& record = storage->history.at(id);
                const auto* object = record.object;
                const int32_t index = findOmniPvdClassOffset(object->mOmniPvdClass->mOmniClassHandle,
                    write.attribute->mClass->mOmniClassHandle, dom.mInheritedClassToClassOffset);
                write.sample = static_cast<const OmniPvdAttributeSample*>(
                    object->mInheritedClassInstances[index].mClassAttributeLists[write.attribute->mClassOffset]->mLast);
                const uint64_t attribute = write.attribute->mOmniAttributeHandle;
                const auto scope = std::make_tuple(id, attribute, write.recordingSegment, write.context);
                if ((!record.writes.count(attribute) && !budget.charge(128)) ||
                    (!storage->scopedWrites.count(scope) && !budget.charge(192)) || !budget.charge(2 * sizeof(size_t))) return false;
                auto& writes = record.writes[attribute];
                storage->scopedWrites[scope].push_back(writes.size());
                writes.push_back(write);
            }
            if (segmentBeforeCommand != dom.mCurrentRecordingSegmentId)
            {
                for (ObjectHistory* record : openMemberships)
                {
                    if (!updateMembership(*record, dom, clocks, budget)) return false;
                }
                openMemberships.clear();
                storage->openMembershipIndices.clear();
            }
            else
            {
                for (const auto* root : roots)
                    if (!collectMembershipSubtree(root, dom, affected, budget)) return false;
                std::sort(affected.begin(), affected.end());
                affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
                for (uint64_t id : affected)
                {
                    const auto entry = storage->history.find(id);
                    if (entry != storage->history.end() && !refreshMembership(entry->second)) return false;
                }
            }
            if (dom.mOvdIntegVersionWasChecked && !dom.mOvdIntegVersionPassed)
                return fail(error, Error::Code::Unsupported, "Unsupported PhysX integration version");
        }
        if (stream.file.bad()) return fail(error, Error::Code::Io, "File read failed");
        if (!stream.sameSize())
            return fail(error, stream.file.fail() ? Error::Code::Io : Error::Code::Invalid,
                        stream.file.fail() ? "File size check failed" : "File changed while loading");
        const PendingEdge* earliest = nullptr;
        for (const auto& target : pending)
            for (const PendingEdge& edge : target.second)
                if (edge.optional) ++storage->stats.unresolvedOptionalEdges;
                else if (!earliest || edge.command < earliest->command) earliest = &edge;
        if (earliest)
        {
            error.byteOffset = earliest->offset;
            error.commandIndex = earliest->command;
            return fail(error, Error::Code::Invalid, "Unresolved forward reference at end of file");
        }
        storage->stats.streamBytes = recoveredTail ? terminalError.byteOffset : stream.offset;
        LoadInfo completedInfo;
        completedInfo.recoveredTail = recoveredTail;
        completedInfo.sourceBytes = stream.size;
        completedInfo.terminalError = terminalError;
        error = Error();
        static_assert(std::is_nothrow_move_assignable<LoadInfo>::value, "LoadInfo publication must not throw");
        mLoadInfo = std::move(completedInfo);
        mStorage.swap(storage);
        return true;
    }
    catch (const std::bad_alloc&) { return fail(error, Error::Code::Limit, "Allocation failed"); }
    catch (const std::length_error&) { return fail(error, Error::Code::Limit, "Container size limit exceeded"); }
}
}
