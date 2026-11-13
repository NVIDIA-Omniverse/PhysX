// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PvdDomJson.h"
#include "PvdDomInspectionSchema.h"
#include "PvdDomInspectionValue.h"
#include "OvdAnnotationFile.h"
#include "OvdCheckedOutput.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace PvdDomInspect;
namespace
{
using Options = std::map<std::string, std::string>;
class LoadFailure : public std::runtime_error
{
public:
    LoadFailure(const char* role, const char* path, const Error& error)
        : std::runtime_error(std::string(role) + " capture '" + path + "': " + error.message +
            " (command " + std::to_string(error.commandIndex) + ")"), role(role), error(error) {}
    std::string role;
    Error error;
};
const char* errorCode(Error::Code code)
{
    switch (code)
    {
    case Error::Code::Io: return "Io";
    case Error::Code::Invalid: return "Invalid";
    case Error::Code::Unsupported: return "Unsupported";
    case Error::Code::Limit: return "Limit";
    case Error::Code::None: return "None";
    }
    return "Invalid";
}
const char* limitOption(const Error& error)
{
    if (error.code != Error::Code::Limit) return nullptr;
    switch (error.limitKind)
    {
    case Error::LimitKind::StreamBytes: return "--max-stream-bytes";
    case Error::LimitKind::Commands: return "--max-commands";
    case Error::LimitKind::Creates: return "--max-creates";
    case Error::LimitKind::Classes: return "--max-classes";
    case Error::LimitKind::Attributes: return "--max-attributes";
    case Error::LimitKind::InheritanceDepth: return "--max-inheritance-depth";
    case Error::LimitKind::AttributeBytes: return "--max-attribute-bytes";
    case Error::LimitKind::FrameRecords: return "--max-frame-records";
    case Error::LimitKind::HandlerSteps: return "--max-handler-steps";
    case Error::LimitKind::RetainedBytes: return "--max-retained-bytes";
    case Error::LimitKind::None: return nullptr;
    }
    return nullptr;
}
std::string loadErrorJson(const LoadFailure& failure)
{
    Json j;
    j.out << "{\"format\":\"physx.ovd.error\",\"version\":1,\"code\":";
    j.text(errorCode(failure.error.code));
    j.out << ",\"message\":"; j.text(failure.error.message);
    j.out << ",\"byteOffset\":"; j.id(failure.error.byteOffset);
    j.out << ",\"commandIndex\":"; j.id(failure.error.commandIndex);
    j.out << ",\"role\":"; j.text(failure.role);
    j.out << ",\"limitOption\":";
    if (const char* option = limitOption(failure.error)) j.text(option); else j.out << "null";
    j.out << ",\"textWasReplaced\":"; j.boolean(j.textWasReplaced);
    j.out << '}';
    return j.out.str();
}
uint64_t natural(const std::string& s, bool positive = false)
{
    if (s.empty()) throw std::runtime_error("Missing integer");
    uint64_t value = 0;
    for (char c : s)
    {
        if (c < '0' || c > '9' || value > (UINT64_MAX - unsigned(c - '0')) / 10)
            throw std::runtime_error("Invalid integer: " + s);
        value = value * 10 + unsigned(c - '0');
    }
    if (positive && !value) throw std::runtime_error("Value must be positive");
    return value;
}
std::string required(const Options& o, const std::string& key)
{
    auto i = o.find(key);
    if (i == o.end() || i->second.empty()) throw std::runtime_error("Required option: " + key);
    return i->second;
}
uint64_t number(const Options& o, const std::string& key) { return natural(required(o, key)); }
ObjectSelector selector(const Options& o, const std::string& prefix)
{
    ObjectSelector s;
    auto id = o.find(prefix + "id"), name = o.find(prefix + "name"), cls = o.find(prefix + "class");
    if ((id == o.end()) == (name == o.end())) throw std::runtime_error("Choose exactly one " + prefix + "id or " + prefix + "name");
    if (id != o.end()) s.objectId = natural(id->second, true);
    else s.name = required(o, prefix + "name");
    if (cls != o.end()) s.className = cls->second;
    return s;
}
void load(Document& doc, const char* path, const LoadLimits& limits, const char* role = "input",
          LoadMode mode = LoadMode::Strict)
{
    Error e;
    if (!doc.load(path, limits, e, mode)) throw LoadFailure(role, path, e);
    if (doc.loadInfo().recoveredTail)
        std::cerr << "Warning: recovered a truncated command tail; capture may be incomplete\n";
}
void loadInfoJson(Json& j, const LoadInfo& info)
{
    j.out << "{\"recoveredTail\":"; j.boolean(info.recoveredTail);
    j.out << ",\"sourceBytes\":"; j.id(info.sourceBytes);
    j.out << ",\"terminalError\":{\"code\":"; j.text(errorCode(info.terminalError.code));
    j.out << ",\"message\":"; j.text(info.terminalError.message);
    j.out << ",\"byteOffset\":"; j.id(info.terminalError.byteOffset);
    j.out << ",\"commandIndex\":"; j.id(info.terminalError.commandIndex);
    j.out << "}}";
}
CaptureProvenance provenance(const char* path)
{
    CaptureProvenance p;
    p.sourcePath = path;
    p.metadata = "Capture SDK version and file hash are not inferred by this tool";
    return p;
}
void saveNew(const std::string& path, const std::string& text)
{
    saveNewOvdAnnotations(path, text,
        [](FILE* file, const std::string& data) { return fwrite(data.data(), 1, data.size(), file) == data.size(); },
        [](FILE* file) { return fclose(file) == 0; });
}
bool writeEvidenceToStdout(const std::string& json)
{
    if (!writeOvdJson(std::cout, json)) { std::cerr << "Could not write stdout\n"; return false; }
    return true;
}
void page(Json& j, uint64_t offset, uint64_t count, bool hasMore, uint64_t total = UINT64_MAX)
{
    auto& out = j.out;
    out << "{\"offset\":"; j.id(offset);
    out << ",\"count\":"; j.id(count);
    out << ",\"hasMore\":"; j.boolean(hasMore);
    out << ",\"nextOffset\":";
    if (hasMore && count <= UINT64_MAX - offset) j.id(offset + count); else out << "null";
    if (total != UINT64_MAX) { out << ",\"total\":"; j.id(total); }
    out << '}';
}
void schemaAttributes(Json& j, const Document& document, const ObjectIdentity& identity,
                      uint64_t offset, uint64_t limit)
{
    std::vector<SchemaAttribute> attributes;
    bool more = false; std::string error;
    if (!schemaPage(document, identity.objectId, offset, limit, attributes, more, error))
        throw std::runtime_error(error);
    j.out << ",\"attributes\":[";
    uint64_t count = 0;
    for (const auto& item : attributes)
    {
        const auto* attribute = item.attribute;
        if (count++) j.out << ',';
        j.out << "{\"name\":"; j.text(attribute->mAttributeName);
        j.out << ",\"declaringClass\":"; j.text(item.declaringClass->mClassName);
        j.out << ",\"dataType\":" << attribute->mDataType;
        j.out << ",\"dataTypeName\":"; j.text(dataTypeName(attribute->mDataType));
        j.out << ",\"fieldCount\":"; j.id(attribute->mNbrFields);
        j.out << ",\"hasRecordedWrite\":";
        j.boolean(item.hasRecordedWrite);
        j.out << '}';
    }
    j.out << "],\"attributePage\":"; page(j, offset, count, more);
}
double decimalTolerance(const std::string& value)
{
    size_t i = 0;
    bool digit = false, nonzero = false;
    while (i < value.size() && value[i] >= '0' && value[i] <= '9')
    { digit = true; nonzero |= value[i++] != '0'; }
    if (i < value.size() && value[i] == '.')
    {
        ++i;
        while (i < value.size() && value[i] >= '0' && value[i] <= '9')
        { digit = true; nonzero |= value[i++] != '0'; }
    }
    if (!digit) throw std::runtime_error("Tolerance must be a finite nonnegative decimal");
    if (i < value.size() && (value[i] == 'e' || value[i] == 'E'))
    {
        ++i;
        if (i < value.size() && (value[i] == '+' || value[i] == '-')) ++i;
        const size_t exponent = i;
        while (i < value.size() && value[i] >= '0' && value[i] <= '9') ++i;
        if (i == exponent) throw std::runtime_error("Tolerance must be a finite nonnegative decimal");
    }
    if (i != value.size()) throw std::runtime_error("Tolerance must be a finite nonnegative decimal");
    double parsed = 0;
    std::istringstream parser(value); parser.imbue(std::locale::classic());
    if (!(parser >> parsed) || parser.peek() != EOF || !std::isfinite(parsed) || (nonzero && parsed == 0))
        throw std::runtime_error("Tolerance is outside the finite decimal range");
    return parsed;
}
const char* help =
    "ovd-inspect --version | --help\n"
    "ovd-inspect summary FILE [--recover-tail] [--segment N --context N --frame N] [--offset N --limit N]\n"
    "  [--frame-offset N --frame-limit N --object-offset N --object-limit N]\n"
    "  [--attribute-offset N --attribute-limit N]\n"
    "ovd-inspect inspect FILE [--recover-tail] --segment N --context N --frame N (--name NAME | --id ID)\n"
    "  [--class CLASS] --attribute ATTRIBUTE\n"
    "ovd-inspect compare LEFT RIGHT (--left-name NAME | --left-id ID)\n"
    "  (--right-name NAME | --right-id ID) [--left-class CLASS --right-class CLASS]\n"
    "  --left-segment N --right-segment N --left-context N --right-context N\n"
    "  --start N --end N --offset SIGNED --tolerance NUMBER [--stride N]\n"
    "  [--allow-carried] [--units TEXT] [--annotations OUTPUT]\n"
    "  [--attribute NAME --distance vec3|position|orientation]\n"
    "  Explicit vec3/position require --units; orientation uses radians only.\n"
    "Load limits: --max-stream-bytes --max-commands --max-creates --max-classes\n"
    "  --max-attributes --max-inheritance-depth --max-attribute-bytes\n"
    "  --max-frame-records --max-handler-steps --max-retained-bytes (positive integers).\n"
    "Summary pages frame records, scene objects and schema attributes (default limit 100).\n"
    "Inspect supports finite scalar floats, signed/unsigned integers, raw flags, vec3 and SDK transforms.\n"
    "Scalar values are JSON numbers; integer text is exact. Binary64-only clients may round magnitudes above 2^53.\n"
    "Frames are recorded START intervals, not application steps or frozen STOP snapshots.\n"
    "Scene membership is required. Global resources and unmapped contexts are unsupported.\n"
    "Compare pins each selected incarnation and requires exact writes by default.\n"
    "Sources are read-only. Recovery inspects only a validated prefix, not a complete capture.\n"
    "Annotations require a new path, with no viewer import support.\n"
    "Exit: 0 successful/equal, 1 different, 2 load rejection/incomparable/usage/I/O failure.\n"
    "Annotation I/O can fail after valid comparison JSON reaches stdout.\n";

int run(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        std::cout << help; std::cout.flush();
        if (!std::cout) { std::cerr << "Could not write stdout\n"; return 2; }
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--version")
    {
        Json j;
        j.out << "{\"tool\":\"ovd-inspect\",\"sdkVersion\":"; j.text(PVDDOM_SDK_VERSION);
        j.out << ",\"build\":"; j.text(PVDDOM_BUILD_INFO);
        j.out << ",\"evidenceVersion\":1,\"supportedEvidenceVersions\":[1,2]}";
        return writeEvidenceToStdout(j.out.str()) ? 0 : 2;
    }
    if (argc < 3) throw std::runtime_error(help);
    const std::string command = argv[1];
    const bool compare = command == "compare", summary = command == "summary", inspect = command == "inspect";
    if (!compare && !summary && !inspect) throw std::runtime_error("Unknown command");
    if (compare && argc < 4) throw std::runtime_error("Compare requires two files");
    LoadLimits limits;
    std::map<std::string, uint64_t*> limitOptions = {
        {"--max-stream-bytes", &limits.maxStreamBytes}, {"--max-commands", &limits.maxCommands},
        {"--max-creates", &limits.maxCreates}, {"--max-classes", &limits.maxClasses},
        {"--max-attributes", &limits.maxAttributes}, {"--max-inheritance-depth", &limits.maxInheritanceDepth},
        {"--max-attribute-bytes", &limits.maxAttributeBytes}, {"--max-frame-records", &limits.maxFrameRecords},
        {"--max-handler-steps", &limits.maxHandlerSteps},
        {"--max-retained-bytes", &limits.maxRetainedBytes}};
    const std::string allowed = compare ?
        " --left-name --left-id --right-name --right-id --left-class --right-class --left-segment --right-segment --left-context --right-context --start --end --offset --stride --tolerance --allow-carried --units --annotations --attribute --distance " :
        summary ? " --segment --context --frame --offset --limit --frame-offset --frame-limit --object-offset --object-limit --attribute-offset --attribute-limit --recover-tail " : " --segment --context --frame --name --id --class --attribute --recover-tail ";
    Options o;
    for (int i = compare ? 4 : 3; i < argc; ++i)
    {
        const std::string key = argv[i];
        auto limit = limitOptions.find(key);
        if (limit == limitOptions.end() && allowed.find(" " + key + " ") == std::string::npos)
            throw std::runtime_error("Unknown option: " + key);
        if (o.count(key)) throw std::runtime_error("Duplicate option: " + key);
        std::string value = "true";
        if (key != "--allow-carried" && key != "--recover-tail")
        {
            if (++i == argc || std::string(argv[i]).find("--") == 0) throw std::runtime_error("Missing value: " + key);
            value = argv[i];
        }
        o.emplace(key, value);
        if (limit != limitOptions.end()) *limit->second = natural(value, true);
    }
    Document left;
    load(left, argv[2], limits, compare ? "left" : "input",
        o.count("--recover-tail") ? LoadMode::RecoverTruncatedTail : LoadMode::Strict);
    if (compare)
    {
        ComparisonOptions opt;
        opt.leftObject = selector(o, "--left-"); opt.rightObject = selector(o, "--right-");
        opt.leftSegment = number(o, "--left-segment"); opt.rightSegment = number(o, "--right-segment");
        opt.leftContext = number(o, "--left-context"); opt.rightContext = number(o, "--right-context");
        opt.startFrame = number(o, "--start"); opt.endFrame = number(o, "--end");
        const std::string offset = required(o, "--offset");
        const bool negative = offset[0] == '-';
        const uint64_t magnitude = natural(negative || offset[0] == '+' ? offset.substr(1) : offset);
        if (magnitude > uint64_t(INT64_MAX) + unsigned(negative)) throw std::runtime_error("Offset out of range");
        opt.rightFrameOffset = negative ? -int64_t(magnitude - (magnitude != 0)) - (magnitude != 0) : int64_t(magnitude);
        const std::string tolerance = required(o, "--tolerance");
        opt.tolerance = decimalTolerance(tolerance);
        if (o.count("--stride")) opt.frameStride = natural(o.at("--stride"), true);
        if (o.count("--units")) opt.units = required(o, "--units");
        opt.requireExactSamples = !o.count("--allow-carried");
        const bool explicitAttribute = o.count("--attribute") != 0;
        if (explicitAttribute != (o.count("--distance") != 0))
            throw std::runtime_error("--attribute and --distance must be paired");
        ComparisonMeasure measure;
        if (explicitAttribute)
        {
            measure.attributeName = required(o, "--attribute");
            const std::string distance = required(o, "--distance");
            if (distance == "vec3") measure.distance = DistanceKind::Vec3Euclidean;
            else if (distance == "position") measure.distance = DistanceKind::TransformPositionEuclidean;
            else if (distance == "orientation") measure.distance = DistanceKind::TransformOrientationRadians;
            else throw std::runtime_error("Unsupported distance: " + distance);
            if (measure.distance == DistanceKind::TransformOrientationRadians)
            {
                if (o.count("--units") && required(o, "--units") != "radians")
                    throw std::runtime_error("Orientation units must be radians");
                measure.units = "radians";
            }
            else measure.units = required(o, "--units");
        }
        Document right; load(right, argv[3], limits, "right");
        const ComparisonResult result = explicitAttribute ? compareAttribute(left, right, opt, measure) :
            compareLinearVelocity(left, right, opt);
        const std::string json = explicitAttribute ?
            attributeEvidenceToJson(result, provenance(argv[2]), provenance(argv[3])) :
            evidenceToJson(result, provenance(argv[2]), provenance(argv[3]));
        const int resultCode = result.status == ComparisonStatus::Equal ? 0 : result.status == ComparisonStatus::Different ? 1 : 2;
        if (!writeEvidenceToStdout(json)) return 2;
        if (o.count("--annotations")) saveNew(required(o, "--annotations"), json);
        if (result.status == ComparisonStatus::Incomparable) std::cerr << result.reason << '\n';
        return resultCode;
    }
    Json j; auto& out = j.out;
    out << "{\"format\":\"physx.ovd." << command << "\",\"version\":1,\"capture\":"; j.provenance(provenance(argv[2]));
    out << ",\"loadInfo\":"; loadInfoJson(j, left.loadInfo());
    if (summary)
    {
        const uint64_t defaultOffset = o.count("--offset") ? number(o, "--offset") : 0;
        const uint64_t defaultLimit = o.count("--limit") ? natural(o.at("--limit"), true) : 100;
        const uint64_t frameOffset = o.count("--frame-offset") ? number(o, "--frame-offset") : defaultOffset;
        const uint64_t frameLimit = o.count("--frame-limit") ? natural(o.at("--frame-limit"), true) : defaultLimit;
        const uint64_t objectOffset = o.count("--object-offset") ? number(o, "--object-offset") : defaultOffset;
        const uint64_t objectLimit = o.count("--object-limit") ? natural(o.at("--object-limit"), true) : defaultLimit;
        const uint64_t attributeOffset = o.count("--attribute-offset") ? number(o, "--attribute-offset") : 0;
        const uint64_t attributeLimit = o.count("--attribute-limit") ? natural(o.at("--attribute-limit"), true) : 100;
        out << ",\"streamBytes\":"; j.id(left.stats().streamBytes);
        out << ",\"commands\":"; j.id(left.stats().commands);
        out << ",\"ignoredCommands\":"; j.id(left.stats().ignoredCommands);
        out << ",\"unresolvedOptionalEdges\":"; j.id(left.stats().unresolvedOptionalEdges);
        out << ",\"creates\":"; j.id(left.stats().creates);
        out << ",\"handlerSteps\":"; j.id(left.stats().handlerSteps);
        out << ",\"membershipVisits\":"; j.id(left.stats().membershipVisits);
        out << ",\"accountedRetainedBytes\":"; j.id(left.stats().accountedRetainedBytes);
        out << ",\"contexts\":["; bool first = true;
        for (const auto& entry : left.frameIndex())
        {
            if (!first) out << ','; first = false;
            out << "{\"recordingSegment\":"; j.id(entry.first.first);
            out << ",\"context\":"; j.id(entry.first.second);
            out << ",\"sceneMapped\":"; j.boolean(entry.second.sceneMapped);
            out << ",\"monotonic\":"; j.boolean(entry.second.monotonic);
            out << ",\"frameCount\":"; j.id(entry.second.recordedFrames.size());
            out << ",\"firstFrame\":"; j.id(entry.second.recordedFrames.front());
            out << ",\"lastFrame\":"; j.id(entry.second.recordedFrames.back()); out << '}';
        }
        out << "],\"frames\":["; first = true;
        uint64_t frameCount = 0;
        for (uint64_t i = frameOffset; i < left.frames().size() && frameCount < frameLimit; ++i)
        {
            const auto& f = left.frames()[size_t(i)];
            if (!first) out << ','; first = false;
            out << "{\"frame\":"; j.frame({f.recordingSegment, f.context, f.recordedFrame});
            out << ",\"sourceTimestamp\":"; j.id(f.sourceTimestamp);
            out << ",\"completed\":"; j.boolean(f.completed); out << '}';
            ++frameCount;
        }
        out << ']';
        const bool moreFrames = frameOffset < left.frames().size() &&
            frameCount < left.frames().size() - frameOffset;
        out << ",\"framePage\":";
        page(j, frameOffset, frameCount, moreFrames, left.frames().size());
        if (o.count("--segment") || o.count("--context") || o.count("--frame"))
        {
            FrameRef frame{number(o, "--segment"), number(o, "--context"), number(o, "--frame")};
            std::vector<ObjectIdentity> objects; std::string error;
            const uint64_t lookahead = objectLimit == UINT64_MAX ? UINT64_MAX : objectLimit + 1;
            if (!listObjects(left, frame, objectOffset, lookahead, objects, error)) throw std::runtime_error(error);
            const bool moreObjects = objects.size() > objectLimit;
            if (moreObjects) objects.pop_back();
            out << ",\"objects\":["; first = true;
            for (const auto& object : objects)
            {
                if (!first) out << ','; first = false;
                Json identity;
                identity.identity(object);
                const std::string fields = identity.out.str();
                j.textWasReplaced |= identity.textWasReplaced;
                out << fields.substr(0, fields.size() - 1);
                schemaAttributes(j, left, object, attributeOffset, attributeLimit);
                out << '}';
            }
            out << ']';
            out << ",\"objectPage\":"; page(j, objectOffset, objects.size(), moreObjects);
        }
    }
    else
    {
        FrameRef frame{number(o, "--segment"), number(o, "--context"), number(o, "--frame")};
        ObjectIdentity identity; std::string error;
        if (!selectObject(left, selector(o, "--"), frame, identity, error)) throw std::runtime_error(error);
        SampleResult sample;
        InspectionValue value;
        const std::string attribute = required(o, "--attribute");
        if (!readInspectionValue(left, identity.objectId, frame, attribute.c_str(), sample, value, error)) throw std::runtime_error(error);
        out << ",\"attribute\":"; j.text(attribute);
        out << ",\"sample\":"; j.sample(sample);
        switch (value.kind)
        {
        case InspectionValueKind::Missing: out << ",\"value\":null"; break;
        case InspectionValueKind::FloatingScalar: out << ",\"value\":"; j.scalar(value.floating); break;
        case InspectionValueKind::SignedScalar: out << ",\"value\":"; j.integer(value.signedValue); break;
        case InspectionValueKind::UnsignedScalar: out << ",\"value\":"; j.integer(value.unsignedValue); break;
        case InspectionValueKind::Vec3: out << ",\"value\":"; j.vector(value.components); break;
        case InspectionValueKind::Transform:
            out << ",\"position\":"; j.vector(value.components); out << ",\"quaternion\":[";
            for (int i = 0; i < 4; ++i) { if (i) out << ','; j.number(value.quaternion[i]); } out << ']';
            break;
        }
    }
    out << ",\"textWasReplaced\":"; j.boolean(j.textWasReplaced); out << '}';
    return writeEvidenceToStdout(out.str()) ? 0 : 2;
}
}
int main(int argc, char** argv)
{
    try { return run(argc, argv); }
    catch (const LoadFailure& e)
    {
        std::cerr << e.what() << '\n';
        writeEvidenceToStdout(loadErrorJson(e));
        return 2;
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
