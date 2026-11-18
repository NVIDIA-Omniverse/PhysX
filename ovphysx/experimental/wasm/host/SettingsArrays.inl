// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Included in the copied OVStage host after its settings helpers.
std::string settingsArrayElement(const char* path, size_t index)
{
    return key(path) + "/" + std::to_string(index);
}
size_t settingsArrayPrefix(const char* path)
{
    std::lock_guard<std::recursive_mutex> lock(state().mutex);
    if (getItemType(path) != ItemType::eDictionary)
        return 0;
    size_t length = 0;
    while (state().settings.find(settingsArrayElement(path, length)) != state().settings.end())
        ++length;
    return length;
}
size_t settingsArrayLength(const char* path)
{
    std::lock_guard<std::recursive_mutex> lock(state().mutex);
    const size_t prefix = settingsArrayPrefix(path);
    if (!prefix)
        return 0;
    const std::string parent = key(path) + "/";
    size_t children = 0;
    for (const std::pair<const std::string, std::shared_ptr<Setting>>& entry : state().settings)
        if (entry.first.compare(0, parent.size(), parent) == 0 &&
            entry.first.find('/', parent.size()) == std::string::npos)
            ++children;
    return children == prefix ? prefix : 0;
}
void setSettingsStringArray(const char* path, const char* const* values, size_t count)
{
    if (count && !values)
        throw std::invalid_argument("Nonempty string arrays require a value array");
    std::vector<std::string> copied;
    copied.reserve(count);
    for (size_t index = 0; index != count; ++index)
        copied.emplace_back(values[index] ? values[index] : "");
    setValue(path, Setting{});
    for (size_t index = 0; index != count; ++index)
        setString(settingsArrayElement(path, index).c_str(), copied[index].data(), copied[index].size());
}
void installSettingsArrays(carb::settings::ISettings& table)
{
    table.setStringArray = setSettingsStringArray;
    table.setStringArrayS = [](const carb::cpp::string_view& path, const char* const* values, size_t count) {
        setSettingsStringArray(std::string(path.data(), path.size()).c_str(), values, count);
    };
    table.isAccessibleAsArray = [](const char* path) { return settingsArrayPrefix(path) != 0; };
    table.isAccessibleAsArrayS = [](const carb::cpp::string_view& path) {
        return settingsArrayPrefix(std::string(path.data(), path.size()).c_str()) != 0;
    };
    table.isAccessibleAsArrayOf = [](ItemType type, const char* path) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const size_t length = settingsArrayPrefix(path);
        for (size_t index = 0; index != length; ++index)
            if (!isAccessibleAs(type, settingsArrayElement(path, index).c_str()))
                return false;
        return length != 0;
    };
    table.isAccessibleAsArrayOfS = [](ItemType type, const carb::cpp::string_view& path) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const std::string name(path.data(), path.size());
        const size_t length = settingsArrayPrefix(name.c_str());
        for (size_t index = 0; index != length; ++index)
            if (!isAccessibleAs(type, settingsArrayElement(name.c_str(), index).c_str()))
                return false;
        return length != 0;
    };
    table.getArrayLength = settingsArrayLength;
    table.getArrayLengthS = [](const carb::cpp::string_view& path) {
        return settingsArrayLength(std::string(path.data(), path.size()).c_str());
    };
    table.internalGetStringBufferAt = [](const char* path, size_t index, size_t* length) {
        return getString(settingsArrayElement(path, index).c_str(), length);
    };
    table.getStringBufferAtS = [](const carb::cpp::string_view& path, size_t index) {
        return getStringView(settingsArrayElement(std::string(path.data(), path.size()).c_str(), index).c_str());
    };
}
