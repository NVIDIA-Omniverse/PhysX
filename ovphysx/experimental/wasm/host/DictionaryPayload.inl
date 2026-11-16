// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Included in the copied OVStage host after its settings helpers.
struct PayloadNode
{
    carb::dictionary::Item value;
    carb::dictionary::Item* parent = nullptr;
    std::string name;
    std::vector<carb::dictionary::Item*> children;
};
std::unordered_map<const carb::dictionary::Item*, std::unique_ptr<PayloadNode>>& payloadNodes()
{
    static std::unordered_map<const carb::dictionary::Item*, std::unique_ptr<PayloadNode>> nodes;
    return nodes;
}
PayloadNode* payloadNode(const carb::dictionary::Item* item)
{
    const auto found = payloadNodes().find(item);
    return found == payloadNodes().end() ? nullptr : found->second.get();
}
void destroyPayload(carb::dictionary::Item* item)
{
    std::lock_guard<std::recursive_mutex> lock(state().mutex);
    PayloadNode* node = payloadNode(item);
    if (!node)
        unsupported("destroying an item not owned by the payload dictionary");
    const std::vector<carb::dictionary::Item*> children = node->children;
    for (carb::dictionary::Item* child : children)
        destroyPayload(child);
    if (node->parent)
    {
        std::vector<carb::dictionary::Item*>& siblings = payloadNode(node->parent)->children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), item), siblings.end());
    }
    payloadNodes().erase(item);
}
void payloadType(carb::dictionary::Item* item, ItemType type)
{
    PayloadNode* node = payloadNode(item);
    if (!node)
        unsupported("writing settings through the payload dictionary");
    if (item->type != type)
    {
        const std::vector<carb::dictionary::Item*> children = node->children;
        for (carb::dictionary::Item* child : children)
            destroyPayload(child);
        item->type = type;
        item->integer = 0;
        item->floating = 0;
        item->string.clear();
    }
}
carb::dictionary::Item* newPayload(carb::dictionary::Item* parent, std::string name, ItemType type)
{
    std::unique_ptr<PayloadNode> node(new PayloadNode);
    node->parent = parent;
    node->name = std::move(name);
    node->value.type = type;
    carb::dictionary::Item* item = &node->value;
    payloadNodes().emplace(item, std::move(node));
    if (parent)
        payloadNode(parent)->children.push_back(item);
    return item;
}
carb::dictionary::Item* payloadPath(carb::dictionary::Item* base, std::string path, bool create, ItemType type)
{
    std::lock_guard<std::recursive_mutex> lock(state().mutex);
    if (!base)
        return create ? newPayload(nullptr, std::move(path), type) : nullptr;
    if (!payloadNode(base))
    {
        if (create)
            unsupported("creating settings through the payload dictionary");
        std::string basePath;
        if (base == &state().root)
            basePath = "/";
        else
            for (const std::pair<const std::string, Setting>& entry : state().settings)
                if (&entry.second == base)
                    basePath = entry.first;
        if (basePath.empty())
            return nullptr;
        if (path.empty())
            return base;
        const auto found = state().settings.find(key(((basePath == "/" ? basePath : basePath + "/") + path).c_str()));
        return found == state().settings.end() ? nullptr : &found->second;
    }
    size_t offset = 0;
    while (offset < path.size())
    {
        if (path[offset] == '/')
            throw std::invalid_argument("Dictionary child paths must be relative and have no empty components");
        const size_t slash = path.find('/', offset);
        const std::string component = path.substr(offset, slash == std::string::npos ? slash : slash - offset);
        PayloadNode* node = payloadNode(base);
        carb::dictionary::Item* next = nullptr;
        for (carb::dictionary::Item* child : node->children)
            if (payloadNode(child)->name == component)
                next = child;
        if (!next)
        {
            if (!create)
                return nullptr;
            payloadType(base, ItemType::eDictionary);
            next = newPayload(base, component, slash == std::string::npos ? type : ItemType::eDictionary);
        }
        base = next;
        if (slash == std::string::npos)
            break;
        offset = slash + 1;
    }
    if (create)
        payloadType(base, type);
    return base;
}
void installPayloadDictionary(carb::dictionary::IDictionary& table)
{
    using Item = carb::dictionary::Item;
    table.readLock = [](const Item*) { state().mutex.lock(); };
    table.writeLock = [](Item*) { state().mutex.lock(); };
    table.unlock = [](const Item*) { state().mutex.unlock(); };
    table.getItem = [](const Item* base, const char* path) -> const Item* {
        return payloadPath(const_cast<Item*>(base), path ? path : "", false, ItemType::eCount);
    };
    table.getItemMutable = [](Item* base, const char* path) {
        return payloadPath(base, path ? path : "", false, ItemType::eCount);
    };
    table.getItemS = [](const Item* base, const carb::cpp::string_view& path) -> const Item* {
        return payloadPath(const_cast<Item*>(base), std::string(path.data(), path.size()), false, ItemType::eCount);
    };
    table.getItemMutableS = [](Item* base, const carb::cpp::string_view& path) {
        return payloadPath(base, std::string(path.data(), path.size()), false, ItemType::eCount);
    };
    table.createItem = [](Item* base, const char* path, ItemType type) {
        if (!path || type == ItemType::eCount)
            throw std::invalid_argument("Invalid dictionary item description");
        return payloadPath(base, path, true, type);
    };
    table.createItemS = [](Item* base, const carb::cpp::string_view& path, ItemType type) {
        if (type == ItemType::eCount)
            throw std::invalid_argument("Invalid dictionary item type");
        return payloadPath(base, std::string(path.data(), path.size()), true, type);
    };
    table.destroyItem = destroyPayload;
    table.setInt64 = [](Item* item, int64_t value) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        payloadType(item, ItemType::eInt);
        item->integer = value;
    };
    table.setFloat64 = [](Item* item, double value) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        payloadType(item, ItemType::eFloat);
        item->floating = value;
    };
    table.setBool = [](Item* item, bool value) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        payloadType(item, ItemType::eBool);
        item->integer = value;
    };
    table.internalSetString = [](Item* item, const char* value, size_t length) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        payloadType(item, ItemType::eString);
        item->string.assign(value ? value : "", !value ? 0 : length == size_t(-1) ? std::strlen(value) : length);
    };
    table.setStringS = [](Item* item, const carb::cpp::string_view& value) {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        payloadType(item, ItemType::eString);
        item->string.assign(value.data(), value.size());
    };
    table.getItemChildCount = [](const Item* item) -> size_t {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item && item->type == ItemType::eDictionary)
            unsupported("enumerating settings through the payload dictionary");
        return node ? node->children.size() : 0;
    };
    table.getItemChildByIndex = [](const Item* item, size_t index) -> const Item* {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item && item->type == ItemType::eDictionary)
            unsupported("enumerating settings through the payload dictionary");
        return node && index < node->children.size() ? node->children[index] : nullptr;
    };
    table.getItemChildByIndexMutable = [](Item* item, size_t index) -> Item* {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item && item->type == ItemType::eDictionary)
            unsupported("enumerating settings through the payload dictionary");
        return node && index < node->children.size() ? node->children[index] : nullptr;
    };
    table.getItemParent = [](const Item* item) -> const Item* {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item)
            unsupported("querying settings parents through the payload dictionary");
        return node ? node->parent : nullptr;
    };
    table.getItemParentMutable = [](Item* item) -> Item* {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item)
            unsupported("querying settings parents through the payload dictionary");
        return node ? node->parent : nullptr;
    };
    table.getItemName = [](const Item* item) -> const char* {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item)
            unsupported("querying settings names through the payload dictionary");
        return node ? node->name.c_str() : nullptr;
    };
    table.getItemNameS = [](const Item* item) -> carb::cpp::optional<carb::cpp::zstring_view> {
        std::lock_guard<std::recursive_mutex> lock(state().mutex);
        const PayloadNode* node = payloadNode(item);
        if (!node && item)
            unsupported("querying settings names through the payload dictionary");
        return node ? carb::cpp::optional<carb::cpp::zstring_view>(carb::cpp::zstring_view(node->name)) : carb::cpp::nullopt;
    };
}
