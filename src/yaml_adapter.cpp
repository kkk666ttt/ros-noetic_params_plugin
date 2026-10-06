#include <ros_params/yaml/yaml_adapter.hpp>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_set>
#include <utility>

namespace vision::ros_params
{
    namespace
    {
        YAML::Node lookup(const YAML::Node& root, const std::string& path)
        {
            YAML::Node node(root);
            std::size_t begin = 0;
            while (begin < path.size())
            {
                if (!node.IsDefined())
                {
                    return YAML::Node(YAML::NodeType::Undefined);
                }
                if (!node.IsMap())
                {
                    throw std::runtime_error("YAML path traverses a non-map node");
                }
                const auto end = path.find('/', begin);
                const auto key = path.substr(begin, end == std::string::npos ? end : end - begin);
                // reset 重绑定句柄，不能用赋值改变原 YAML 树。
                const YAML::Node child = static_cast<const YAML::Node&>(node)[key];
                if (!child.IsDefined())
                {
                    return YAML::Node(YAML::NodeType::Undefined);
                }
                node.reset(child);
                if (end == std::string::npos)
                {
                    break;
                }
                begin = end + 1;
            }
            return node;
        }
    }

    void YamlParamAdapter::loadFile(std::string path)
    {
        bindings_.checkMutable();
        if (path.empty())
        {
            detail::fail(ErrorCode::YAML_FILE_ERROR, "", path, "Empty YAML filename");
        }
        file_ = std::move(path);
    }

    void YamlParamAdapter::validate(const std::vector<kernel::ParamInfo>& schema) const
    {
        std::unordered_set<std::string> paths, external;
        for (const auto& entry : bindings_.entries_)
        {
            const auto& info = entry.info;
            if (!paths.insert(info.canonical_path).second || !external.insert(info.external_path).second)
            {
                detail::fail(ErrorCode::DUPLICATE_BINDING, info.canonical_path, info.external_path,
                             "Duplicate YAML binding");
            }
            if (info.external_path.empty() || info.external_path.front() == '/' ||
                info.external_path.back() == '/' || info.external_path.find("//") != std::string::npos)
            {
                detail::fail(ErrorCode::YAML_DECODE_ERROR, info.canonical_path, info.external_path,
                             "YAML path requires nonempty slash-separated keys");
            }
            detail::validateBinding(info, schema, false);
        }
    }

    void YamlParamAdapter::apply()
    {
        YAML::Node root(YAML::NodeType::Map);
        if (!file_.empty())
        {
            try
            {
                root.reset(YAML::LoadFile(file_));
            }
            catch (const std::exception& e)
            {
                detail::fail(ErrorCode::YAML_FILE_ERROR, "", file_, e.what());
            }
            if (!root.IsMap())
            {
                detail::fail(ErrorCode::YAML_FILE_ERROR, "", file_, "YAML root must be a map");
            }
        }
        std::vector<kernel::ParamUpdate> updates;
        updates.reserve(bindings_.entries_.size());
        for (const auto& entry : bindings_.entries_)
        {
            const auto& info = entry.info;
            try
            {
                const auto node = lookup(root, info.external_path);
                if (!node.IsDefined())
                {
                    if (entry.presence == Presence::REQUIRED)
                    {
                        detail::fail(ErrorCode::YAML_MISSING_REQUIRED, info.canonical_path,
                                     info.external_path, "Required YAML field is missing");
                    }
                    continue;
                }
                updates.push_back({info.canonical_path, entry.decode(node)});
            }
            catch (const RosParamsException&)
            {
                throw;
            }
            catch (const std::exception& e)
            {
                detail::fail(ErrorCode::YAML_DECODE_ERROR, info.canonical_path, info.external_path, e.what());
            }
            catch (...)
            {
                detail::fail(ErrorCode::YAML_DECODE_ERROR, info.canonical_path, info.external_path,
                             "YAML decoder threw an unknown exception");
            }
        }
        try
        {
            const auto result = server_.update(updates);
            if (!result)
            {
                detail::fail(ErrorCode::PARAM_UPDATE_REJECTED, result.path, file_, result.message);
            }
        }
        catch (const RosParamsException&)
        {
            throw;
        }
        catch (const std::exception& e)
        {
            detail::fail(ErrorCode::PARAM_UPDATE_REJECTED, "", file_, e.what());
        }
        catch (...)
        {
            detail::fail(ErrorCode::PARAM_UPDATE_REJECTED, "", file_,
                         "Kernel update threw an unknown exception");
        }
    }
}
