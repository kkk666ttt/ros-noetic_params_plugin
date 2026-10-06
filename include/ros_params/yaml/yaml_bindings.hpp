#pragma once
#include <ros_params/detail/binding.hpp>
#include <ros_params/yaml/yaml_codec.hpp>
#include <functional>
#include <string>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace vision::ros_params
{
    enum class Presence
    {
        OPTIONAL,
        REQUIRED
    };
    class YamlParamAdapter;

    /** @brief 初始化前声明路径映射；保留引用也不能在封存后继续注册。 */
    class YamlBindings
    {
    public:
        YamlBindings() = default;
        YamlBindings(const YamlBindings&) = delete;
        YamlBindings& operator=(const YamlBindings&) = delete;

        template <class T>
        void bind(std::string canonical_path, std::string yaml_path, Presence presence = Presence::OPTIONAL)
        {
            bind<T>(
                std::move(canonical_path), std::move(yaml_path),
                [](const YAML::Node& node)
                {
                    return YamlCodec<T>::decode(node);
                },
                presence);
        }

        /** @brief 显式 decoder 负责外部表示转换；解码期间不得写入 ParamServer。 */
        template <class T>
        void bind(std::string canonical_path,
                  std::string yaml_path,
                  std::function<T(const YAML::Node&)> decoder,
                  Presence presence = Presence::OPTIONAL)
        {
            static_assert(std::is_same_v<T, std::decay_t<T>> && std::is_copy_constructible_v<T>,
                          "Bindings require unqualified copy-constructible Kernel value types");
            checkMutable();
            if (!decoder)
            {
                throw std::invalid_argument("YAML decoder is empty");
            }
            entries_.push_back({{std::move(canonical_path), std::move(yaml_path), typeid(T)},
                                presence,
                                [decoder = std::move(decoder)](const YAML::Node& node)
                                {
                                    return kernel::ParamValue(decoder(node));
                                }});
        }

    private:
        friend class YamlParamAdapter;

        struct Entry
        {
            detail::BindingInfo info;
            Presence presence;
            std::function<kernel::ParamValue(const YAML::Node&)> decode;
        };

        void checkMutable() const
        {
            if (sealed_)
            {
                detail::fail(ErrorCode::INVALID_STATE, "", "", "YAML bindings are sealed");
            }
        }

        bool sealed_ = false;
        std::vector<Entry> entries_;
    };
}
