#pragma once
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace vision::ros_params
{
    /** @brief 业务可特化 YamlCodec<T>::decode；输出必须为 Kernel 声明的最终值类型。 */
    template <class T, class Enable = void>
    struct YamlCodec;

    template <class T>
    struct YamlCodec<T,
                     std::enable_if_t<std::is_same_v<T, bool> || std::is_same_v<T, int> ||
                                      std::is_same_v<T, std::int32_t> || std::is_same_v<T, std::int64_t> ||
                                      std::is_same_v<T, float> || std::is_same_v<T, double> ||
                                      std::is_same_v<T, std::string>>>
    {
        static T decode(const YAML::Node& node)
        {
            if (!node.IsScalar())
            {
                throw YAML::BadConversion(node.Mark());
            }
            return node.as<T>();
        }
    };

    template <class T>
    struct YamlCodec<std::vector<T>>
    {
        static std::vector<T> decode(const YAML::Node& node)
        {
            if (!node.IsSequence())
            {
                throw YAML::BadConversion(node.Mark());
            }
            std::vector<T> value;
            value.reserve(node.size());
            for (const auto& element : node)
            {
                value.push_back(YamlCodec<T>::decode(element));
            }
            return value;
        }
    };
}
