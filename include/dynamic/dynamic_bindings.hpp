#pragma once
#include <detail/binding.hpp>
#include <functional>
#include <string>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace vision::ros_params
{
    template <class Config>
    class DynamicParamAdapter;

    /** @brief generated Config 的编译期映射；仅绑定 RUNTIME_ABLE 参数。 */
    template <class Config>
    class DynamicBindings
    {
    public:
        DynamicBindings() = default;
        DynamicBindings(const DynamicBindings&) = delete;
        DynamicBindings& operator=(const DynamicBindings&) = delete;

        template <class T>
        void bind(std::string canonical_path, T Config::*member)
        {
            if (!member)
            {
                throw std::invalid_argument("Dynamic member pointer is null");
            }
            bind<T>(
                std::move(canonical_path),
                [member](const Config& cfg)
                {
                    return cfg.*member;
                },
                [member](Config& cfg, const T& value)
                {
                    cfg.*member = value;
                });
        }

        /**
         * @brief 显式转换最终 Kernel T；converter 不得写 ParamServer 或重入管理操作。
         * @note to_cfg 必须能表示所有合法 Kernel 值；普通转换失败不提交参数。
         */
        template <class T>
        void bind(std::string canonical_path,
                  std::function<T(const Config&)> from_cfg,
                  std::function<void(Config&, const T&)> to_cfg)
        {
            static_assert(std::is_same_v<T, std::decay_t<T>> && std::is_copy_constructible_v<T>,
                          "Bindings require unqualified copy-constructible Kernel value types");
            if (sealed_)
            {
                detail::fail(ErrorCode::INVALID_STATE, canonical_path, "", "Dynamic bindings are sealed");
            }
            if (!from_cfg || !to_cfg)
            {
                throw std::invalid_argument("Dynamic converter is empty");
            }
            entries_.push_back(
                {{std::move(canonical_path), "", typeid(T)},
                 [from_cfg = std::move(from_cfg)](const Config& cfg)
                 {
                     return kernel::ParamValue(from_cfg(cfg));
                 },
                 [to_cfg](Config& cfg, const kernel::ParamSnapshot& snapshot, const std::string& path)
                 {
                     to_cfg(cfg, snapshot.get<T>(path));
                 },
                 [to_cfg](Config& cfg, const kernel::ParamValue& value)
                 {
                     to_cfg(cfg, value.getRef<T>());
                 }});
        }

    private:
        friend class DynamicParamAdapter<Config>;

        struct Entry
        {
            detail::BindingInfo info;
            std::function<kernel::ParamValue(const Config&)> from;
            std::function<void(Config&, const kernel::ParamSnapshot&, const std::string&)> restore;
            std::function<void(Config&, const kernel::ParamValue&)> normalize;
        };

        bool sealed_ = false;
        std::vector<Entry> entries_;
    };
}
