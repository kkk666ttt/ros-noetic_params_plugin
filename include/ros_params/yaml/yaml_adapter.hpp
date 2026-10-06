#pragma once
#include <ros_params/yaml/yaml_bindings.hpp>
#include <string>
#include <vector>

namespace vision::ros_params
{
    class RosParamsManager;

    /** @brief YAML 仅为启动输入；全部解码后一次提交，不监听文件。 */
    class YamlParamAdapter
    {
    public:
        explicit YamlParamAdapter(kernel::ParamServer& server) : server_(server)
        {
        }

        YamlParamAdapter(const YamlParamAdapter&) = delete;
        YamlParamAdapter& operator=(const YamlParamAdapter&) = delete;

        YamlBindings& bindings() noexcept
        {
            return bindings_;
        }

        /** @brief 设置启动文件；读取延迟到 initialize。空文件名被拒绝。 */
        void loadFile(std::string path);

    private:
        friend class RosParamsManager;

        void seal() noexcept
        {
            bindings_.sealed_ = true;
        }

        void validate(const std::vector<kernel::ParamInfo>& schema) const;
        void apply();
        kernel::ParamServer& server_;
        YamlBindings bindings_;
        std::string file_;
    };
}
