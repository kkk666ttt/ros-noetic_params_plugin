#pragma once
#include <ros_params/dynamic/dynamic_registry.hpp>
#include <ros_params/yaml/yaml_adapter.hpp>
#include <ros/node_handle.h>
#include <string>

namespace vision::ros_params
{
    enum class ManagerState
    {
        CREATED,
        INITIALIZING,
        ACTIVE,
        SHUTDOWN
    };

    /**
     * @brief 启动编排：全量 Binding 校验 → YAML 事务 → Kernel 快照到 cfg。
     * @note 所有 onInit 完成后、kernel.start/spinner 启动前 initialize；失败不可重试。
     *       管理操作由宿主串行执行。ParamServer 必须比本对象活得更久。
     */
    class RosParamsManager
    {
    public:
        RosParamsManager(ros::NodeHandle nh, kernel::ParamServer& server);
        ~RosParamsManager();
        RosParamsManager(const RosParamsManager&) = delete;
        RosParamsManager& operator=(const RosParamsManager&) = delete;

        YamlBindings& yaml() noexcept
        {
            return yaml_adapter_.bindings();
        }

        DynamicRegistry& dynamic() noexcept
        {
            return dynamic_registry_;
        }

        /** @brief 设置 YAML 启动文件，可省略以使用组件默认值。 */
        void loadYamlFile(std::string path);
        /** @brief 仅 CREATED 可设置；handler 必须支持多个 Server 并发调用。 */
        void setErrorHandler(ErrorHandler handler);

        ManagerState state() const noexcept
        {
            return state_;
        }

        void initialize();
        /** @brief 先停止并等待相关 spinner；不得从 callback/converter/handler 内调用。 */
        void shutdown() noexcept;

    private:
        void checkCreated() const;
        ros::NodeHandle nh_;
        kernel::ParamServer& server_;
        ErrorHandler handler_;
        YamlParamAdapter yaml_adapter_;
        DynamicRegistry dynamic_registry_;
        ManagerState state_ = ManagerState::CREATED;
    };
}
