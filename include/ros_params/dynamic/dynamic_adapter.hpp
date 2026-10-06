#pragma once
#include <ros_params/dynamic/dynamic_bindings.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <atomic>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace vision::ros_params
{
    class DynamicRegistry;

    enum class DynamicState
    {
        UNINITIALIZED,
        INITIALIZING,
        ACTIVE,
        SHUTDOWN
    };

    /** @brief Registry 的类型擦除边界，不存储算法参数或运行时真值副本。 */
    class IDynamicAdapter
    {
    public:
        virtual ~IDynamicAdapter() = default;
        virtual void seal() noexcept = 0;
        virtual std::vector<detail::BindingInfo> bindingInfo() const = 0;
        virtual void initializeFromKernel() = 0;
        virtual void shutdown() noexcept = 0;
    };

    /**
     * @brief 一个 generated Config 对应一个 ROS Server，一次 callback 对应一次事务。
     * @note ParamServer 生命周期更长；宿主停止并等待 spinner 后才能 shutdown/析构。
     *       每个 Server 独立同步，不提供跨 Server 事务或外部写入的持续 UI 同步。
     */
    template <class Config>
    class DynamicParamAdapter final : public IDynamicAdapter
    {
        static_assert(std::is_nothrow_move_assignable_v<Config>,
                      "Generated Config must support nonthrowing move assignment after commit");

    public:
        DynamicParamAdapter(ros::NodeHandle nh,
                            kernel::ParamServer& server,
                            std::string external_path,
                            const ErrorHandler& handler)
            : nh_(std::move(nh)), server_(server), external_path_(std::move(external_path)), handler_(handler)
        {
        }

        ~DynamicParamAdapter() override
        {
            shutdown();
        }

        DynamicParamAdapter(const DynamicParamAdapter&) = delete;
        DynamicParamAdapter& operator=(const DynamicParamAdapter&) = delete;

        DynamicBindings<Config>& bindings() noexcept
        {
            return bindings_;
        }

        template <class T, class... Args>
        void bind(std::string canonical_path, Args&&... args)
        {
            bindings_.template bind<T>(std::move(canonical_path), std::forward<Args>(args)...);
        }

        DynamicState state() const noexcept
        {
            return state_.load();
        }

    private:
        friend class DynamicRegistry;

        void seal() noexcept override
        {
            bindings_.sealed_ = true;
        }

        std::vector<detail::BindingInfo> bindingInfo() const override
        {
            std::vector<detail::BindingInfo> result;
            for (const auto& entry : bindings_.entries_)
            {
                result.push_back({entry.info.canonical_path, external_path_, entry.info.type});
            }
            return result;
        }

        void initializeFromKernel() override
        {
            if (state() != DynamicState::UNINITIALIZED)
            {
                detail::fail(ErrorCode::INVALID_STATE, "", external_path_,
                             "Dynamic adapter already initialized");
            }
            seal();
            state_ = DynamicState::INITIALIZING;
            try
            {
                boost::recursive_mutex::scoped_lock lock(mutex_);
                Config initial = Config::__getDefault__();
                fillFromKernel(initial);
                dynamic_server_ = std::make_unique<dynamic_reconfigure::Server<Config>>(mutex_, nh_);
                dynamic_server_->setCallback(
                    [this](Config& cfg, std::uint32_t)
                    {
                        callback(cfg);
                    });
                // Noetic 会吞掉 setCallback 中的异常，必须在返回后显式检查初始化结果。
                if (initialization_error_)
                {
                    std::rethrow_exception(initialization_error_);
                }
                dynamic_server_->updateConfig(initial);
                state_ = DynamicState::ACTIVE;
            }
            catch (...)
            {
                shutdown();
                throw;
            }
        }

        /** @brief 管理线程串行调用；不得从 converter/ErrorHandler/ROS callback 中重入。 */
        void shutdown() noexcept override
        {
            state_ = DynamicState::SHUTDOWN;
            if (dynamic_server_)
            {
                dynamic_server_->clearCallback();
            }
            dynamic_server_.reset();
        }

        void fillFromKernel(Config& cfg)
        {
            const auto snapshot = server_.snapshot();
            for (const auto& entry : bindings_.entries_)
            {
                try
                {
                    entry.restore(cfg, snapshot, entry.info.canonical_path);
                }
                catch (const std::exception& e)
                {
                    detail::fail(ErrorCode::DYNAMIC_CONVERSION_ERROR, entry.info.canonical_path,
                                 external_path_, e.what());
                }
                catch (...)
                {
                    detail::fail(ErrorCode::DYNAMIC_CONVERSION_ERROR, entry.info.canonical_path,
                                 external_path_, "Kernel-to-cfg converter threw an unknown exception");
                }
            }
        }

        void runtimeUpdate(Config& cfg)
        {
            std::vector<kernel::ParamUpdate> updates;
            updates.reserve(bindings_.entries_.size());
            Config normalized(cfg);
            for (const auto& entry : bindings_.entries_)
            {
                try
                {
                    auto value = entry.from(cfg);
                    // 提交前检验反向表示，转换失败时不产生已经提交但无法回显的值。
                    entry.normalize(normalized, value);
                    updates.push_back({entry.info.canonical_path, std::move(value)});
                }
                catch (const std::exception& e)
                {
                    detail::fail(ErrorCode::DYNAMIC_CONVERSION_ERROR, entry.info.canonical_path,
                                 external_path_, e.what());
                }
                catch (...)
                {
                    detail::fail(ErrorCode::DYNAMIC_CONVERSION_ERROR, entry.info.canonical_path,
                                 external_path_, "cfg converter threw an unknown exception");
                }
            }
            try
            {
                const auto result = server_.update(updates);
                if (!result)
                {
                    detail::fail(ErrorCode::PARAM_UPDATE_REJECTED, result.path, external_path_,
                                 result.message);
                }
            }
            catch (const RosParamsException&)
            {
                throw;
            }
            catch (const std::exception& e)
            {
                detail::fail(ErrorCode::PARAM_UPDATE_REJECTED, "", external_path_, e.what());
            }
            catch (...)
            {
                detail::fail(ErrorCode::PARAM_UPDATE_REJECTED, "", external_path_,
                             "Kernel update threw an unknown exception");
            }
            cfg = std::move(normalized);
        }

        void rollback(Config& cfg) noexcept
        {
            try
            {
                Config restored(cfg);
                fillFromKernel(restored);
                cfg = std::move(restored);
            }
            catch (const RosParamsException& e)
            {
                state_ = DynamicState::SHUTDOWN;
                reportError(e.error(), handler_);
            }
            catch (...)
            {
                state_ = DynamicState::SHUTDOWN;
                reportError({ErrorCode::DYNAMIC_CONVERSION_ERROR, "", external_path_,
                             "UI rollback failed; adapter disabled"},
                            handler_);
            }
        }

        void callback(Config& cfg) noexcept
        {
            if (state() == DynamicState::INITIALIZING)
            {
                try
                {
                    fillFromKernel(cfg);
                }
                catch (...)
                {
                    initialization_error_ = std::current_exception();
                }
                return;
            }
            if (state() != DynamicState::ACTIVE)
            {
                rollback(cfg);
                return;
            }
            try
            {
                runtimeUpdate(cfg);
            }
            catch (const RosParamsException& e)
            {
                rollback(cfg);
                reportError(e.error(), handler_);
            }
            catch (const std::exception& e)
            {
                rollback(cfg);
                reportError({ErrorCode::DYNAMIC_CONVERSION_ERROR, "", external_path_, e.what()}, handler_);
            }
            catch (...)
            {
                rollback(cfg);
                reportError({ErrorCode::DYNAMIC_CONVERSION_ERROR, "", external_path_,
                             "Dynamic callback threw an unknown exception"},
                            handler_);
            }
        }

        ros::NodeHandle nh_;
        kernel::ParamServer& server_;
        std::string external_path_;
        const ErrorHandler& handler_;
        DynamicBindings<Config> bindings_;
        std::atomic<DynamicState> state_{DynamicState::UNINITIALIZED};
        std::exception_ptr initialization_error_;
        boost::recursive_mutex mutex_;
        std::unique_ptr<dynamic_reconfigure::Server<Config>> dynamic_server_;
    };
}
