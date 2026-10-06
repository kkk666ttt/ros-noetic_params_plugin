#pragma once
#include <ros_params/dynamic/dynamic_adapter.hpp>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vision::ros_params
{
    class RosParamsManager;

    /** @brief 持有多个 Config Server；同一个 Kernel path 在 Registry 内唯一。 */
    class DynamicRegistry
    {
    public:
        DynamicRegistry(ros::NodeHandle nh, kernel::ParamServer& server, const ErrorHandler& handler)
            : nh_(std::move(nh)), server_(server), handler_(handler)
        {
        }

        DynamicRegistry(const DynamicRegistry&) = delete;
        DynamicRegistry& operator=(const DynamicRegistry&) = delete;

        template <class Config>
        DynamicParamAdapter<Config>& add(const std::string& ros_namespace)
        {
            if (sealed_)
            {
                detail::fail(ErrorCode::INVALID_STATE, "", ros_namespace, "Dynamic registry is sealed");
            }
            ros::NodeHandle child(nh_, ros_namespace);
            const auto resolved = child.getNamespace();
            if (!namespaces_.insert(resolved).second)
            {
                detail::fail(ErrorCode::DUPLICATE_BINDING, "", resolved, "Duplicate resolved ROS namespace");
            }
            try
            {
                auto adapter =
                    std::make_unique<DynamicParamAdapter<Config>>(child, server_, resolved, handler_);
                auto& result = *adapter;
                adapters_.push_back(std::move(adapter));
                return result;
            }
            catch (...)
            {
                namespaces_.erase(resolved);
                throw;
            }
        }

    private:
        friend class RosParamsManager;

        void seal() noexcept
        {
            sealed_ = true;
            for (const auto& adapter : adapters_)
            {
                adapter->seal();
            }
        }

        void validate(const std::vector<kernel::ParamInfo>& schema) const
        {
            std::unordered_set<std::string> paths;
            for (const auto& adapter : adapters_)
            {
                for (const auto& info : adapter->bindingInfo())
                {
                    if (!paths.insert(info.canonical_path).second)
                    {
                        detail::fail(ErrorCode::DUPLICATE_BINDING, info.canonical_path, info.external_path,
                                     "Duplicate canonical path in DynamicRegistry");
                    }
                    detail::validateBinding(info, schema, true);
                }
            }
        }

        void initializeFromKernel()
        {
            for (const auto& adapter : adapters_)
            {
                adapter->initializeFromKernel();
            }
        }

        void shutdown() noexcept
        {
            for (auto it = adapters_.rbegin(); it != adapters_.rend(); ++it)
            {
                (*it)->shutdown();
            }
        }

        ros::NodeHandle nh_;
        kernel::ParamServer& server_;
        const ErrorHandler& handler_;
        bool sealed_ = false;
        std::unordered_set<std::string> namespaces_;
        std::vector<std::unique_ptr<IDynamicAdapter>> adapters_;
    };
}
