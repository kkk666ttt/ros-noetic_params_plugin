#include <ros_params_manager.hpp>
#include <exception>
#include <utility>

namespace vision::ros_params
{
    RosParamsManager::RosParamsManager(ros::NodeHandle nh, kernel::ParamServer& server)
        : nh_(std::move(nh)), server_(server), yaml_adapter_(server), dynamic_registry_(nh_, server, handler_)
    {
    }

    RosParamsManager::~RosParamsManager()
    {
        shutdown();
    }

    void RosParamsManager::checkCreated() const
    {
        if (state_ != ManagerState::CREATED)
        {
            detail::fail(ErrorCode::INVALID_STATE, "", "", "Manager requires CREATED state");
        }
    }

    void RosParamsManager::loadYamlFile(std::string path)
    {
        checkCreated();
        yaml_adapter_.loadFile(std::move(path));
    }

    void RosParamsManager::setErrorHandler(ErrorHandler handler)
    {
        checkCreated();
        handler_ = std::move(handler);
    }

    void RosParamsManager::initialize()
    {
        checkCreated();
        state_ = ManagerState::INITIALIZING;
        yaml_adapter_.seal();
        dynamic_registry_.seal();
        try
        {
            const auto schema = server_.list();
            yaml_adapter_.validate(schema);
            dynamic_registry_.validate(schema);
            yaml_adapter_.apply();
            dynamic_registry_.initializeFromKernel();
            state_ = ManagerState::ACTIVE;
        }
        catch (...)
        {
            shutdown();
            throw;
        }
    }

    void RosParamsManager::shutdown() noexcept
    {
        yaml_adapter_.seal();
        dynamic_registry_.seal();
        state_ = ManagerState::SHUTDOWN;
        dynamic_registry_.shutdown();
    }
}
