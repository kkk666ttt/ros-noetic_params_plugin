#include <error.hpp>
#include <ros/console.h>
#include <exception>

namespace vision::ros_params
{
    void reportError(const Error& error, const ErrorHandler& handler) noexcept
    {
        try
        {
            if (handler)
            {
                handler(error);
            }
            else
            {
                ROS_ERROR_STREAM("ros_params: " << error.canonical_path << " [" << error.external_path << "] "
                                                << error.message);
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("ros_params ErrorHandler threw: %s", e.what());
        }
        catch (...)
        {
            ROS_ERROR("ros_params ErrorHandler threw an unknown exception");
        }
    }
}
