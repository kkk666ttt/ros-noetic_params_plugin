#pragma once
#include <error.hpp>
#include <param_server.hpp>
#include <algorithm>
#include <string>
#include <typeindex>
#include <vector>

namespace vision::ros_params::detail
{
    struct BindingInfo
    {
        std::string canonical_path;
        std::string external_path;
        std::type_index type;
    };

    inline void
    fail(ErrorCode code, const std::string& path, const std::string& external, const std::string& message)
    {
        throw RosParamsException({code, path, external, message});
    }

    inline void
    validateBinding(const BindingInfo& binding, const std::vector<kernel::ParamInfo>& schema, bool dynamic)
    {
        const auto found = std::find_if(schema.begin(), schema.end(),
                                        [&](const auto& info)
                                        {
                                            return info.descriptor.path == binding.canonical_path;
                                        });
        if (found == schema.end())
        {
            fail(ErrorCode::UNKNOWN_KERNEL_PARAMETER, binding.canonical_path, binding.external_path,
                 "Kernel parameter is not registered");
        }
        if (found->descriptor.type != binding.type)
        {
            fail(ErrorCode::TYPE_MISMATCH, binding.canonical_path, binding.external_path,
                 "Binding requires the exact Kernel C++ type");
        }
        if (dynamic && found->descriptor.mode != kernel::ParamMode::RUNTIME_ABLE)
        {
            fail(ErrorCode::INVALID_PARAM_MODE, binding.canonical_path, binding.external_path,
                 "Dynamic binding requires RUNTIME_ABLE");
        }
    }
}
