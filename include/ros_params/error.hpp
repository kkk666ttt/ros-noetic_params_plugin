#pragma once
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace vision::ros_params
{
    enum class ErrorCode
    {
        INVALID_STATE,
        DUPLICATE_BINDING,
        UNKNOWN_KERNEL_PARAMETER,
        TYPE_MISMATCH,
        INVALID_PARAM_MODE,
        YAML_FILE_ERROR,
        YAML_MISSING_REQUIRED,
        YAML_DECODE_ERROR,
        DYNAMIC_CONVERSION_ERROR,
        PARAM_UPDATE_REJECTED
    };

    /** @brief ROS 表示层错误；external_path 为 YAML 路径或 dynamic server namespace。 */
    struct Error
    {
        ErrorCode code;
        std::string canonical_path;
        std::string external_path;
        std::string message;
    };

    using ErrorHandler = std::function<void(const Error&)>;

    /** @brief 初始化失败携带结构化错误，不吞掉失败或进入 ACTIVE。 */
    class RosParamsException : public std::runtime_error
    {
    public:
        explicit RosParamsException(Error error) : std::runtime_error(error.message), error_(std::move(error))
        {
        }

        const Error& error() const noexcept
        {
            return error_;
        }

    private:
        Error error_;
    };

    /** @brief 输出 ROS 日志或调用用户 handler；隔离用户 handler 的异常。 */
    void reportError(const Error& error, const ErrorHandler& handler) noexcept;
}
