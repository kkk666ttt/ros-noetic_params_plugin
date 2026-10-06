#include <ros_params/ros_params_manager.hpp>
#include <kernel.hpp>
#include <ros_noetic_params_plugin/PrimaryConfig.h>
#include <ros_noetic_params_plugin/SecondaryConfig.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <ros/ros.h>
#include <atomic>
#include <cstdint>
#include <exception>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unistd.h>

// 只在指定的 ROS callback 线程注入一次分配失败，其他线程不受影响。
thread_local bool reject_next_allocation = false;

void* operator new(std::size_t size)
{
    if (reject_next_allocation)
    {
        reject_next_allocation = false;
        throw std::bad_alloc();
    }
    if (void* memory = std::malloc(size ? size : 1))
    {
        return memory;
    }
    throw std::bad_alloc();
}

void operator delete(void* memory) noexcept
{
    std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept
{
    std::free(memory);
}

void* operator new[](std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void* memory) noexcept
{
    ::operator delete(memory);
}

void operator delete[](void* memory, std::size_t) noexcept
{
    ::operator delete(memory);
}

namespace vk = vision::kernel;
namespace rp = vision::ros_params;
using Primary = ros_noetic_params_plugin::PrimaryConfig;
using Secondary = ros_noetic_params_plugin::SecondaryConfig;

struct Value
{
    int number;
};
enum class Choice
{
    ONE = 1,
    TWO = 2
};

namespace vision::ros_params
{
    template <>
    struct YamlCodec<Value>
    {
        static Value decode(const YAML::Node& node)
        {
            return {node["number"].as<int>()};
        }
    };
}

namespace
{
    std::atomic<int> checks{0};

    void require(bool condition, const std::string& message)
    {
        ++checks;
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    template <class Function>
    void expectError(rp::ErrorCode code, Function function)
    {
        try
        {
            function();
        }
        catch (const rp::RosParamsException& error)
        {
            require(error.error().code == code, "Wrong structured error code: " + std::string(error.what()));
            return;
        }
        throw std::runtime_error("Expected RosParamsException");
    }

    class Unit : public vk::KernelComponent
    {
        void onInit() override
        {
            params().create<int>("alpha", 100, vk::ParamMode::RUNTIME_ABLE);
            params().create<int>("beta", 100, vk::ParamMode::RUNTIME_ABLE);
            params().create<int>("gamma", 100, vk::ParamMode::RUNTIME_ABLE);
            params().create<int>("startup", 10, vk::ParamMode::START_ONLY);
            params().create<double>("scale", 1.5, vk::ParamMode::RUNTIME_ABLE);
            params().create<float>("gain", 2.5f, vk::ParamMode::START_ONLY);
            params().create<bool>("enabled", false, vk::ParamMode::RUNTIME_ABLE);
            params().create<std::string>("label", "kernel-default", vk::ParamMode::RUNTIME_ABLE);
            params().create<std::int64_t>("large", 42, vk::ParamMode::START_ONLY);
            params().create<std::vector<int>>("array", {1, 2}, vk::ParamMode::START_ONLY);
            params().create<Value>("value", {9}, vk::ParamMode::START_ONLY);
            params().create<Choice>("choice", Choice::ONE, vk::ParamMode::RUNTIME_ABLE);
            for (int i = 0; i < 30; ++i)
            {
                params().create<int>("p" + std::to_string(i), i, vk::ParamMode::RUNTIME_ABLE);
            }
        }
    };

    struct Fixture
    {
        vk::Kernel kernel;
        ros::NodeHandle nh{"~"};

        Fixture()
        {
            kernel.create<Unit>("unit");
        }
    };

    struct File
    {
        std::string path;

        explicit File(const std::string& contents)
        {
            static unsigned serial = 0;
            path = "/tmp/ros-params-" + std::to_string(::getpid()) + "-" + std::to_string(serial++) + ".yaml";
            std::ofstream stream(path);
            stream << contents;
            if (!stream)
            {
                throw std::runtime_error("Cannot write test YAML");
            }
        }

        ~File()
        {
            std::remove(path.c_str());
        }
    };

    int readRos(const ros::NodeHandle& nh, const std::string& path)
    {
        int value = -1;
        require(nh.getParam(path, value), "Missing ROS representation " + path);
        return value;
    }

    template <class Config>
    Config request(ros::NodeHandle nh, const std::string& name, Config cfg)
    {
        auto client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(name + "/set_parameters");
        require(client.waitForExistence(ros::Duration(3)), "Dynamic service unavailable");
        dynamic_reconfigure::Reconfigure message;
        cfg.__toMessage__(message.request.config);
        require(client.call(message), "Dynamic service call failed");
        require(cfg.__fromMessage__(message.response.config), "Dynamic response cannot be decoded");
        return cfg;
    }

    void yamlBinding()
    {
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            File yaml("nested: {alpha: 180, startup: 20, scale: 2.25, enabled: true, label: text, gain: 3.5, "
                      "large: 5000000000, array: [3, 4], value: {number: 77}}\n");
            manager.loadYamlFile(yaml.path);
            auto& bindings = manager.yaml();
            bindings.bind<int>("unit.alpha", "nested/alpha", rp::Presence::REQUIRED);
            bindings.bind<int>("unit.startup", "nested/startup");
            bindings.bind<double>("unit.scale", "nested/scale");
            bindings.bind<bool>("unit.enabled", "nested/enabled");
            bindings.bind<std::string>("unit.label", "nested/label");
            bindings.bind<float>("unit.gain", "nested/gain");
            bindings.bind<std::int64_t>("unit.large", "nested/large");
            bindings.bind<std::vector<int>>("unit.array", "nested/array");
            bindings.bind<Value>("unit.value", "nested/value");
            bindings.bind<int>("unit.beta", "missing");
            const auto before = f.kernel.paramServer().snapshot();
            manager.initialize();
            const auto snapshot = f.kernel.paramServer().snapshot();
            require(snapshot.version() == before.version() + 1, "YAML must publish one version");
            require(snapshot.get<int>("unit.alpha") == 180, "YAML alpha");
            require(snapshot.get<int>("unit.startup") == 20, "START_ONLY startup YAML");
            require(snapshot.get<int>("unit.beta") == 100, "OPTIONAL default");
            require(snapshot.get<double>("unit.scale") == 2.25, "Double codec");
            require(snapshot.get<float>("unit.gain") == 3.5f, "Float codec");
            require(snapshot.get<bool>("unit.enabled"), "Bool codec");
            require(snapshot.get<std::string>("unit.label") == "text", "String codec");
            require(snapshot.get<std::int64_t>("unit.large") == 5000000000LL, "Int64 codec");
            require(snapshot.get<std::vector<int>>("unit.array") == std::vector<int>({3, 4}), "Vector codec");
            require(snapshot.get<Value>("unit.value").number == 77, "Custom codec");
            require(before.get<int>("unit.alpha") == 100, "Historical snapshot unchanged");
        }
        for (int scenario = 0; scenario < 6; ++scenario)
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            rp::ErrorCode error = rp::ErrorCode::DUPLICATE_BINDING;
            if (scenario == 0)
            {
                manager.yaml().bind<int>("missing.path", "value"),
                    error = rp::ErrorCode::UNKNOWN_KERNEL_PARAMETER;
            }
            if (scenario == 1)
            {
                manager.yaml().bind<double>("unit.alpha", "value"), error = rp::ErrorCode::TYPE_MISMATCH;
            }
            if (scenario == 2)
            {
                manager.yaml().bind<int>("unit.alpha", "a");
                manager.yaml().bind<int>("unit.alpha", "b");
            }
            if (scenario == 3)
            {
                manager.yaml().bind<int>("unit.alpha", "a");
                manager.yaml().bind<int>("unit.beta", "a");
            }
            if (scenario == 4)
            {
                manager.yaml().bind<int>("unit.alpha", "/a"), error = rp::ErrorCode::YAML_DECODE_ERROR;
            }
            if (scenario == 5)
            {
                manager.yaml().bind<int>("unit.alpha", "a//b"), error = rp::ErrorCode::YAML_DECODE_ERROR;
            }
            const auto before = f.kernel.paramServer().snapshot();
            expectError(error,
                        [&]
                        {
                            manager.initialize();
                        });
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "Validation must not write");
            require(manager.state() == rp::ManagerState::SHUTDOWN, "Failed initialization is terminal");
        }
        require(rp::YamlCodec<std::int32_t>::decode(YAML::Load("23")) == 23, "Int32 codec");
    }

    void yamlTransaction()
    {
        for (int scenario = 0; scenario < 7; ++scenario)
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            manager.yaml().bind<int>("unit.alpha", "alpha");
            manager.yaml().bind<int>("unit.beta", "beta", rp::Presence::REQUIRED);
            const std::vector<std::string> text = {"alpha: 180\n",
                                                   "alpha: 180\nbeta: invalid\n",
                                                   "alpha: 180\nbeta: null\n",
                                                   "alpha: 180\nbeta: [1, 2]\n",
                                                   "[1, 2]\n",
                                                   "alpha: [\n",
                                                   "alpha: 180\nbeta: 20\n"};
            File yaml(text[scenario]);
            manager.loadYamlFile(scenario == 6 ? yaml.path + ".missing" : yaml.path);
            const auto before = f.kernel.paramServer().snapshot();
            const auto error = scenario == 0  ? rp::ErrorCode::YAML_MISSING_REQUIRED
                               : scenario < 4 ? rp::ErrorCode::YAML_DECODE_ERROR
                                              : rp::ErrorCode::YAML_FILE_ERROR;
            expectError(error,
                        [&]
                        {
                            manager.initialize();
                        });
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "YAML failure must be atomic");
            require(f.kernel.paramServer().snapshot().get<int>("unit.alpha") == 100,
                    "First field must stay default");
        }
        for (int bad : {0, 15, 26, 29})
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            std::string contents;
            for (int i = 0; i < 30; ++i)
            {
                manager.yaml().bind<int>("unit.p" + std::to_string(i), "p" + std::to_string(i));
                contents += "p" + std::to_string(i) + ": " + (i == bad ? "wrong" : "999") + "\n";
            }
            File yaml(contents);
            manager.loadYamlFile(yaml.path);
            const auto before = f.kernel.paramServer().snapshot();
            expectError(rp::ErrorCode::YAML_DECODE_ERROR,
                        [&]
                        {
                            manager.initialize();
                        });
            const auto after = f.kernel.paramServer().snapshot();
            require(after.version() == before.version(), "30-field batch must not publish on failure");
            for (int i = 0; i < 30; ++i)
            {
                require(after.get<int>("unit.p" + std::to_string(i)) == i, "Partial YAML commit");
            }
        }
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            manager.yaml().bind<Choice>("unit.choice", "choice",
                                        [](const YAML::Node& node)
                                        {
                                            return static_cast<Choice>(node.as<int>());
                                        });
            File yaml("choice: 2\n");
            manager.loadYamlFile(yaml.path);
            manager.initialize();
            require(f.kernel.paramServer().snapshot().get<Choice>("unit.choice") == Choice::TWO,
                    "Explicit YAML decoder");
        }
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            manager.yaml().bind<int>("unit.startup", "startup");
            File yaml("startup: 99\n");
            manager.loadYamlFile(yaml.path);
            f.kernel.start();
            const auto before = f.kernel.paramServer().snapshot();
            expectError(rp::ErrorCode::PARAM_UPDATE_REJECTED,
                        [&]
                        {
                            manager.initialize();
                        });
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "Frozen YAML update rejected atomically");
        }
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            manager.yaml().bind<int>("unit.alpha", "parent/alpha");
            File yaml("parent: 3\n");
            manager.loadYamlFile(yaml.path);
            expectError(rp::ErrorCode::YAML_DECODE_ERROR,
                        [&]
                        {
                            manager.initialize();
                        });
        }
    }

    void dynamicBinding()
    {
        for (int scenario = 0; scenario < 5; ++scenario)
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            auto& adapter = manager.dynamic().add<Primary>("first");
            rp::ErrorCode error = rp::ErrorCode::DUPLICATE_BINDING;
            if (scenario == 0)
            {
                adapter.bind<int>("unit.startup", &Primary::alpha), error = rp::ErrorCode::INVALID_PARAM_MODE;
            }
            if (scenario == 1)
            {
                adapter.bind<double>("unit.alpha", &Primary::scale), error = rp::ErrorCode::TYPE_MISMATCH;
            }
            if (scenario == 2)
            {
                adapter.bind<int>("absent.parameter", &Primary::alpha),
                    error = rp::ErrorCode::UNKNOWN_KERNEL_PARAMETER;
            }
            if (scenario == 3)
            {
                adapter.bind<int>("unit.alpha", &Primary::alpha);
                adapter.bind<int>("unit.alpha", &Primary::beta);
            }
            if (scenario == 4)
            {
                adapter.bind<int>("unit.alpha", &Primary::alpha);
                manager.dynamic().add<Secondary>("second").bind<int>("unit.alpha", &Secondary::gamma);
            }
            File yaml("alpha: 180\n");
            manager.yaml().bind<int>("unit.alpha", "alpha");
            manager.loadYamlFile(yaml.path);
            const auto before = f.kernel.paramServer().snapshot();
            expectError(error,
                        [&]
                        {
                            manager.initialize();
                        });
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "Dynamic validation precedes YAML commit");
        }
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            manager.dynamic().add<Primary>("first");
            expectError(rp::ErrorCode::DUPLICATE_BINDING,
                        [&]
                        {
                            manager.dynamic().add<Secondary>(f.nh.getNamespace() + "/first");
                        });
        }
    }

    void dynamicInitialization()
    {
        Fixture f;
        rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
        File yaml("alpha: 180\nbeta: 180\n");
        manager.loadYamlFile(yaml.path);
        manager.yaml().bind<int>("unit.alpha", "alpha");
        manager.yaml().bind<int>("unit.beta", "beta");
        auto& adapter = manager.dynamic().add<Primary>("first");
        adapter.bind<int>("unit.alpha", &Primary::alpha);
        adapter.bind<int>("unit.beta", &Primary::beta);
        adapter.bind<double>("unit.scale", &Primary::scale);
        adapter.bind<bool>("unit.enabled", &Primary::enabled);
        adapter.bind<std::string>("unit.label", &Primary::label);
        manager.dynamic().add<Secondary>("second").bind<int>("unit.gamma", &Secondary::gamma);
        const auto before = f.kernel.paramServer().snapshot();
        manager.initialize();
        require(manager.state() == rp::ManagerState::ACTIVE, "Manager ACTIVE");
        require(adapter.state() == rp::DynamicState::ACTIVE, "Adapter ACTIVE");
        require(f.kernel.paramServer().snapshot().version() == before.version() + 1,
                "Init callback must not write");
        require(f.kernel.paramServer().snapshot().get<int>("unit.alpha") == 180,
                "cfg default must not replace YAML");
        require(readRos(f.nh, "first/alpha") == 180, "cfg initialization from Kernel");
        require(readRos(f.nh, "first/beta") == 180, "Same Config fields initialized");
        require(readRos(f.nh, "second/gamma") == 100, "Second Config initialized");
        f.kernel.start();
        ros::AsyncSpinner spinner(4);
        spinner.start();
        const auto old = f.kernel.paramServer().snapshot();
        auto cfg = Primary::__getDefault__();
        cfg.alpha = cfg.beta = 222;
        cfg.label = "runtime";
        cfg.scale = 3.0;
        cfg.enabled = true;
        cfg = request(f.nh, "first", cfg);
        const auto next = f.kernel.paramServer().snapshot();
        require(next.version() == old.version() + 1, "One Config callback must publish one version");
        require(next.get<int>("unit.alpha") == 222 && next.get<int>("unit.beta") == 222, "Runtime batch");
        require(next.get<std::string>("unit.label") == "runtime" && next.get<bool>("unit.enabled"),
                "Non-int dynamic fields");
        require(old.get<int>("unit.alpha") == 180, "Old snapshot unchanged after callback");
        std::atomic<bool> reading{true}, coherent{true};
        std::thread reader(
            [&]
            {
                while (reading.load())
                {
                    const auto snapshot = f.kernel.paramServer().snapshot();
                    if (snapshot.get<int>("unit.alpha") != snapshot.get<int>("unit.beta"))
                    {
                        coherent = false;
                    }
                }
            });
        std::exception_ptr first_error, second_error;
        std::thread first(
            [&]
            {
                try
                {
                    for (int i = 0; i < 40; ++i)
                    {
                        cfg.alpha = cfg.beta = 300 + i;
                        request(f.nh, "first", cfg);
                    }
                }
                catch (...)
                {
                    first_error = std::current_exception();
                }
            });
        std::thread second(
            [&]
            {
                try
                {
                    auto other = Secondary::__getDefault__();
                    for (int i = 0; i < 40; ++i)
                    {
                        other.gamma = 500 + i;
                        request(f.nh, "second", other);
                    }
                }
                catch (...)
                {
                    second_error = std::current_exception();
                }
            });
        first.join();
        second.join();
        reading = false;
        reader.join();
        spinner.stop();
        manager.shutdown();
        if (first_error)
        {
            std::rethrow_exception(first_error);
        }
        if (second_error)
        {
            std::rethrow_exception(second_error);
        }
        require(coherent.load(), "Concurrent reader observed mixed Config transaction");
        require(f.kernel.paramServer().snapshot().get<int>("unit.gamma") == 539,
                "Concurrent second Config updates");
        require(!ros::service::exists(f.nh.resolveName("first/set_parameters"), false),
                "Server destroyed on shutdown");
    }

    void dynamicRollback()
    {
        Fixture f;
        rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
        File yaml("alpha: 120\n");
        manager.loadYamlFile(yaml.path);
        manager.yaml().bind<int>("unit.alpha", "alpha");
        std::atomic<int> errors{0}, code{-1};
        manager.setErrorHandler(
            [&](const rp::Error& error)
            {
                ++errors;
                code = static_cast<int>(error.code);
                throw std::runtime_error("Handler must not escape callback");
            });
        auto& adapter = manager.dynamic().add<Primary>("first");
        adapter.bind<int>(
            "unit.alpha",
            [](const Primary& cfg)
            {
                if (cfg.alpha == 200)
                {
                    throw std::runtime_error("Synthetic converter failure");
                }
                if (cfg.alpha == 201)
                {
                    throw 7;
                }
                return cfg.alpha;
            },
            [](Primary& cfg, const int& value)
            {
                cfg.alpha = value;
            });
        adapter.bind<int>(
            "unit.beta",
            [](const Primary& cfg)
            {
                return cfg.beta;
            },
            [](Primary& cfg, const int& value)
            {
                cfg.beta = value;
                if (value == 777)
                {
                    reject_next_allocation = true;
                }
                if (value == 778)
                {
                    throw std::runtime_error("Synthetic reverse converter failure");
                }
            });
        manager.dynamic().add<Secondary>("enum").bind<Choice>(
            "unit.choice",
            [](const Secondary& cfg)
            {
                return static_cast<Choice>(cfg.gamma);
            },
            [](Secondary& cfg, const Choice& value)
            {
                cfg.gamma = static_cast<int>(value);
            });
        manager.initialize();
        require(readRos(f.nh, "first/alpha") == 120, "YAML to cfg");
        f.kernel.start();
        ros::AsyncSpinner spinner(2);
        spinner.start();
        auto cfg = Primary::__getDefault__();
        cfg.alpha = 150;
        cfg.beta = 150;
        cfg = request(f.nh, "first", cfg);
        require(f.kernel.paramServer().snapshot().get<int>("unit.alpha") == 150,
                "Single source updated to 150");
        // 外部直接写 Kernel 后，失败回滚必须重新读取真实值，不能回到缓存的 150。
        require(static_cast<bool>(f.kernel.paramServer().set<int>("unit.alpha", 160)),
                "External Kernel update");
        for (int candidate : {200, 201})
        {
            const auto before = f.kernel.paramServer().snapshot();
            cfg.alpha = candidate;
            cfg.beta = 999;
            cfg = request(f.nh, "first", cfg);
            require(cfg.alpha == 160 && cfg.beta == 150, "Converter failure rollback from current Kernel");
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "Converter failure must not commit");
            require(code == static_cast<int>(rp::ErrorCode::DYNAMIC_CONVERSION_ERROR),
                    "Conversion error classification");
        }
        for (int candidate : {777, 778})
        {
            const auto before = f.kernel.paramServer().snapshot();
            cfg.alpha = 190;
            cfg.beta = candidate;
            cfg = request(f.nh, "first", cfg);
            require(cfg.alpha == 160 && cfg.beta == 150, "Failed transaction response restores cfg");
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "Failed transaction version unchanged");
            require(code == static_cast<int>(candidate == 777 ? rp::ErrorCode::PARAM_UPDATE_REJECTED
                                                              : rp::ErrorCode::DYNAMIC_CONVERSION_ERROR),
                    "Update failure classification");
            require(readRos(f.nh, "first/alpha") == 160, "Published ROS representation rolled back");
        }
        auto enum_cfg = Secondary::__getDefault__();
        enum_cfg.gamma = 2;
        enum_cfg = request(f.nh, "enum", enum_cfg);
        require(f.kernel.paramServer().snapshot().get<Choice>("unit.choice") == Choice::TWO,
                "Custom enum converter");
        cfg.alpha = cfg.beta = 170;
        request(f.nh, "first", cfg);
        require(f.kernel.paramServer().snapshot().get<int>("unit.alpha") == 170,
                "Callback still works after errors");
        require(errors == 4, "Each error reported once");
        spinner.stop();
        manager.shutdown();
    }

    void managerLifecycle()
    {
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            int conversion_calls = 0;
            manager.dynamic()
                .add<Primary>("initial_callback_failure")
                .bind<int>(
                    "unit.alpha",
                    [](const Primary& cfg)
                    {
                        return cfg.alpha;
                    },
                    [&](Primary& cfg, const int& value)
                    {
                        if (++conversion_calls == 2)
                        {
                            throw std::runtime_error("Initialization callback converter failed");
                        }
                        cfg.alpha = value;
                    });
            const auto before = f.kernel.paramServer().snapshot();
            expectError(rp::ErrorCode::DYNAMIC_CONVERSION_ERROR,
                        [&]
                        {
                            manager.initialize();
                        });
            require(conversion_calls == 2,
                    "Failure occurred inside real setCallback initialization callback");
            require(f.kernel.paramServer().snapshot().version() == before.version(),
                    "Failing init callback never writes");
            require(!ros::service::exists(f.nh.resolveName("initial_callback_failure/set_parameters"), false),
                    "Failed initialization callback server removed");
        }
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            auto& yaml = manager.yaml();
            auto& dynamic = manager.dynamic();
            auto& adapter = dynamic.add<Primary>("first");
            auto& bindings = adapter.bindings();
            bindings.bind<int>("unit.alpha", &Primary::alpha);
            const auto version = f.kernel.paramServer().snapshot().version();
            manager.initialize();
            require(f.kernel.paramServer().snapshot().version() == version,
                    "Empty YAML and init callbacks do not publish");
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            manager.initialize();
                        });
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            yaml.bind<int>("unit.beta", "b");
                        });
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            bindings.bind<int>("unit.beta", &Primary::beta);
                        });
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            dynamic.add<Secondary>("second");
                        });
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            manager.loadYamlFile("late.yaml");
                        });
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            manager.setErrorHandler({});
                        });
            manager.shutdown();
            manager.shutdown();
            require(adapter.state() == rp::DynamicState::SHUTDOWN, "Adapter shuts down");
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            manager.initialize();
                        });
            require(!ros::service::exists(f.nh.resolveName("first/set_parameters"), false),
                    "Shutdown removes service");
        }
        {
            Fixture f;
            rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
            File yaml("alpha: 180\n");
            manager.loadYamlFile(yaml.path);
            manager.yaml().bind<int>("unit.alpha", "alpha");
            manager.dynamic().add<Primary>("first").bind<int>("unit.alpha", &Primary::alpha);
            manager.dynamic().add<Secondary>("second").bind<int>(
                "unit.gamma",
                [](const Secondary& cfg)
                {
                    return cfg.gamma;
                },
                [](Secondary&, const int&)
                {
                    throw std::runtime_error("Cannot represent Kernel value");
                });
            expectError(rp::ErrorCode::DYNAMIC_CONVERSION_ERROR,
                        [&]
                        {
                            manager.initialize();
                        });
            require(manager.state() == rp::ManagerState::SHUTDOWN, "Partial dynamic init cleans up");
            require(f.kernel.paramServer().snapshot().get<int>("unit.alpha") == 180,
                    "Successful YAML transaction retained");
            require(!ros::service::exists(f.nh.resolveName("first/set_parameters"), false),
                    "Earlier Server cleaned after later failure");
            expectError(rp::ErrorCode::INVALID_STATE,
                        [&]
                        {
                            manager.initialize();
                        });
        }
        {
            Fixture f;
            {
                rp::RosParamsManager manager(f.nh, f.kernel.paramServer());
                manager.dynamic().add<Primary>("first").bind<int>("unit.alpha", &Primary::alpha);
                manager.initialize();
            }
            require(!ros::service::exists(f.nh.resolveName("first/set_parameters"), false),
                    "Destructor removes server");
            require(f.kernel.paramServer().snapshot().get<int>("unit.alpha") == 100,
                    "Kernel outlives Manager");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_params_contract_test", ros::init_options::AnonymousName);
    try
    {
        if (argc != 2)
        {
            throw std::runtime_error("Expected test suite argument");
        }
        const std::string suite(argv[1]);
        if (suite == "yaml_binding")
        {
            yamlBinding();
        }
        else if (suite == "yaml_transaction")
        {
            yamlTransaction();
        }
        else if (suite == "dynamic_binding")
        {
            dynamicBinding();
        }
        else if (suite == "dynamic_initialization")
        {
            dynamicInitialization();
        }
        else if (suite == "dynamic_rollback")
        {
            dynamicRollback();
        }
        else if (suite == "manager_lifecycle")
        {
            managerLifecycle();
        }
        else
        {
            throw std::runtime_error("Unknown test suite");
        }
        std::cout << suite << ": " << checks.load() << " checks passed\n";
        ros::shutdown();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAILED after " << checks.load() << " checks: " << error.what() << '\n';
        ros::shutdown();
        return 1;
    }
}
