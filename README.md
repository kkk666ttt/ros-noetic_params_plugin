# ros-noetic_params_plugin

ROS1 Noetic 官方参数适配层 v0.1，C++ 命名空间为 `vision::ros_params`，catkin 包名为 `ros_noetic_params_plugin`。这是普通 C++17/catkin 库，不使用 pluginlib。Core 保持独立；本库不控制 Kernel 生命周期，不创建算法线程，也不依赖业务 Config。

`vision::kernel::ParamServer` 是唯一运行时真值。YAML 是启动输入，generated dynamic_reconfigure Config 是编辑表示，算法只读取不可变 `ParamSnapshot`。

```text
启动：YAML → 一次 ParamServer::update → snapshot → dynamic cfg
运行：dynamic cfg → 一次 ParamServer::update → 算法 snapshot
失败：ParamServer 当前 snapshot → callback Config& → ROS response / parameter_updates
```

## 构建与验证

依赖 ROS1 Noetic 的 catkin、roscpp、dynamic_reconfigure，以及 yaml-cpp、独立安装的 vision_kernel。采用方案 A：vision_kernel 保持纯 standalone CMake，不添加 ROS package.xml、vendor 包或 rosdep rule。适配库的 package.xml 只声明可由 ROS 工具链识别的依赖，当前不声明 vision_kernel 这个尚无 package/rosdep 定义的名称；它仍是构建和下游接入的必需依赖，由用户先独立安装。其 CMake 包必须能通过 `find_package(vision_kernel CONFIG REQUIRED)` 找到。仅运行 rosdep 不会替你构建本地 Core。

非 catkin 库也可以成为 manifest 依赖：ROS 依赖标签使用 ROS package 名或系统 rosdep key，见 [REP 140](https://github.com/ros-infrastructure/rep/blob/master/rep-0140.rst)。当前项目没有提供 Core 的 ROS manifest 或 rosdep rule，所以 v0.1 按源码构建、预先安装 Core 的约定接入；不依赖 ROS 工具自动部署 Core。

以下命令在 bash 中执行，假设两个源码目录相邻：

```sh
source /opt/ros/noetic/setup.bash
cmake -S ../vision_kernel -B /tmp/ros-params-core-build \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF
cmake --build /tmp/ros-params-core-build --parallel 2
cmake --install /tmp/ros-params-core-build --prefix /tmp/ros-params-core-prefix

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_PREFIX_PATH='/tmp/ros-params-core-prefix;/opt/ros/noetic'
cmake --build build --parallel 2
(cd build && ctest --output-on-failure)
cmake --install build --prefix /tmp/ros-params-prefix
```

六组 CTest 各自启动一个隔离端口的 ROS master，自动清理。测试使用真实 generated Config、ROS service request/response 和多线程 spinner，不需要预先运行 roscore 或 rqt。generated 测试 Config 只存在于构建目录，不导出或安装。

业务工程通过 CMake 使用安装包：

```cmake
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure ros_noetic_params_plugin)
target_link_libraries(your_target PRIVATE vision::ros_params)
```

`CMAKE_PREFIX_PATH` 须包含本库、Core 和 ROS 的安装前缀。业务 generated Config 由业务包自己的 `generate_dynamic_reconfigure_options()` 构建，并自行添加相应生成目标依赖。`vision::ros_params` 自动提供 C++17、公开头文件及传递链接依赖。

公开头文件位于 `include/ros_params/`，以 `include/` 为引用根目录：`<ros_params/ros_params_manager.hpp>`、`<ros_params/error.hpp>`、`<ros_params/yaml/yaml_bindings.hpp>`、`<ros_params/dynamic/dynamic_registry.hpp>`。`ros_params/` 是安装后的头文件命名空间，避免将通用 error.hpp、detail/、yaml/、dynamic/ 名称直接放入全局 include 前缀。安装和源码构建使用同样的引用方式。

## 注册与启动

所有 Component 先完成 `onInit()`，随后创建 Manager、注册 Binding、初始化，最后启动 Kernel 和宿主 spinner。

```cpp
#include <kernel.hpp>
#include <ros_params/ros_params_manager.hpp>
#include <ros/ros.h>
#include <example/SampleConfig.h> // 使用方在自己的包中生成

vision::kernel::Kernel kernel;
registerComponents(kernel);

vision::ros_params::RosParamsManager manager(nh, kernel.paramServer());
manager.loadYamlFile(config_file); // 省略时使用空 YAML map
manager.yaml().bind<int>("sample.value", "settings/value");
manager.dynamic().add<example::SampleConfig>("sample")
    .bind<int>("sample.value", &example::SampleConfig::value);
manager.initialize();
kernel.start();

ros::AsyncSpinner spinner(2);
spinner.start();
// 宿主运行算法，只向 RUNNING 的组件派发任务。
ros::waitForShutdown();
spinner.stop();
manager.shutdown();
// 停止算法派发并等待任务结束后：
kernel.stop();
```

Manager 持有 `ParamServer&`，必须先于 Kernel 析构。管理操作和状态查询由宿主串行执行；注册表运行期间保持稳定。**shutdown/析构前必须停止并等待使用这些 Server 的 spinner/callback queue 消费线程。** 不得从 callback、converter 或 ErrorHandler 中销毁或重入 Manager。库不把 Noetic Server 的析构当成正在执行的 ROS request 已完成的保证。

## YAML Binding

第一个参数固定为 Kernel canonical path，第二个为独立的 YAML `/` 层级路径。

```cpp
using vision::ros_params::Presence;
auto& yaml = manager.yaml();
yaml.bind<double>("sample.scale", "settings/scale", Presence::REQUIRED);
yaml.bind<MyValue>("sample.value", "settings/value",
    [](const YAML::Node& node) -> MyValue { return decodeMyValue(node); });
```

默认 OPTIONAL：字段缺失时不产生更新，保留 Component 默认值。REQUIRED 缺失使整批失败。路径不允许为空、首尾 `/` 或空路径段；root 必须是 map，中间节点必须是 map。`null` 是存在的字段，基础 Codec 会拒绝它；显式 decoder 可自行处理。

基础 `YamlCodec<T>::decode()` 支持 bool、int/int32_t/int64_t、float、double、string 和递归 vector。业务可特化 `YamlCodec<MyValue>`。基础标量解码使用 yaml-cpp 的 `as<T>()` 直接得到声明的最终 C++ T；不先推断另一种 C++ 参数类型再转换。Kernel 类型校验始终精确匹配 `typeid(T)`，没有 Kernel 参数间的数值/字符串隐式转换。

初始化先利用一次 `ParamServer::list()` 校验全部 YAML 和 Dynamic Binding。未知路径、类型不匹配、YAML 内重复 canonical path、重复 YAML path 都失败，不修改参数。文件读取、所有解码完成后仅调用一次 `update()`；任一错误均不产生部分提交。未绑定的 YAML 键不自动映射为 Kernel 参数。

## Dynamic Binding

`DynamicRegistry::add<Config>(namespace)` 持有 Adapter 和 Server，可同时注册多个 generated Config 类型。同类型成员可以直接绑定，也可以显式提供双向转换：

```cpp
auto& view = manager.dynamic().add<example::SampleConfig>("sample");
view.bind<double>("sample.scale", &example::SampleConfig::scale);
view.bind<MyEnum>("sample.mode",
    [](const example::SampleConfig& cfg) { return static_cast<MyEnum>(cfg.mode); },
    [](example::SampleConfig& cfg, const MyEnum& value)
    { cfg.mode = static_cast<int>(value); });
```

Dynamic 只允许 RUNTIME_ABLE；绑定 START_ONLY 在初始化校验时失败，YAML 可以绑定两种模式。Registry 内 canonical path 必须唯一，解析后的 ROS namespace 也必须唯一；YAML 与 Dynamic 可以映射同一个 Kernel 参数。

Adapter 显式使用 UNINITIALIZED / INITIALIZING / ACTIVE / SHUTDOWN。INITIALIZING 期间任何 callback 只从 Kernel 回填 cfg，不提交参数；不依赖“第一次 callback”计数。每次回填一个 Config 使用同一份 snapshot。Noetic 的 `setCallback()` 会同步触发 callback，初始化转换错误会被 Adapter 记录，并在该调用返回后重新抛出，防止 ROS Server 吞掉错误后误进入 ACTIVE。

ACTIVE callback 解码所有绑定字段，提交前检查反向表示，然后仅调用一次 `update()`。一次非空 callback 成功产生一个新版本；不同 Server 之间没有跨 Server 原子性，也没有全局 ROS 参数 mutex。请求到达 Adapter 前，Noetic 会先执行 generated Config 的 `__clamp__()`；这些 cfg 范围属于外部表示，不进入 Core。

转换或更新失败时，重新获取当前 Kernel snapshot 回填 callback Config&，ROS Server 随后发布回滚值和 service response。不保存 last-good 参数副本。Converter 必须无参数服务写入/管理重入副作用，`to_cfg` 必须能正确表示所有合法 Kernel 值，相关双向映射由业务保证。generated Config 须支持 noexcept move assignment，以免提交后回填本地 Config 再抛异常。

若业务 `to_cfg` 对当前真值也抛异常，或回滚资源分配失败，则无法保证 UI 恢复；Adapter 报告错误并进入 SHUTDOWN，禁用后续写入，Server 由宿主正常 shutdown 清理。这不属于可恢复的普通候选参数错误。

初始化成功后，cfg 来自初始化快照；callback 成功后来自本次事务，失败后来自回滚快照。其他入口直接写 ParamServer 不会主动推送到 UI；没有 watch 或持续实时同步契约。生成的 Server 缓存是 ROS 编辑表示，不作为后续回滚的真值。

## 错误与生命周期

`RosParamsException::error()` 携带 ErrorCode、canonical_path、external_path 和 message。Dynamic 的 external_path 是解析后的 Server namespace。运行错误默认输出 ROS error 日志，也可在初始化前设置：

```cpp
manager.setErrorHandler([](const vision::ros_params::Error& error)
{
    // 多个 Server 可能并发调用，用户 handler 自行保证线程安全。
});
```

用户 handler 抛异常会被隔离并记录日志，不逃逸到 spinner。普通转换/更新错误不退出节点。

Manager 仅 CREATED 可以 initialize，一旦开始即封存全部 Binding；保留旧引用也不能继续注册。重复 initialize 为 INVALID_STATE。失败进入 SHUTDOWN，逆序清理已创建的 Server，不允许重试。shutdown 幂等。

YAML 事务成功后若 Dynamic 初始化失败，已经提交的 YAML 值保留；整个 initialize 不提供跨 YAML 与 ROS Server 的回滚事务。参数的生命周期与 Kernel Component 完全由宿主管理。

## 范围

不提供 pluginlib、动态加载、YAML watch/reload、START_ONLY runtime UI、自动生成 cfg、业务 validator、范围同步、跨 Server 事务、组件创建删除、Kernel 生命周期控制、Executor、Debug 或 ROS2。实际验证证据与限制见 [PROJECT_STATE.md](PROJECT_STATE.md)。
