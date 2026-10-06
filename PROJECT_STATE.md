# ros-noetic_params_plugin 项目状态

更新日期：2026-10-06。契约与接入说明见 [README.md](README.md)。

## 已实现

- C++17/catkin 独立库；namespace `vision::ros_params`，包 `ros_noetic_params_plugin`，链接目标 `vision::ros_params`。
- RosParamsManager 初始化/关闭与 Binding 封存；Kernel 公共 API 和源码未修改。
- YAML canonical path Binding、OPTIONAL/REQUIRED、基础 Codec、自定义 decoder/Codec、全量 schema 校验及单批事务。
- 类型擦除 DynamicRegistry、多 generated Config、成员和双向 converter Binding、RUNTIME_ABLE 校验。
- 显式初始化状态机；Kernel 同版本快照到 cfg，运行 callback 单批更新、失败从当前 Kernel 回填，ErrorHandler 异常隔离。
- 六组契约测试，真实 generated Config/ROS service、隔离 master、多 Server 并发及算法快照读取；一次更新分配失败注入。
- 安装配置导入独立 Core/yaml-cpp，测试 generated Config 不安装或导出。

## 验证记录

本地环境为 GCC 9.4、ROS1 Noetic、yaml-cpp 和 Python 3.8。Debug 下六组 CTest 共 520 项检查通过；同样的六组测试在 Core、Adapter 和测试程序均启用 AddressSanitizer / UndefinedBehaviorSanitizer 的构建中通过。包含第 1/16/27/30 项解码失败的 30 参数事务、默认值 128 不覆盖 YAML 180、初始化 callback 不发布版本、多字段单版本、转换错误/更新分配失败回滚及 ErrorHandler 抛异常。

未修改的 Core 在临时独立构建中三套 CTest（普通、分配失败、Eigen）及 threshold_demo 通过。

安装到独立临时前缀后，另一个项目通过 catkin 的 find_package、导入的 `vision::ros_params` 和安装后的 Core 编译运行，初始化 cfg = Kernel 180，未使用适配库源码头文件。公开头文件独立包含检查与新增文件空白检查通过。测试 Config 未出现在安装前缀中。

本地测试通过不表示远端 CI 成功；尚未配置或查询远端 CI，也未启动交互式 rqt。测试验证了 rqt 使用的真实 service response 和 ROS 参数表示，没有交互 GUI 验证。

## 明确边界

- 生命周期管理由宿主串行执行；shutdown/析构前停止并等待相关 spinner。不能在 callback/converter/handler 中重入管理操作。
- initialize 失败不可重试；YAML 已提交后 Dynamic 失败不撤销该 YAML 事务。
- 外部直接写 ParamServer 不主动同步到 UI；跨 Dynamic Server 没有原子事务。
- 回滚 converter 必须能表示当前合法 Kernel 值。回滚本身失败时报告错误并禁用 Adapter，不能声称 UI 已恢复。
- 稳定注册表中正常 Binding 不会自然触发 UNKNOWN/TYPE/FROZEN 拒绝；Dynamic 的更新失败测试通过线程局部的一次分配失败进入真实 ParamServer::update 异常路径，未破坏 Kernel 的 create/remove 并发边界。
- ROS 编译依赖保持在本库；业务 Config 只在使用方生成，库不导入任何业务算法。
