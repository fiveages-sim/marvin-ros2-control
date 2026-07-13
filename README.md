# Marvin ROS2 Control

Marvin 机械臂（M6 等）的 ROS2 Control 硬件接口。通过 **Marvin SDK** 连接控制器，在臂体 **RS485 / 场总线 CAN** 上驱动末端工具。插件：`marvin_ros2_control/MarvinHardware`。

独立 USB 转接的末端请用 [modbus_ros2_control](https://github.com/fiveages-sim/modbus-ros2-control) 或 [can_ros2_control](https://github.com/fiveages-sim/can-ros2-control)，不走本包。

## 1. 支持的末端执行器

`MarvinHardware::createTool` 按 `left_ee_type` / `right_ee_type`（大写归一化）创建工具。双臂：左 → A 通道，右 → B 通道。

| 类型键（常用） | 产品 | 实现 | 默认总线 |
|----------------|------|------|----------|
| `RG75`, `JDGRIPPER` | Jodell RG75 | `JDGripper` | RS485（COM1） |
| `eincinx`, `epgi180`, `EINCINX` | EincinX / EPGI180 电动夹爪 | `EincinXGripper` | RS485（COM1，夹持轴站号 2） |
| `ERG32`, `erg32` | Jodell ERG32-150（旋转 + 夹爪，2-DOF） | `ERG32Hand` | RS485（COM1） |
| `AG2F90_C`, `AG2F90C`, `CHANGINGTEK90C` | ChangingTek AG2F90-C | `ChangingtekGripper90C` | RS485 |
| `AG2F90_D`, `AG2F90D`, `CHANGINGTEK90D` | ChangingTek AG2F90-D | `ChangingtekGripper90D` | RS485 |
| `AG2F120S` | ChangingTek AG2F120S | `ChangingtekGripper120S` | RS485 |
| `AG2F120S_D` | ChangingTek AG2F120S-D | `ChangingtekGripper120S_D` | RS485 |
| `linkerhand_o7`, `O7` | LinkerHand O7 | `DexterousHandO7` | RS485 |
| `linkerhand_o6`, `O6` | LinkerHand O6 | `DexterousHandO6` | RS485 |
| `linkerhand_l6`, `L6` | LinkerHand L6 | `DexterousHandL6` | RS485 |
| `freedom_v1`, `FREEDOM` | Freedom V1 | `FreedomHandV1` | RS485；`ee_channel=1` 时 CAN |
| `freedom_v2` | Freedom V2 | `FreedomHandV2` | RS485；`ee_channel=1` 时 CAN |
| `inspire_e2`, `RH56E2`, `INSPIRE` | Inspire RH56E2 | `InspireHandE2` / `InspireHandE2Canfd` | RS485；`ee_channel=1` 时 CAN FD |
| `inspire_f2`, `RH56F2` | Inspire RH56F2 | 同上（F2 类型） | 同上 |

未知夹爪类型回退为 `JDGripper`；未知灵巧手类型回退为 O7。

## 2. 代码结构

```
marvin_ros2_control/
├── src/marvin_hardware.cpp          # MarvinHardware 入口
├── src/tool/                        # 部分末端 .cpp 实现（如 jd_gripper、erg32_hand）
├── include/marvin_ros2_control/tool/
│   ├── modbus_io.h                  # SDK 485/CAN 帧收发
│   ├── grippers/                    # RG75（jodell/）、Changingtek
│   └── hands/                       # LinkerHand、Freedom、Inspire、ERG32（jodell/）
└── external/TJ_FX_ROBOT_CONTRL_SDK/ # Marvin SDK（子模块）
```

夹爪实现复用 `gripper_hardware_common`；灵巧手协议在包内实现（Inspire 与 modbus/can 栈并行，见 [§8 TODO](#8-todo)）。

## 3. 配置参考

在机器人 `*_description` 的 `ros2_control` 中配置 `hardware_parameters`（示例）：

```xml
<ros2_control name="M6_CCS_System" type="system">
  <hardware>
    <plugin>marvin_ros2_control/MarvinHardware</plugin>
    <param name="arm_type">dual</param>
    <param name="device_ip">192.168.1.190</param>
    <param name="left_ee_type">erg32</param>
    <param name="right_ee_type">linkerhand_o7</param>
    <!-- 可选：场总线 CAN（tianji_can） -->
  <!-- <param name="left_ee_channel">1</param> -->
  <!-- <param name="right_ee_channel">1</param> -->
    <param name="ctrl_mode">position</param>
    <param name="max_joint_speed">50</param>
    <param name="max_joint_acceleration">30</param>
  </hardware>
  <!-- 臂 + 末端关节接口由 description xacro 注入 -->
</ros2_control>
```

### 3.1 常用 `hardware_parameters`

| 参数 | 说明 |
|------|------|
| `arm_type` | `left` / `right` / `dual` |
| `device_ip` | 控制器 IP |
| `device_port` | 端口（可选） |
| `left_ee_type` / `right_ee_type` | 末端类型键（见 §1） |
| `left_ee_channel` / `right_ee_channel` | `1` = CAN/CAN FD；`2` = COM1 RS485（默认） |
| `ctrl_mode` | 初始控制模式 |
| `max_joint_speed` / `max_joint_acceleration` | 关节限速 |
| `left_dyn_param` / `right_dyn_param` | 工具动力学（10 维） |
| `left_kine_param` / `right_kine_param` | 工具运动学（6 维） |

### 3.2 硬件接口

- **命令：** 关节 `position`
- **状态：** 关节 `position`、`velocity`、`effort`

## 4. 运行期 ROS2 参数

启动时 `hardware_parameters` 会同步声明为节点参数，可用 `ros2 param set` 动态修改（节点名因 launch 而异，用 `ros2 param list` 查找）。

| 参数 | 说明 |
|------|------|
| `ctrl_mode` | `POSITION` / `JOINT_IMPEDANCE` / `CART_IMPEDANCE` / `POWER_OFF` |
| `joint_k_gains` / `joint_d_gains` | 7 维关节阻抗 |
| `cart_k_gains` / `cart_d_gains` | 7 维笛卡尔阻抗 |
| `max_joint_speed` / `max_joint_acceleration` | 限速 |
| `left_brake_release` / `right_brake_release` | 抱闸/松闸（仅 `POWER_OFF`） |
| `left_tool_torque` / `left_tool_velocity` | 左末端归一化力矩/速度 |
| `right_tool_torque` / `right_tool_velocity` | 右末端 |

```bash
ros2 param set /<hardware_node> ctrl_mode JOINT_IMPEDANCE
ros2 param set /<hardware_node> joint_k_gains "[2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0]"
ros2 param set /<hardware_node> left_tool_torque 0.8
```

**抱闸 / 下使能：** 先 `deactivate` 控制器，再将硬件组件设为 `inactive`（触发 `POWER_OFF` 与下使能），方可 `left_brake_release` / `right_brake_release`。松闸期间仅允许 `POWER_OFF`；恢复前先抱闸，再 `active` 硬件并重新 `activate` 控制器。

```bash
ros2 control switch_controllers --deactivate ocs2_wbc_controller
ros2 control set_hardware_component_state <SystemName> inactive
ros2 param set /<hardware_node> left_brake_release true
# … 手动调整 …
ros2 param set /<hardware_node> left_brake_release false
ros2 control set_hardware_component_state <SystemName> active
ros2 param set /<hardware_node> ctrl_mode POSITION
ros2 control switch_controllers --activate ocs2_wbc_controller
```

数组参数长度须匹配（阻抗 7 维，`*_dyn_param` 10 维，`*_kine_param` 6 维）。

## 5. 编译

```bash
cd ~/ros2_ws/src/arms_ros2_control/hardwares/marvin_ros2_control
git submodule update --init external/TJ_FX_ROBOT_CONTRL_SDK

cd ~/ros2_ws
colcon build --packages-up-to marvin_ros2_control --symlink-install
```

`full` deb 构建会在 CI 中按目标架构重编 SDK（见父仓 `README.deb.md`）。

## 6. 示例

`examples/` 目录含夹爪/灵巧手 SDK 层调试程序（不经过 `ros2_control`）：

- `marvin_gripper_init_example.cpp`
- `marvin_hand_init_example.cpp`

## 7. 依赖

- Marvin SDK 子模块：`external/TJ_FX_ROBOT_CONTRL_SDK`
- `gripper_hardware_common`
- ROS2：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`

## 8. TODO

- [ ] **抽取 Inspire 协议公共层** — `InspireHandE2` / `InspireHandE2Canfd` 与 [modbus_ros2_control](https://github.com/fiveages-sim/modbus-ros2-control)、[can_ros2_control](https://github.com/fiveages-sim/can-ros2-control) 去重。
