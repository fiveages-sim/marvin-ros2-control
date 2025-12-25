# Marvin ROS2 Control

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
* state:
    * joint effort
    * joint position
    * joint velocity

## 2. Build

* Submodule
```bash
git submodule update --init
```

* Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to marvin_ros2_control --symlink-install
```

## 3. 机制说明（参数加载/动态修改/夹爪）

本包的 `MarvinHardware`（`src/marvin-ros2-control/src/marvin_hardware.cpp`）采用“**hardware 参数作为初始配置来源 + 同步声明为 ROS2 参数以支持运行期动态修改**”的机制。

### 3.1 参数加载优先级与规则

- **初始值来源**：优先从 `ros2_control` 的 `hardware_parameters`（URDF/Xacro 或 YAML 中的 `hardware_parameters` 字段）读取。
- **同步为 ROS2 参数**：启动时会把这些初始值声明为节点参数（`declare_node_parameters()`），方便后续使用 `ros2 param set` 动态修改。
- **不重复/不覆盖规则**：
  - **如果 ROS2 参数已存在且类型正确且（对数组参数）长度也正确**：尊重现有值（通常来自 launch/YAML 覆盖），不会被 `hardware_parameters` 覆盖。
  - **如果参数已存在但类型不对，或数组长度不符合预期**：会 `undeclare` 后重新以正确类型/长度 `declare`，避免运行时类型冲突或越界风险。

### 3.2 支持动态修改的 ROS2 参数

以下参数在运行时可通过 `ros2 param set` 修改，并由参数回调应用到硬件：

- **控制模式**
  - `ctrl_mode`：`POSITION` / `JOINT_IMPEDANCE` / `CART_IMPEDANCE`
- **阻抗与拖动相关**
  - `joint_k_gains`（double array，长度 7）
  - `joint_d_gains`（double array，长度 7）
  - `cart_k_gains`（double array，长度 7）
  - `cart_d_gains`（double array，长度 7）
  - `cart_type`（int）
  - `drag_mode`（int）
- **限速**
  - `max_joint_speed`（double）
  - `max_joint_acceleration`（double）
- **工具/夹爪负载参数**
  - `left_kine_param`（double array，长度 6）
  - `left_dyn_param`（double array，长度 10）
  - `right_kine_param`（double array，长度 6）
  - `right_dyn_param`（double array，长度 10）

示例（动态切换控制模式/阻抗参数）：

```bash
# 切换控制模式
ros2 param set /<controller_manager_node_name> ctrl_mode JOINT_IMPEDANCE

# 设置关节阻抗 K/D（7 维）
ros2 param set /<controller_manager_node_name> joint_k_gains "[2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0]"
ros2 param set /<controller_manager_node_name> joint_d_gains "[0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4]"
```

> 注：节点名取决于你的 `ros2_control` 启动方式，常见为 controller_manager 所在节点。可先用 `ros2 param list` 查找包含上述参数的节点名。

### 3.3 hardware_parameters 中常用参数（初始配置）

以下参数一般在 `hardware_parameters` 中配置，启动时会作为初始值并同步声明为 ROS2 参数：

- `arm_type`：`LEFT` / `RIGHT` / `DUAL`
- `device_ip`：设备 IP（string）
- `device_port`：设备端口（int）
- `gripper_type`：夹爪类型（见下节）

### 3.4 夹爪类型（gripper_type）选项与映射

`gripper_type` 会先做标准化（转大写、去空格）。当前支持的值如下（大小写不敏感）：

- **JD / RG75**：`RG75`、`JDGRIPPER`
- **Changingtek 90C**：`CHANGINGTEK90C`、`AG2F90`、`AG2F90C`、`AG2F90_C`
- **Changingtek 90D**：`CHANGINGTEK90D`、`AG2F90D`、`AG2F90_D`

双臂时夹爪通道映射规则：

- 左夹爪：A 通道（索引 0）
- 右夹爪：B 通道（索引 1）

### 3.4.1 m6_ccs 中的“动态解析 gripper_type”机制

在 `m6_ccs_description` 中，夹爪类型并不是手写死在某一个 URDF 里，而是通过 **xacro 参数 `type` 动态解析**得到：

- 解析逻辑位于：`src/robot-descriptions-tianji/m6_ccs_description/xacro/robot.xacro`
  - 它会根据 `type`（例如 `dual_ag2f90_c` / `dual_ag2f90_d` / `dual_rg75` / `rg75` / `usb-90c` 等）设置：
    - `selected_arm_config`（left/right/dual）
    - `selected_gripper_type`（none/rg75/ag2f90_c/ag2f90_d 等）
- ros2_control 硬件参数注入位于：`src/robot-descriptions-tianji/m6_ccs_description/xacro/ros2_control/robot.xacro`
  - 在 `ros2_control_hardware_type=real` 时，会把解析结果作为 `hardware_parameters` 传给 `MarvinHardware`：
    - `<param name="arm_type">${selected_arm_config}</param>`
    - `<param name="gripper_type">${selected_gripper_type}</param>`
  - 同时它会按 `selected_gripper_type` 动态 `include` 对应夹爪的 ros2_control 接口描述（例如 Changingtek AG2F90-C 或 Jodell RG75）。

因此在 m6_ccs 场景下，**你只需要在生成机器人描述时选对 `type`**，`gripper_type` 会自动跟随并传入 `marvin_ros2_control/MarvinHardware`，再由本包完成：

- `hardware_parameters` 初始值加载
- 声明为 ROS2 参数
- 运行时 `ros2 param set` 动态修改

### 3.5 常见注意事项

- **数组参数长度必须匹配**：例如 `joint_k_gains/joint_d_gains/cart_k_gains/cart_d_gains` 必须为 7；工具参数必须为 6/10。长度不匹配会被重置为默认值。
- **动态修改的时机**：某些参数修改会触发重新下发配置/模式切换，建议在控制器稳定运行时修改，并观察日志确认生效。
