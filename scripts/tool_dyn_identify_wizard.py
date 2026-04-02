#!/usr/bin/env python3
import sys
import os
import time
import logging


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))


def _has_sdk_python(root: str) -> bool:
    return os.path.isdir(os.path.join(root, "SDK_PYTHON"))


def _resolve_sdk_root() -> str:
    """定位 vendored TJ_FX_ROBOT_CONTRL_SDK 根目录（兼容源码运行/安装后运行）。"""
    # 1) 源码运行：<pkg>/scripts/xxx.py -> <pkg>/external/TJ_FX_ROBOT_CONTRL_SDK
    pkg_dir = os.path.dirname(CURRENT_DIR)
    cand1 = os.path.join(pkg_dir, "external", "TJ_FX_ROBOT_CONTRL_SDK")
    if _has_sdk_python(cand1):
        return cand1

    # 2) 安装后（colcon）：可执行文件在 <prefix>/lib/<pkg>/tool_dyn_identify_wizard
    #    故 prefix = dirname(dirname(CURRENT_DIR))，share 在 <prefix>/share/<pkg>/...
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        share_dir = get_package_share_directory("marvin_ros2_control")
        cand_ament = os.path.join(share_dir, "TJ_FX_ROBOT_CONTRL_SDK")
        if _has_sdk_python(cand_ament):
            return cand_ament
    except Exception:
        pass

    # 3) 不依赖 ament 索引时，从安装路径推断 prefix（勿用三级 dirname，否则会指到 workspace 根目录）
    prefix = os.path.dirname(os.path.dirname(CURRENT_DIR))
    cand2 = os.path.join(prefix, "share", "marvin_ros2_control", "TJ_FX_ROBOT_CONTRL_SDK")
    if _has_sdk_python(cand2):
        return cand2

    raise RuntimeError(
        "无法定位 TJ_FX_ROBOT_CONTRL_SDK（需要包含 SDK_PYTHON）。"
        "请 source install/setup.bash 后重试，并确认已重新 colcon build marvin_ros2_control（子模块含 SDK_PYTHON）。"
    )


# Use SDK python bindings from marvin_ros2_control's vendored SDK.
SDK_ROOT = _resolve_sdk_root()
if SDK_ROOT not in sys.path:
    sys.path.insert(0, SDK_ROOT)

from SDK_PYTHON.fx_kine import Marvin_Kine  # noqa: E402
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS  # noqa: E402


def _arm_index(arm: str) -> int:
    return 0 if arm == "A" else 1


def _check_joints_reached(fb_joints, target_joints, tol_deg: float = 0.5) -> bool:
    try:
        return all(abs(float(a) - float(b)) <= tol_deg for a, b in zip(fb_joints, target_joints))
    except Exception:
        return False


def _max_abs_joint_error_deg(fb_joints, target_joints) -> float:
    try:
        return max(abs(float(a) - float(b)) for a, b in zip(fb_joints, target_joints))
    except Exception:
        return float("inf")


def _resolve_ccs_paths(arm: str):
    """根据选择的手臂（A 左 / B 右）解析 CCS 示例路径。"""
    # PVT/配置文件从 SDK 安装目录读取；采集数据写入当前工作目录（可写）。
    base_template = os.path.join(SDK_ROOT, "DEMO_PYTHON", "LoadData_ccs_right", "LoadData")
    side = "Left" if arm == "A" else "Right"
    pvt_file = os.path.join(base_template, "IdenTraj", f"LoadIdenTraj_MarvinCCS_{side}.fmv")

    # 工作目录：包含辨识所需的结构（CfgFile/IdenTraj/LoadData.csv/NoLoadData.csv）
    work_base = os.path.join(os.getcwd(), "tool_dyn_identify_data", "ccs", arm)
    os.makedirs(work_base, exist_ok=True)
    os.makedirs(os.path.join(work_base, "CfgFile"), exist_ok=True)
    os.makedirs(os.path.join(work_base, "IdenTraj"), exist_ok=True)

    # 复制模板里的配置/轨迹文件到工作目录（避免安装目录不可写）
    def _copy_if_missing(src: str, dst: str):
        if os.path.isfile(src) and (not os.path.isfile(dst)):
            with open(src, "rb") as fsrc, open(dst, "wb") as fdst:
                fdst.write(fsrc.read())

    # CfgFile 里的内容用于辨识（按 demo 目录结构复制）
    cfg_dir = os.path.join(base_template, "CfgFile")
    if os.path.isdir(cfg_dir):
        for name in os.listdir(cfg_dir):
            _copy_if_missing(os.path.join(cfg_dir, name), os.path.join(work_base, "CfgFile", name))

    # IdenTraj 文件也复制一份（用于完整性/可迁移；采集阶段仍读取原始 pvt_file）
    traj_dst = os.path.join(work_base, "IdenTraj", os.path.basename(pvt_file))
    _copy_if_missing(pvt_file, traj_dst)

    # identify_tool_dyn expects fixed filenames inside ipath.
    load_csv = os.path.join(work_base, "LoadData.csv")
    noload_csv = os.path.join(work_base, "NoLoadData.csv")
    return work_base, pvt_file, load_csv, noload_csv


def _go_to_joint_zero(
    robot: Marvin_Robot,
    dcss: DCSS,
    arm: str,
    logger: logging.Logger,
    vel_ratio: int = 30,
    acc_ratio: int = 30,
    timeout_s: float = 60.0,
) -> None:
    """将指定手臂切到位置模式，并运动到关节零位 (0,0,0,0,0,0,0)（单位：度），等待到位。

    注意：某些情况下 SDK 的 low_speed_flag 可能在未到位时就为 1（例如未使能/命令未生效/通信异常）。
    因此这里以“关节误差满足阈值”为到位的唯一判据，low_speed_flag 仅作为诊断信息。
    """
    target = [0.0] * 7
    idx = _arm_index(arm)

    robot.clear_set()
    robot.set_vel_acc(arm=arm, velRatio=vel_ratio, AccRatio=acc_ratio)
    robot.set_state(arm=arm, state=1)  # POSITION
    robot.send_cmd()
    time.sleep(0.8)

    logger.info(f"[步骤] 手臂 {arm} 运动到关节零位 {target}（单位：度）")
    t0 = time.time()
    last_log_t = 0.0
    last_clear_t = 0.0
    clear_attempts = 0
    last_pos_state_req_t = 0.0
    pos_state_req_attempts = 0
    while True:
        # IMPORTANT:
        # The SDK uses an internal send buffer which accumulates "set" operations until send_cmd().
        # If we don't clear it, the buffer can overflow and later commands may become ineffective
        # while still printing local logs. Therefore, we clear_set() every cycle to send a single,
        # fresh command frame.
        robot.clear_set()
        robot.set_joint_cmd_pose(arm=arm, joints=target)
        robot.send_cmd()
        time.sleep(0.02)

        sub_data = robot.subscribe(dcss)
        fb = sub_data["outputs"][idx]["fb_joint_pos"]
        low_spd = sub_data["outputs"][idx]["low_speed_flag"][0]
        cur_state = sub_data["states"][idx].get("cur_state", None)
        cmd_state = sub_data["states"][idx].get("cmd_state", None)
        err_code = sub_data["states"][idx].get("err_code", None)
        max_err = _max_abs_joint_error_deg(fb, target)

        # If controller reports arm error, try clearing it periodically.
        # Especially important for ARM_ERR_Emcy=13: after releasing physical E-stop,
        # cabinet often stays in error state until clear-error succeeds.
        now = time.time()
        if err_code not in (None, 0):
            if now - last_clear_t > 1.0:
                last_clear_t = now
                clear_attempts += 1
                try:
                    robot.clear_error(arm)
                except Exception:
                    pass
                # Try to print servo fault details (hex + decoded) for faster diagnosis.
                try:
                    servo_faults = robot.get_servo_error_code(arm, lang="CN")
                    logger.warning(f"[步骤] arm={arm} err_code={err_code}，已触发清错(第{clear_attempts}次)。伺服故障码: {servo_faults}")
                except Exception:
                    logger.warning(f"[步骤] arm={arm} err_code={err_code}，已触发清错(第{clear_attempts}次)。")

        # If error is cleared but arm is still in IDLE (0), re-request POSITION mode periodically.
        # After E-stop clear, cabinet often ends up in IDLE (servo down) and needs a fresh state request.
        if err_code in (None, 0) and cur_state == 0:
            if now - last_pos_state_req_t > 1.0:
                last_pos_state_req_t = now
                pos_state_req_attempts += 1
                try:
                    robot.clear_set()
                    robot.set_vel_acc(arm=arm, velRatio=vel_ratio, AccRatio=acc_ratio)
                    robot.set_state(arm=arm, state=1)  # POSITION
                    robot.send_cmd()
                    logger.warning(
                        f"[步骤] arm={arm} 当前处于下伺服(cur_state=0)，已重新请求进入位置模式(第{pos_state_req_attempts}次)。"
                    )
                except Exception as e:
                    logger.warning(f"[步骤] arm={arm} 重新请求位置模式时异常: {e}")

        # Periodic diagnostics (avoid spamming)
        if now - last_log_t > 0.5:
            last_log_t = now
            logger.info(
                f"[步骤] 零位等待中 arm={arm} cur_state={cur_state} cmd_state={cmd_state} err_code={err_code} "
                f"low_speed_flag={low_spd} max_err_deg={max_err:.3f} fb_joint_pos={fb}"
            )

        if _check_joints_reached(fb, target, tol_deg=0.05):
            logger.info(f"[步骤] 手臂 {arm} 已到达关节零位。max_err_deg={max_err:.3f} fb_joint_pos={fb}")
            return
        if (time.time() - t0) > timeout_s:
            logger.error(
                f"[步骤] 等待手臂 {arm} 到达零位超时。cur_state={cur_state} cmd_state={cmd_state} err_code={err_code} "
                f"low_speed_flag={low_spd} max_err_deg={max_err:.3f} fb_joint_pos={fb}"
            )
            raise TimeoutError("等待手臂到达关节零位超时")


def collect_identy_data(
    robot: Marvin_Robot,
    dcss: DCSS,
    logger: logging.Logger,
    arm: str,
    pvt_file: str,
    pvt_id: int,
    save_path: str,
):
    idx = _arm_index(arm)

    robot.clear_set()
    robot.set_state(arm=arm, state=2)  # PVT
    robot.send_cmd()
    time.sleep(0.5)

    sub_data = robot.subscribe(dcss)
    logger.info(f'当前状态 cur_state={sub_data["states"][idx]["cur_state"]}')
    logger.info(f'指令状态 cmd_state={sub_data["states"][idx]["cmd_state"]}')
    logger.info(f'错误码 err_code={sub_data["states"][idx]["err_code"]}')

    robot.send_pvt_file(arm, pvt_file, pvt_id)
    logger.info(f"设置 PVT 轨迹文件: {pvt_file}, PVT id: {pvt_id}")
    time.sleep(0.5)

    cols = 15
    if arm == "A":
        target_ids = [
            0,
            1,
            2,
            3,
            4,
            5,
            6,
            50,
            51,
            52,
            53,
            54,
            55,
            56,
            76,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    elif arm == "B":
        target_ids = [
            100,
            101,
            102,
            103,
            104,
            105,
            106,
            150,
            151,
            152,
            153,
            154,
            155,
            156,
            176,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    else:
        raise ValueError("arm must be A or B")

    rows = 1000000
    robot.clear_set()
    robot.collect_data(targetNum=cols, targetID=target_ids, recordNum=rows)
    robot.send_cmd()
    logger.info("开始采集辨识数据")
    time.sleep(0.5)

    robot.clear_set()
    robot.set_pvt_id(arm, pvt_id)
    robot.send_cmd()
    logger.info("开始运行 PVT 轨迹")

    time.sleep(60)

    robot.clear_set()
    robot.stop_collect_data()
    robot.send_cmd()
    time.sleep(0.5)

    robot.save_collected_data_to_path(save_path)
    time.sleep(1)

    processed_data = []
    with open(save_path, "r") as file:
        lines = file.readlines()
    lines = lines[1:]
    for line in lines:
        parts = line.strip().split("$")
        numbers = []
        for part in parts:
            if part:
                num_str = part.split()[-1]
                numbers.append(num_str)
        if len(numbers) >= 2:
            numbers = numbers[2:]
        processed_data.append(numbers)
    time.sleep(0.5)
    os.remove(save_path)
    time.sleep(0.5)
    with open(save_path, "w") as out_file:
        for row in processed_data:
            out_file.write(",".join(row) + "\n")

    logger.info(f"数据已保存为: {save_path}")


def _prompt(msg: str) -> str:
    return input(msg).strip()


def run_wizard():
    log_path = os.path.join(os.getcwd(), "tool_dyn_identify.log")
    logger = logging.getLogger("tool_dyn_identify")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    sh = logging.StreamHandler()
    sh.setFormatter(fmt)
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setFormatter(fmt)
    logger.addHandler(sh)
    logger.addHandler(fh)

    print("=== 机械臂负载辨识向导（工具动力学参数辨识 / CCS）===")
    arm = _prompt("请选择手臂 [A=左臂 / B=右臂]（默认 B）：").upper() or "B"
    if arm not in ("A", "B"):
        raise ValueError("手臂只能是 A 或 B")
    ip = _prompt("请输入机器人控制器 IP（默认 192.168.1.190）：") or "192.168.1.190"

    base, pvt_file, load_csv, noload_csv = _resolve_ccs_paths(arm)
    logger.info(f"[配置] arm={arm} ip={ip}")
    logger.info(f"[配置] pvt_file={pvt_file}")
    logger.info(f"[配置] ipath(base)={base}")
    logger.info(f"[配置] noload_csv={noload_csv}")
    logger.info(f"[配置] load_csv={load_csv}")

    dcss = DCSS()
    robot = Marvin_Robot()

    logger.info("[步骤] 连接机器人")
    init = robot.connect(ip)
    if init == 0:
        logger.error("连接机器人失败：端口可能被占用")
        raise RuntimeError("连接失败")

    logger.info("[步骤] 检查并清除错误")
    robot.check_error_and_clear(dcss)

    logger.info("[步骤] 验证 UDP 订阅通道（frame_serial 是否刷新）")
    motion_tag = 0
    frame_update = None
    for _ in range(8):
        sub_data = robot.subscribe(dcss)
        fs = sub_data["outputs"][0]["frame_serial"]
        print(f"连接帧 frame_serial: {fs}")
        if fs != 0 and fs != frame_update:
            motion_tag += 1
            frame_update = fs
        time.sleep(0.01)
    if motion_tag <= 0:
        logger.error("连接失败：frame_serial 未刷新（可能被防火墙/网络阻断）")
        raise RuntimeError("订阅数据未刷新")
    logger.info("[步骤] 连接成功（frame_serial 正常刷新）")

    robot.log_switch("1")
    robot.local_log_switch("1")

    _prompt(
        f"\n[确认] 即将让手臂 {arm} 运动到关节零位 (0,0,0,0,0,0,0)（单位：度）。\n"
        f"确认无误请按回车继续（Ctrl+C 退出）。"
    )
    _go_to_joint_zero(robot, dcss, arm, logger)

    _prompt(
        "\n[确认] 请确认当前为【空载】状态（无工具/无负载）。\n"
        "确认无误请按回车开始【空载】采集。"
    )
    collect_identy_data(robot, dcss, logger, arm=arm, pvt_file=pvt_file, pvt_id=3, save_path=noload_csv)
    logger.info("[步骤] 空载采集完成")

    _prompt(
        "\n[确认] 请现在安装工具/负载，切换到【带载】状态。\n"
        "准备好后请按回车开始【带载】采集。"
    )
    collect_identy_data(robot, dcss, logger, arm=arm, pvt_file=pvt_file, pvt_id=3, save_path=load_csv)
    logger.info("[步骤] 带载采集完成")

    logger.info("[步骤] 开始工具动力学参数辨识")
    kk = Marvin_Kine()
    tool_dynamic_parameters = kk.identify_tool_dyn(robot_type=1, ipath=base)
    logger.info(f"[结果] 工具动力学参数 (m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz): {tool_dynamic_parameters}")
    print("\n=== 辨识结果 ===")
    print(tool_dynamic_parameters)
    print(f"\n日志已保存到: {log_path}")

    _prompt("\n按回车释放机器人连接。")
    robot.release_robot()


if __name__ == "__main__":
    run_wizard()

