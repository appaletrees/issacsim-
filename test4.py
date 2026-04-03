# ====================== 必须最先 ======================
from isaacsim import SimulationApp
import json  # 新增：用于格式化保存数据

# ====================== 初始化数据记录结构 ======================
history_data = {
    "trajectory": [],  # 存储 [[x1, y1], [x2, y2], ...]
    "error": [],       # 存储 [[dist_err1, ang_err1], ...]
    "control": []      # 存储 [[v_final1, w_final1], ...] ← 最终指令（规划+修正）
}
simulation_app = SimulationApp({"headless": False})   # 改 True 就是无界面运行

# ====================== 这之后才能 import ======================
from isaacsim.core.utils.stage import open_stage
import omni.graph.core as og
import time
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np
from PIDcontroller import PositionControllerPID

# ====================== 加载你的 USD 场景 ======================
usd_path = r"C:\isaac-sim\a_mytasks\car_4.usd"

print(f"正在加载场景: {usd_path}")
open_stage(usd_path=usd_path)

# ====================== 【核心】轨迹规划：生成带速度信息的参考轨迹 ======================
# 您的思路完全正确！这是标准的「前馈 + 反馈」控制结构：
# 1. 轨迹规划 → 一系列离散参考点 + 规划速度（v_planned, w_planned）
# 2. PID → 只计算当前位置与当前参考点的误差，输出速度修正（v_corr, w_corr）
# 3. 最终指令 = 规划速度 + PID修正
# 这样机器人能以期望速度精确跟踪路径，无稳态滞后
waypoints = np.array([
    [0, 0],
    [5, 0],
    [10, 5],
    [15, 5]
])
segments = np.diff(waypoints, axis=0)
segment_lengths = np.linalg.norm(segments, axis=1)
total_length = np.sum(segment_lengths)
cum_lengths = np.cumsum(np.concatenate(([0.], segment_lengths)))

desired_speed = 0.5          # 参考轨迹的前进速度（m/s），可自行调整
dt = 1.0 / 60.0              # 仿真步长（Isaac Sim 默认 60Hz）
current_s = 0.0              # 弧长参数（随时间自动前进）

print(f"✅ 轨迹规划完成：总长度 {total_length:.2f}m，参考速度 {desired_speed:.2f} m/s")
print("   PID 现在仅输出修正量，最终速度 = 规划速度 + PID修正")

# 1. 创建控制器实例（PID 参数保持不变，仅作为修正器使用）
pid_controller = PositionControllerPID(
    kp_linear=0.6, ki_linear=0.08, kd_linear=0.15,
    kp_angular=2.5, ki_angular=0.15, kd_angular=0.25
)

# ====================== 创建 World 并重置 ======================
world = World(stage_units_in_meters=1.0)
world.reset()

# ====================== 获取机器人 ======================
robot_path = "/World/jetbot"
robot = world.scene.get_object("jetbot")

if robot is None:
    robot = Articulation(prim_path=robot_path, name="jetbot")
    world.scene.add(robot)

def get_chassis_position():
    position, quat = robot.get_world_pose()
    x, y, z = position
    x, y = y, x
    qw, qx, qy, qz = quat
    return position, quat

print("✅ 场景加载完成，开始控制...")
initial_position, initial_quat = robot.get_world_pose()
print(f"📍 初始位姿已记录: pos={initial_position}, quat={initial_quat}")

# ====================== 获取 Differential Controller 节点 ======================
graph_path = "/Graphs/differential_controller"
controller_node = og.Controller().node(graph_path + "/DifferentialController")
if not controller_node:
    print("❌ 找不到 Differential Controller 节点！请在 GUI 里确认完整路径")
else:
    print("✅ 找到控制器，开始设置速度...")
    try:
        controller_node.get_attribute("inputs:linearVelocity").set(0.0)
        controller_node.get_attribute("inputs:angularVelocity").set(0.0)
        print("✅ 速度设置成功")
    except Exception as e:
        print("❌ 设置失败:", e)

# ====================== simulation loop ======================
print("🚀 开始 simulation loop（按 Ctrl+C 或关闭窗口退出）...")
try:
    frame = 0
    while simulation_app.is_running():
        current_pos, current_quat = get_chassis_position()

        # ====================== 【轨迹规划核心】更新参考点（带速度信息） ======================
        current_s = min(current_s + desired_speed * dt, total_length)

        if current_s >= total_length:
            ref_goal = waypoints[-1].tolist()
            v_planned = 0.0
            w_planned = 0.0
        else:
            # 找到当前所在段，计算参考位置
            segment_idx = np.searchsorted(cum_lengths, current_s) - 1
            s_in_seg = current_s - cum_lengths[segment_idx]
            fraction = s_in_seg / segment_lengths[segment_idx]
            ref_pos = waypoints[segment_idx] + fraction * segments[segment_idx]
            ref_goal = ref_pos.tolist()
            v_planned = desired_speed
            w_planned = 0.0   # 直线段角速度规划为 0（转弯由 PID 修正完成）

        # ====================== PID 只输出修正量 ======================
        v_corr, w_corr = pid_controller.compute_controls(current_pos, current_quat, ref_goal)

        # ====================== 最终控制指令 = 规划速度 + PID 修正 ======================
        v = v_planned + v_corr
        w = w_planned + w_corr

        # 终点强制停止（防止 PID 持续修正）
        if current_s >= total_length:
            dx = ref_goal[0] - current_pos[0]
            dy = ref_goal[1] - current_pos[1]
            dist = np.sqrt(dx ** 2 + dy ** 2)
            if dist < 0.1:
                v = 0.0
                w = 0.0

        # 2. 记录数据（与您原来结构完全一致，只是 control 现在是最终指令）
        history_data["trajectory"].append([float(current_pos[0]), float(current_pos[1])])
        history_data["error"].append(
            [float(pid_controller.current_dist_error), float(pid_controller.current_angle_error)])
        history_data["control"].append([float(v), float(w)])

        # 设置速度给 Differential Controller
        controller_node.get_attribute("inputs:linearVelocity").set(v)
        controller_node.get_attribute("inputs:angularVelocity").set(w)

        world.step(render=True)

        frame += 1
        if frame % 100 == 0:
            current_pos, current_quat = get_chassis_position()
            print(f"Frame {frame} | 当前位置: {current_pos[:2]} | 参考点: {ref_goal} | 指令: v={v:.3f}, w={w:.3f}")

except KeyboardInterrupt:
    print("👋 用户主动中断，退出...")
except Exception as e:
    print("❌ 循环中发生错误:", e)
finally:
    # ====================== 保存数据到 TXT ======================
    file_name = "simulation_log.txt"
    try:
        with open(file_name, "w", encoding="utf-8") as f:
            json.dump(history_data, f, indent=4)
        print(f"📂 数据已成功保存至: {file_name}（包含 trajectory / error / control）")
    except Exception as e:
        print(f"❌ 存档失败: {e}")
    simulation_app.close()