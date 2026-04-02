# ====================== 必须最先 ======================
from isaacsim import SimulationApp
import json  # 新增：用于格式化保存数据

# ====================== 初始化数据记录结构 ======================
history_data = {
    "trajectory": [],  # 存储 [[x1, y1], [x2, y2], ...]
    "error": [],       # 存储 [[dist_err1, ang_err1], ...]
    "control": []      # 存储 [[v1, w1], ...]
}
simulation_app = SimulationApp({"headless": False})   # 改 True 就是无界面运行

# ====================== 这之后才能 import ======================
from isaacsim.core.utils.stage import open_stage      # ←←← 关键修正：用新路径
import omni.graph.core as og
import time
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np
from PIDcontroller import PositionControllerPID
# ====================== 加载你的 USD 场景 ======================
usd_path = r"C:\isaac-sim\a_mytasks\car_4.usd"   # 确认这个路径完全正确

print(f"正在加载场景: {usd_path}")
open_stage(usd_path=usd_path)          # 现在不会报 NameError 了
# 1. 创建控制器实例（可以自定义参数）
pid_controller = PositionControllerPID(
    kp_linear=0.6, ki_linear=0.08, kd_linear=0.15,
    kp_angular=2.5, ki_angular=0.15, kd_angular=0.25
)
# ====================== 创建 World 并重置 ======================
world = World(stage_units_in_meters=1.0)
world.reset()
waypoints = [
    [0, 0],
    [5, 0],
    [10, 5],
    [15, 5]
]
current_waypoint_idx = 0
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
    qw, qx, qy, qz=quat
    # print("在呢")
    return position, quat

print("✅ 场景加载完成，开始控制...")
initial_position, initial_quat = robot.get_world_pose()
print(f"📍 初始位姿已记录: pos={initial_position}, quat={initial_quat}")
# ====================== 获取 Differential Controller 节点 ======================
graph_path = "/Graphs/differential_controller"   # ← 如果还是报错，在 GUI 里右键节点 → Copy Path 替换
controller_node = og.Controller().node(graph_path + "/DifferentialController")
if not controller_node:
    print("❌ 找不到 Differential Controller 节点！请在 GUI 里确认完整路径")
else:
    print("✅ 找到控制器，开始设置速度...")
    try:
        controller_node.get_attribute("inputs:linearVelocity").set(0.0)   # 前进速度
        controller_node.get_attribute("inputs:angularVelocity").set(0.0)  # 直行
        get_chassis_position()
        print("✅ 速度设置成功")
    except Exception as e:
        print("❌ 设置失败:", e)


# ====================== 必须加 simulation loop，否则立刻退出 ======================
print("🚀 开始 simulation loop（按 Ctrl+C 退出）...")
# ====================== simulation loop（核心部分） ======================
print("🚀 开始 simulation loop（按 Ctrl+C 或关闭窗口退出）...")
goal_pos = [0,2]
try:
    frame = 0
    while simulation_app.is_running():
        current_pos,current_quat=get_chassis_position()
        if current_waypoint_idx >= len(waypoints):
            controller_node.get_attribute("inputs:linearVelocity").set(0.0)
            controller_node.get_attribute("inputs:angularVelocity").set(0.0)
            world.step(render=True)
            continue
        goal = waypoints[current_waypoint_idx]
        v, w = pid_controller.compute_controls(current_pos, current_quat, goal)

        # 2. 【核心修改】：记录当前时刻的数据
        # 记录轨迹 (x, y)
        history_data["trajectory"].append([float(current_pos[0]), float(current_pos[1])])
        # 记录跟踪误差 (distance_error, angle_error)
        history_data["error"].append(
            [float(pid_controller.current_dist_error), float(pid_controller.current_angle_error)])
        # 记录控制输出 (v, w)
        history_data["control"].append([float(v), float(w)])


        controller_node.get_attribute("inputs:linearVelocity").set(v)  # 前进速度
        controller_node.get_attribute("inputs:angularVelocity").set(w)  # 直行
        world.step(render=True)

        frame += 1
        if frame % 100 == 0:  # 每 100 帧（约 1 秒）打印一次位置
            current_pos, current_quat = get_chassis_position()
            print(current_pos)
            # print("在呢")
        # 判断是否到达当前点 → 切换下一个点
        dx = goal[0] - current_pos[0]
        dy = goal[1] - current_pos[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)
        if dist < 0.1:  # 到达阈值
            print(f"✅ 到达点 {current_waypoint_idx + 1}：{goal}")
            current_waypoint_idx += 1
            pid_controller.reset()  # 重置PID
        # time.sleep(0.01)
except KeyboardInterrupt:
    print("👋 用户主动中断，退出...")
except Exception as e:
    print("❌ 循环中发生错误:", e)
finally:
    # ====================== 保存数据到 TXT ======================
    file_name = "simulation_log.txt"
    try:
        with open(file_name, "w", encoding="utf-8") as f:
            # 使用 json.dump 让格式变得整齐漂亮
            json.dump(history_data, f, indent=4)
        print(f"📂 数据已成功保存至: {file_name}")
    except Exception as e:
        print(f"❌ 存档失败: {e}")
    simulation_app.close()