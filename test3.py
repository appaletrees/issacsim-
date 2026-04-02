# ====================== 必须最先 ======================
from isaacsim import SimulationApp
import json  # 新增：用于格式化保存数据

# ====================== 初始化数据记录结构 ======================
history_data = {
    "trajectory": [],  # 存储 [[x1, y1], [x2, y2], ...]
    "error": [],       # 存储 [[dist_err1, ang_err1], ...]
    "control": []      # 存储 [[v1, w1], ...]
}
simulation_app = SimulationApp({"headless": True})   # 改 True 就是无界面运行

# ====================== 这之后才能 import ======================
from isaacsim.core.utils.stage import open_stage      # ←←← 关键修正：用新路径
import omni.graph.core as og
import time
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np
from PIDcontroller import PositionControllerPID
from analyze_results import *
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


def reset_robot():
    """将机器人完全恢复到初始状态（位置 + 姿态 + 速度清零）"""
    robot.set_world_pose(initial_position, initial_quat)

    # 清零线速度和角速度（防止残留运动）
    try:
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
    except Exception as e:
        print(f"   速度清零警告（可忽略）: {e}")

    # 强制控制器速度为 0
    if controller_node:
        controller_node.get_attribute("inputs:linearVelocity").set(0.0)
        controller_node.get_attribute("inputs:angularVelocity").set(0.0)

    world.step(render=True)  # 让物理引擎稳定一帧
    print("🔄 机器人已复位到初始状态（可进行下一次 PID 参数测试）")

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


def run_simulation_trial(pid_params, trial_name="trial"):
    """
    【外部 PID 自动整定专用函数】
    pid_params: list [kp_linear, ki_linear, kd_linear, kp_angular, ki_angular, kd_angular]

    每次调用会：
    1. 使用新 PID 参数创建控制器
    2. 自动复位机器人到初始状态
    3. 运行一次完整路径跟踪
    4. 保存本次 trial 的 trajectory / error / control 数据
    5. 返回 cost（供优化器最小化）

    你可以直接把这个函数传给任何外部优化器（PSO、GA、Bayesian、Optuna、scipy.optimize 等）
    """
    kp_linear, ki_linear, kd_linear, kp_angular, ki_angular, kd_angular = pid_params

    # 每次 trial 都新建 PID 控制器（完全独立，不受上一次影响）
    pid_controller = PositionControllerPID(
        kp_linear=kp_linear, ki_linear=ki_linear, kd_linear=kd_linear,
        kp_angular=kp_angular, ki_angular=ki_angular, kd_angular=kd_angular
    )

    # 复位机器人（这就是你想要的“下一个 PID 参数时回复初始状态”）
    reset_robot()

    current_waypoint_idx = 0
    history_data = {  # 每个 trial 独立记录数据
        "trajectory": [],
        "error": [],
        "control": []
    }

    frame = 0
    max_frames = 10000  # 防止坏参数导致无限循环，根据你的场景适当调整
    completed = False

    print(f"🚀 开始 Trial: {trial_name} | PID参数: {pid_params}")

    while simulation_app.is_running() and frame < max_frames:
        if current_waypoint_idx >= len(waypoints):
            completed = True
            # 到达终点，停止运动
            if controller_node:
                controller_node.get_attribute("inputs:linearVelocity").set(0.0)
                controller_node.get_attribute("inputs:angularVelocity").set(0.0)
            world.step(render=True)
            break

        current_pos, current_quat = get_chassis_position()
        goal = waypoints[current_waypoint_idx]

        # 计算控制量
        v, w = pid_controller.compute_controls(current_pos, current_quat, goal)

        # 记录当前时刻数据
        history_data["trajectory"].append([float(current_pos[0]), float(current_pos[1])])
        history_data["error"].append(
            [float(pid_controller.current_dist_error), float(pid_controller.current_angle_error)])
        history_data["control"].append([float(v), float(w)])

        # 下发速度
        if controller_node:
            controller_node.get_attribute("inputs:linearVelocity").set(v)
            controller_node.get_attribute("inputs:angularVelocity").set(w)

        world.step(render=True)
        frame += 1

        # 检查是否到达当前 waypoint
        dx = goal[0] - current_pos[0]
        dy = goal[1] - current_pos[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)
        if dist < 0.1:
            print(f"✅ 到达点 {current_waypoint_idx + 1}：{goal}")
            current_waypoint_idx += 1
            pid_controller.reset()  # 重置 PID 积分/微分项

        if frame % 100 == 0:
            print(f"  [Frame {frame}] 位置: {current_pos[:2]} 目标: {goal}")

    # ==================== 计算 cost（供外部优化器使用） ====================
    # 你可以在这里自定义评价指标，例如：
    #   - 完成时间（frame 越小越好）
    #   - 误差积分（ISE / IAE）
    #   - 是否成功 + 超调惩罚
    cost2 = analyze_simulation(history_data, plot=False)  # cost2 是 metrics 字典
    if completed:
        # ←←← 这里改成用 ISE 作为 cost（推荐，更适合 PID 优化）
        cost = cost2.get("ISE (平方误差积分)", frame) if isinstance(cost2, dict) else frame
        print(f"✅ Trial 完成 | ISE={cost:.2f}")
    else:
        cost = max_frames * 10
        print("❌ Trial 超时")

    # 保存本次 trial 数据（方便后续分析轨迹）
    file_name = f"simulation_log_{trial_name}.txt"
    try:
        with open(file_name, "w", encoding="utf-8") as f:
            json.dump(history_data, f, indent=4)
        print(f"📂 Trial 数据已保存: {file_name}")
    except Exception as e:
        print(f"❌ 保存失败: {e}")

    return cost,cost2


# ====================== 示例运行（保持你原来单次运行的功能） ======================
print("\n🚀 代码已封装完成！下面演示一次默认参数运行（兼容原脚本）")
print("   你现在可以直接把 run_simulation_trial 传给任何外部优化器进行自动整定\n")

try:
    print("\n🚀 启动 PSO 自动 PID 参数整定...")

    bounds = [
        (0.1, 2.0), (0.0, 0.5), (0.0, 0.5),  # kp_linear, ki_linear, kd_linear
        (0.5, 6.0), (0.0, 0.5), (0.0, 1.0)  # kp_angular, ki_angular, kd_angular
    ]


    # 目标函数（PSO 会反复调用）
    def objective(params):
        trial_name = f"pso_{int(time.time())}"
        cost, cost2 = run_simulation_trial(params, trial_name)
        return cost  # ← PSO 最小化的就是这个 cost


    # 导入 PSO 类并开始优化
    from pso_pid_optimizer import PSO

    pso = PSO(n_particles=15, max_iter=25, verbose=True)
    best_params, best_cost = pso.optimize(objective, bounds)
    pso.plot_convergence(save_path="./pso_convergence.png")
    print("\n" + "=" * 60)
    print("🎉 PSO 全局最优结果")
    print("=" * 60)
    print(f"最优 PID 参数: {np.round(best_params, 4)}")
    print(f"最优 cost 值:   {best_cost:.4f}")
    print("顺序 → [kp_linear, ki_linear, kd_linear, kp_angular, ki_angular, kd_angular]")

except KeyboardInterrupt:
    print("👋 用户主动中断，退出...")
except Exception as e:
    print("❌ 运行中发生错误:", e)
finally:
    # ====================== 程序结束时关闭 ======================
    simulation_app.close()
    print("✅ 仿真已安全关闭")