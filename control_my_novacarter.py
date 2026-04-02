from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers import DifferentialController

# ==================== 设置你的文件路径 ====================
usd_path = "C:/Users/Pengguankai/Desktop/my_robot_scene/My_NovaCarter.usd"   # ←←← 这里改成你的实际路径！

# ==================== 初始化世界 ====================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 加载你的机器人
add_reference_to_stage(usd_path=usd_path, prim_path="/World/MyNovaCarter")

# 使用正确的路径
robot = world.scene.add(
    Articulation(
        prim_path="/World/MyNovaCarter/nova_carter_empty",   # ← 已修正
        name="MyNovaCarter"
    )
)

# 差速控制器
controller = DifferentialController(
    name="diff_controller",
    wheel_radius=0.14,      # Nova Carter 轮子半径
    wheel_base=0.413        # 轮间距
)

world.reset()
print("✅ Nova Carter 加载成功！开始速度控制...")

# ==================== 主循环 ====================
step = 0
while simulation_app.is_running():
    world.step(render=True)
    step += 1

    # 运动示例（可自行修改）
    if step < 200:
        linear, angular = 0.4, 0.0     # 前进
    elif step < 400:
        linear, angular = 0.0, 0.8     # 左转
    elif step < 600:
        linear, angular = 0.3, -0.6    # 右弧线
    else:
        linear, angular = 0.0, 0.0     # 停止

    wheel_actions = controller.forward([linear, angular])
    robot.apply_wheel_actions(wheel_actions)

    if step % 100 == 0:
        print(f"Step {step:4d} | 线速度: {linear:.2f} m/s | 角速度: {angular:.2f} rad/s")

simulation_app.close()