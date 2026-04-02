from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

# 创建世界
world = World()

# 加载你的USD
usd_path = r"C:\Users\Pengguankai\Desktop\my_robot_scene\My_NovaCarter_FULL.usd"
world.scene.add_default_ground_plane()
world.scene.add(
    Articulation(
        prim_path="/World/nova_carter_empty",  # ⚠️ 这个要和你右边树一致
        name="robot"
    )
)

# 初始化
world.reset()

robot = world.scene.get_object("robot")

# 控制循环
while simulation_app.is_running():
    world.step(render=True)

    # 示例：给轮子速度（需要joint名字）
    robot.set_joint_velocities([1.0, 1.0])