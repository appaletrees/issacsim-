from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World

world = World()

usd_path = r"C:\isaac-sim\a_mytasks\car_4.usd"

add_reference_to_stage(usd_path, "/World/Robot")

world.reset()

while simulation_app.is_running():
    world.step(render=True)