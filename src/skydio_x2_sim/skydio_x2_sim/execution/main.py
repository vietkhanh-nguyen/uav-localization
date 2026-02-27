import os
import argparse
import rclpy
from ament_index_python.packages import get_package_share_directory

from skydio_x2_sim.scenario.scenario_manual_control import ScenarioX2ManualControl
from skydio_x2_sim.mjc_simulate.mjc_sim import MujocoSim
from skydio_x2_sim.mjc_simulate.mjc_headless import MujocoHeadless

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parsed_args, _ = parser.parse_known_args()

    
    pkg_share = get_package_share_directory('skydio_x2_sim')
    xml_path = os.path.join(pkg_share, 'mjcf', 'scene.xml')

    simulation_time = float('inf')
    time_step = None
    fps = 60
    num_drones = 1
    render_mode = parsed_args.gui

    scenario = ScenarioX2ManualControl()

    try:
        if render_mode:
            sim = MujocoSim(xml_path, num_drones, simulation_time, time_step, fps, scenario, None, True)
            sim.main_loop()
        else:
            sim = MujocoHeadless(xml_path, num_drones, simulation_time, time_step, fps, scenario)
            sim.main_loop()

    except KeyboardInterrupt:
        pass
    finally:
        scenario.finish()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()