import os
import mujoco as mj
import numpy as np

os.environ['MUJOCO_GL'] = 'egl'

class MujocoHeadless:

    def __init__(self, xml_name, num_drones, simulation_time, time_step, fps, scenario):
        dirname = os.path.dirname(__file__)
        abspath = os.path.join(dirname, xml_name)

        self.model = mj.MjModel.from_xml_path(abspath)
        if time_step is not None:
            self.model.opt.timestep = time_step
        self.time_step = self.model.opt.timestep
        self.data = mj.MjData(self.model)
        
        self.cam = mj.MjvCamera()
        self.opt = mj.MjvOption()

        self.simulation_time = simulation_time
        self.fps = fps
        self.num_drones = num_drones
        self.scenario = scenario
        self.counter = 0

    def main_loop(self):
        def init_controller(model, data):
            self.scenario.init(self, model, data)

        def controller(model, data):
            self.scenario.update(self, model, data)

        init_controller(self.model, self.data)
        mj.set_mjcb_control(controller)

        while self.data.time < self.simulation_time:
            mj.mj_step(self.model, self.data)
            self.scenario.render(self.data, self.data.time)
            self.counter += 1
            