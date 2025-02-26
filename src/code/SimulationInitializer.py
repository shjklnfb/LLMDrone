import json

class SimulationInitializer:
    def __init__(self, drones, models, map):
        self.drones = drones
        self.models = models
        self.map = map

    '''
    配置无人机仿真
    '''
    def conf_drone(self):
        pass

        
    '''
    生成启动文件
    '''
    def generate_init_file(drones, map):
        from src.llm.Initialize import generate_launch_file
        return generate_launch_file(drones, map)

