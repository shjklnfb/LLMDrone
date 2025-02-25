from src.code.Entity import Drone,SubTask
import os
from datetime import datetime
'''
任务分配器：
负责将任务分配给无人机，无人机是要在无人机库中进行选择，根据能力匹配，需要注意的是有些子任务必须分配到同一台无人机上。
根据功能为无人机选择对应的模型完成功能，根据能力匹配
'''
class TaskAssigner:
    def __init__(self, drones, models, subtasks):
        # 可选无人机
        self.drones = drones
        # 可选模型
        self.models = models
        # 子任务列表
        self.subtasks = subtasks

    '''
    向分配器中添加可选的无人机
    '''
    def add_drone(self, drone):
        self.drones.append(drone)

    '''
    向分配器中添加可选的模型
    '''
    def add_model(self, model):
        self.models.append(model)

    '''
    将子任务与无人机进行匹配
    '''
    def task_with_drone(self):   
        #TODO 匹配        
        for i in self.subtasks:
            i.device = "typhoon_h480_1"
        log_subtasks(self.subtasks, 'task_with_drone')
        return self.subtasks
    
    '''
    将子任务与模型进行匹配
    '''
    def task_with_model(self):
        #TODO 匹配
        log_subtasks(self.subtasks, 'task_with_model')
        return self.subtasks

def log_subtasks(subtasks, log_type):
    log_dir = '../log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_{log_type}.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        for subtask in subtasks:
            file.write(f"Subtask: {subtask.name}, Device: {subtask.device}\n")

def log_error(message):
    log_dir = '../log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_error.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(message)