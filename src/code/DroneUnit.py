import threading
from collections import defaultdict

from src.code.Entity import SubTask
from src.code.DroneMonitor import DroneMonitor
from src.code.InterruptListener import InterruptListener
from src.code.TaskExecutor import TaskExecutor
from src.code.DroneScheduler import DroneScheduler

# 无人机控制单元类
class DroneUnit(threading.Thread):
    _lock = threading.Lock()

    def __init__(self, drone_id):
        super().__init__()
        self.drone_id = drone_id
        self.shared_data = defaultdict(lambda: {
            'state': None,
            'position': None,
            'velocity': None,
            'image': None,
            'other': None
        })
        self.task_executor = TaskExecutor()  # 使用单例模式的TaskExecutor实例

        self.monitor = DroneMonitor(self.drone_id, self.shared_data)
        self.scheduler = DroneScheduler(self.drone_id, self.shared_data)

    # 添加子任务到调度器
    def add_subtask(self, subtask: SubTask):
        self.scheduler.add_subtask(subtask)

    def run(self):
        self.monitor.start()
        self.scheduler.start()
