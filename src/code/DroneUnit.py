import threading
from src.code.Entity import SubTask
from src.code.DroneMonitor import DroneMonitor
from src.code.InterruptListener import InterruptListener
from src.code.TaskExecutor import TaskExecutor
from src.code.DroneScheduler import DroneScheduler

# 无人机控制单元类
class DroneUnit(threading.Thread):
    _port = 5000
    _lock = threading.Lock()

    def __init__(self, drone_id):
        super().__init__()
        self.drone_id = drone_id
        self.shared_data = {}
        self.task_executor = TaskExecutor()  # 使用单例模式的TaskExecutor实例
        with self._lock:
            self.port = DroneUnit._port
            DroneUnit._port += 1
        self.interrupt_listener = InterruptListener(self.task_executor, self.port)
        self.monitor = DroneMonitor(self.drone_id, self.shared_data)
        self.scheduler = DroneScheduler(self.drone_id, self.shared_data, self.interrupt_listener)

    # 添加子任务到调度器
    def add_subtask(self, subtask: SubTask):
        self.scheduler.add_subtask(subtask)

    def run(self):
        self.monitor.start()
        self.scheduler.start()
        self.interrupt_listener.start()