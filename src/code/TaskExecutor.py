import threading
import queue
import time
from src.code.Entity import SubTask, TaskState

"""
TaskExecutor 类负责调度和执行子任务。
任务执行器存储任务分解得到的子任务，所有子任务加入到子任务池当中。任务执行器需要维护子任务的状态，确保子任务之间依赖关系的满足，并且处理突发任务。对于满足依赖关系的子任务，可以加入到优先级队列中，优先级0为突发子任务队列，优先级最高，其他为正常的子任务。所有在优先级队列当中的任务都是已经进入就绪态的任务。
"""
# 任务执行器类 (单例模式)
class TaskExecutor:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super(TaskExecutor, cls).__new__(cls)
        return cls._instance

    @classmethod
    def get_instance(cls):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = cls()
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.subtask_pool = []  # 子任务池列表
            self.priority_queue = queue.PriorityQueue()  # 优先级队列
            self.drone_units = {}  # 存储已启动的无人机控制单元
            self.lock = threading.Lock()
            self.initialized = True
    
    # 添加子任务并检查依赖关系
    def add_subtask(self, subtask: SubTask):
        with self.lock:
            self.subtask_pool.append(subtask)
            # 无依赖关系的任务直接加入优先级队列
            if self.check_dependencies(subtask):
                self.priority_queue.put((subtask.priority, subtask))

    # 检查子任务的依赖关系
    def check_dependencies(self, subtask: SubTask) -> bool:
        # 如果子任务没有依赖关系，直接返回True
        if not subtask.dep_id:
            return True
        # 遍历子任务的所有依赖ID
        for dep_id in subtask.dep_id:
            dep_met = False
            # 检查子任务池中是否有对应的已完成任务
            for task in self.subtask_pool:
                if task.id == dep_id and task.state == TaskState.COMPLETED:
                    dep_met = True
                    break
            # 如果有任何一个依赖没有满足，返回False
            if not dep_met:
                return False
        # 所有依赖都满足，返回True
        return True
    
    # 获取下一个子任务
    def get_subtask(self):
        with self.lock:
            if not self.priority_queue.empty():
                subtask = self.priority_queue.get()[1]
                return subtask
        return None
    
    # 执行子任务
    def execute(self):
        while True:
            subtask = self.get_subtask()
            if subtask:
                # 处理突发任务
                if subtask.priority == 0:
                    from src.code.DroneUnit import DroneUnit
                    if subtask.device:
                        if subtask.device in self.drone_units:
                            drone_unit = self.drone_units[subtask.device]
                            drone_unit.scheduler.handle_interrupt(subtask)
                        else:
                            drone_unit = DroneUnit(subtask.device)
                            drone_unit.add_subtask(subtask)
                            drone_unit.start()
                            self.drone_units[subtask.device] = drone_unit
                    else:
                        print(f"{subtask.name} device not defined")
                    subtask.state = TaskState.RUNNING
                else:
                    # 启动无人机控制单元，三个线程，执行子任务
                    if subtask.device:
                        from src.code.DroneUnit import DroneUnit
                        if subtask.device in self.drone_units:
                            drone_unit = self.drone_units[subtask.device]
                            drone_unit.add_subtask(subtask)
                        else:
                            drone_unit = DroneUnit(subtask.device)
                            drone_unit.add_subtask(subtask)
                            drone_unit.start()
                            self.drone_units[subtask.device] = drone_unit
                    else:
                        print(f"{subtask.name} device not defined")
                    subtask.state = TaskState.RUNNING
            time.sleep(1)
    
    # 完成子任务并检查依赖关系
    def complete_task(self, subtask: SubTask):
        subtask.state = TaskState.COMPLETED
        with self.lock:
            for task in self.subtask_pool:
                if task.dep_id and subtask.id in task.dep_id and self.check_dependencies(task):
                    task.state = TaskState.READY
                    self.priority_queue.put((task.priority, task))
        print(f"Task {subtask.name} completed and dependencies checked.")
