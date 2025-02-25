import threading
import queue
import time
from src.code.DroneExecutor import DroneExecutor
from src.code.Entity import SubTask, TaskState
from src.llm.CheckStep import checkStep

# 无人机调度器类
class DroneScheduler(threading.Thread):
    def __init__(self, drone_id, shared_data):
        super().__init__()
        self.drone_id = drone_id
        self.shared_data = shared_data
        self.subtask_queue = queue.Queue()  # 子任务队列
        self.step_queue = queue.Queue()  # 步骤队列
        self.current_subtask = None

    # 添加子任务到队列
    def add_subtask(self, subtask: SubTask):
        self.subtask_queue.put(subtask)
        # 将子任务的步骤加入步骤队列
        for step in subtask.instructions:
            self.step_queue.put((subtask, step))

    def run(self):
        while True:
            # 不断取步骤，使用执行器执行
            if not self.step_queue.empty():
                subtask, step = self.step_queue.get()
                drone_executor = DroneExecutor(self.drone_id, step, self.shared_data)
                drone_executor.start()
                drone_executor.join()  # 等待步骤执行完成
                if self.step_queue.empty() or self.step_queue.queue[0][0] != subtask:
                    self.notify_task_completion(subtask)
            time.sleep(1)
    
    # 通知任务完成
    def notify_task_completion(self, subtask: SubTask):
        from src.code.TaskExecutor import TaskExecutor
        subtask.state = TaskState.COMPLETED
        TaskExecutor.get_instance().complete_task(subtask)

    # 处理突发任务
    def handle_interrupt(self, emergency_task: SubTask):
        # 保存当前子任务和步骤队列
        saved_subtask = self.current_subtask
        saved_step_queue = list(self.step_queue.queue)
        saved_subtask_queue = list(self.subtask_queue.queue)
        
        # 清空当前队列
        self.step_queue.queue.clear()
        self.subtask_queue.queue.clear()
        
        # 获取突发任务
        if emergency_task:
            print(f"Handling emergency task {emergency_task}")
            self.add_subtask(emergency_task)
            while not self.step_queue.empty():
                subtask, step = self.step_queue.get()
                drone_executor = DroneExecutor(self.drone_id, step, self.shared_data)
                drone_executor.start()
                drone_executor.join()
            self.notify_task_completion(emergency_task)
        
        # 恢复之前的子任务和步骤队列
        self.current_subtask = saved_subtask
        self.step_queue = queue.Queue()
        for item in saved_step_queue:
            self.step_queue.put(item)
        self.subtask_queue = queue.Queue()
        for item in saved_subtask_queue:
            self.subtask_queue.put(item)
        print("Emergency task handled")
        print("Resuming task")
