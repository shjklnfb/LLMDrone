import threading
import time
import queue
from flask import Flask, request, jsonify
from src.code.TaskExecutor import TaskExecutor
from src.code.Entity import SubTask

# 突发监听器类
class InterruptListener(threading.Thread):
    def __init__(self, task_executor: TaskExecutor, port: int):
        super().__init__()
        self.task_executor = TaskExecutor.get_instance()
        self.port = port
        self.interrupt_flag = threading.Event()
        self.emergency_queue = queue.Queue()
        self.app = Flask(__name__)
        self.setup_routes()

    def setup_routes(self):
        @self.app.route('/interrupt', methods=['POST'])
        def interrupt():
            data = request.get_json()
            emergency_task = SubTask(
                data['task_id'],
                data['name'],
                data['priority'],
                data['dep_id'],
                data['is_interrupt'],
                data['device'],
                data['instructions']
            )
            self.emergency_queue.put(emergency_task)
            self.interrupt_flag.set()
            return jsonify({"status": "success"}), 200

    def run(self):
        threading.Thread(target=self.app.run, kwargs={'port': self.port}).start()
        while True:
            # 监听突发任务
            print(f"Listening for emergency tasks on port {self.port}")
            time.sleep(3)
            if not self.emergency_queue.empty():
                emergency_task = self.emergency_queue.get()
                print(f"Received emergency task {emergency_task}")
                self.task_executor.add_subtask(emergency_task)
                self.interrupt_flag.set()

    # 检查是否有突发任务
    def check_interrupt(self):
        return self.interrupt_flag.is_set()
    
    # 清除突发任务标志
    def clear_interrupt(self):
        self.interrupt_flag.clear()