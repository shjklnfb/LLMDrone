from src.code.Entity import *
from src.llm.TaskDecompose import TaskDecompose
import json
import re
import ast
import os
from datetime import datetime

'''
任务规划器：
对用户的输入的任务进行分解
'''
class TaskPlanner:
    def __init__(self):
        # 分解后的子任务列表
        self.subtasks = []

    '''
    调用大模型对任务进行分解
    '''
    def plan_task(self, user_task):
        taskDecompose = TaskDecompose()
        # 调用任务分解方法
        response = taskDecompose.task_decompose(user_task)
        self.subtasks = TaskPlanner.format_decompose(response)
        log_response(response, 'task_decompose')

        # 调用步骤生成方法
        for subtask in self.subtasks:
            response = taskDecompose.generate_instruction(subtask)
            subtask.instructions = TaskPlanner.format_instruction(response)
            log_response(response, 'generate_instruction')
        # 返回子任务，包含名称、要求、指令等
        return self.subtasks
    
    '''
    将LLM任务分解结果转化子任务对象
    '''
    def format_decompose(response):
        # 提取子任务数据
        content = response['output']['choices'][0]['message']['content']
        # 去掉内容中的 ``` 和 ```，确保是一个有效的JSON字符串
        content = content.strip("```").strip()
        content = content.strip()
        # 去掉前面的 "json" 字符串
        content = re.sub(r'^json\s*', '', content)
        # 解析JSON字符串
        subtasks_data = []
        if content:
            try:
                subtasks_data = json.loads(content)
            except json.JSONDecodeError as e:
                log_error(f"JSONDecodeError: {e}\nFailed to parse content: {content}")

        # 将解析后的数据转化为子任务对象
        subtasks = []
        for subtask_data in subtasks_data:
            subtask = SubTask(
            task_id=subtask_data['task_id'],
            name=subtask_data['name'],
            priority=subtask_data['priority'],
            dep_id=subtask_data['dep_id'],
            is_interrupt=subtask_data['is_interrupt'],
            device=None,
            requirements=subtask_data['requirements']
            )
            subtasks.append(subtask)
        return subtasks
    
    # 将步骤生成结果格式化
    def format_instruction(response):
        # 提取指令数据
        content = response['output']['choices'][0]['message']['content']
        # 去掉内容中的 ``` 和 ```，确保是一个有效的JSON字符串
        content = content.strip("```").strip()
        content = re.sub(r'^json\s*', '', content)
        # 解析JSON字符串
        try:
            instructions_data = json.loads(content)
        except json.JSONDecodeError as e:
            log_error(f"JSONDecodeError: {e}\nFailed to parse content: {content}")
            instructions_data = []
        return instructions_data

def log_response(response, log_type):
    log_dir = 'log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_{log_type}.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(json.dumps(response, ensure_ascii=False, indent=4))

def log_error(message):
    log_dir = 'log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_error.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(message)



