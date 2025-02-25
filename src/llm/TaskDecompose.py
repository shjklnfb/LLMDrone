from src.code.Entity import SubTask,Instruction
from http import HTTPStatus
import dashscope
import re
import ast
import json
import yaml
import os
from datetime import datetime

# 通义千问plus api
def call_with_messages(prompt):
    messages = [{"role":"system","content":"你是一个关于无人机执行任务，进行任务分解规划的专家。"},
                {"role":"user","content":""}]
    messages[1]["content"] = prompt
    responses = dashscope.Generation.call(
        model="qwen-plus",
        api_key="sk-d34cba22d2a04a5c8c191f082106d07e",
        messages=messages,
        stream=False,
        result_format='message',  # 将返回结果格式设置为 message
        top_p=0.8,
        temperature=0.7,
        enable_search=False
    )
    if(responses.status_code == 200):
        log_response(responses)
        return responses
    else:
        log_error("Error in call_with_messages")

def log_response(response):
    log_dir = '../log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(json.dumps(response, ensure_ascii=False, indent=4))

def log_error(message):
    log_dir = '../log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_error.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(message)

def format_instruction(response):
    # 提取子任务数据
    content = response['output']['choices'][0]['message']['content']

    # 解析JSON字符串
    try:
        instructions_data = json.loads(content)
    except json.JSONDecodeError as e:
        log_error(f"JSONDecodeError: {e}\nFailed to parse content: {content}")
        instructions_data = []

    # 创建Instruction对象列表
    instructions = [
        Instruction(item['command'], item['params']) for item in instructions_data
    ]
    return instructions

class TaskDecompose:
    def __init__(self):
        import os
        # 读取提示词
        prompt_path = os.path.expanduser('~/Desktop/LLMDrone_2/resources/prompt.yaml')
        with open(prompt_path, 'r', encoding='utf-8') as file:
            self.prompts = yaml.safe_load(file)

    # 根据用户任务，生成子任务列表
    def task_decompose(self, user_task):
        prompt = self.prompts['task_decompose'].format(user_task=user_task)
        response = call_with_messages(prompt)
        log_response(response)
        return response

    # 为每个子任务生成步骤并添加到指令列表中
    def generate_instruction(self, sub_task):
        prompt = self.prompts['generate_instruction'].format(
            sub_task_id=sub_task.id,
            sub_task_name=sub_task.name,
            sub_task_priority=sub_task.priority,
            sub_task_dep_id=sub_task.dep_id,
            sub_task_is_interrupt=sub_task.is_interrupt,
            sub_task_requirements=sub_task.requirements
        )
        response = call_with_messages(prompt)
        log_response(response)
        return response

