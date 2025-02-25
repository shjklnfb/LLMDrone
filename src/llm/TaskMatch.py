import dashscope
import json
import os
from datetime import datetime

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

def choose_drone(subtask):
    pass