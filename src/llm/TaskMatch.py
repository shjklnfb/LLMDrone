import dashscope
import json
import os
from datetime import datetime
import yaml
from src.code.Entity import SubTask

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
    if responses.status_code == 200:
        return responses
    else:
        log_error("Error in call_with_messages")
        return None

def log_response(response):
    log_dir = 'log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(json.dumps(response, ensure_ascii=False, indent=4))

def log_error(message):
    log_dir = 'log/task'
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_error.log")
    with open(log_file, 'w', encoding='utf-8') as file:
        file.write(message)

def choose_drone(subtasks):
    subtasks_dict = [subtask.to_dict() for subtask in subtasks]
    subtasks_json = json.dumps(subtasks_dict, ensure_ascii=False, indent=4)
    
    # # 读取提示词
    # with open('resources/prompt.yaml', 'r', encoding='utf-8') as file:
    #     prompts = yaml.safe_load(file)
    import os
    # 读取提示词
    prompt_path = os.path.expanduser('~/Desktop/LLMDrone/resources/prompt.yaml')
    with open(prompt_path, 'r', encoding='utf-8') as file:
        prompts = yaml.safe_load(file)
    
    # 格式化提示词
    prompt = prompts['DroneAssign'].format(subtasks=subtasks_json)
    
    response = call_with_messages(prompt)

    log_response(response)
    if response and response.get('status_code') == 200:
        response_content = response.get('output', {}).get('choices', [{}])[0].get('message', {}).get('content', '')
        
        try:
            # 去掉多余的字符串标记
            if response_content.startswith("```json"):
                response_content = response_content[7:]
            if response_content.endswith("```"):
                response_content = response_content[:-3]
            
            assigned_subtasks_json = json.loads(response_content)
            assigned_subtasks = []
            for subtask_json in assigned_subtasks_json:
                subtask = SubTask(
                    task_id=subtask_json['id'],
                    name=subtask_json['name'],
                    priority=subtask_json['priority'],
                    dep_id=subtask_json['dep_id'],
                    is_interrupt=subtask_json['is_interrupt'],
                    device=subtask_json['device'],
                    instructions=subtask_json.get('instructions', []),
                    requirements=subtask_json.get('requirements', {}),
                    models=subtask_json.get('models', [])
                )
                assigned_subtasks.append(subtask)
            return assigned_subtasks
        except json.JSONDecodeError:
            log_error(f"Failed to decode JSON from response: {response_content}")
            return []
    else:
        log_error("Failed to get a valid response from the model")
        return []