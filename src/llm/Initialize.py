import dashscope
import json
import yaml
from src.code.Entity import Instruction
import os
from datetime import datetime  # 修正导入

def call_with_messages(prompt):
    messages = [{"role":"system","content":"你是一个关于无人机执行任务的专家，了解ROS+PX4无人机仿真，了解无人机的各种话题和服务，懂得无人机基本的控制原理和基本控制指令。"},
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
        return responses
    else:
        print("error")

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


def generate_launch_file(drones, world_file):
    # # 读取提示词
    # with open('resources/prompt.yaml', 'r', encoding='utf-8') as file:
    #     prompts = yaml.safe_load(file)
    import os
    # 读取提示词
    prompt_path = os.path.expanduser('~/Desktop/LLMDrone/resources/prompt.yaml')
    with open(prompt_path, 'r', encoding='utf-8') as file:
        prompts = yaml.safe_load(file)
    prompt = prompts['LaunchFile'].format(drones=drones, world_file=world_file)
    response = call_with_messages(prompt)
    log_response(response)
    if response and response.get('status_code') == 200:
        response_content = response.get('output', {}).get('choices', [{}])[0].get('message', {}).get('content', '')
        try:
            # 去掉多余的字符串标记
            if response_content.startswith("```xml"):
                response_content = response_content[6:]
            if response_content.endswith("```"):
                response_content = response_content[:-3]
            
            # 将内容写入到LLMDrone.launch文件中
            with open('../LLMDrone.launch', 'w', encoding='utf-8') as file:
                file.write(response_content)
            
            launch_file = response_content
        except json.JSONDecodeError as e:
            log_error(f"JSONDecodeError: {e}\nFailed to parse content: {response_content}")
            launch_file = []
    return response