import dashscope
import json
import yaml
from src.code.Entity import Instruction

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

def step_to_command(step):
    # 读取提示词
    # with open('resources/prompt.yaml', 'r', encoding='utf-8') as file:
    #     prompts = yaml.safe_load(file)
    import os
    # 读取提示词
    prompt_path = os.path.expanduser('~/Desktop/LLMDrone/resources/prompt.yaml')
    with open(prompt_path, 'r', encoding='utf-8') as file:
        prompts = yaml.safe_load(file)
    
    # 格式化提示词
    prompt = prompts['stepTranslate'].format(step=step)
    
    # 调用API生成指令
    response = call_with_messages(prompt)

    # 解析响应
    content = response['output']['choices'][0]['message']['content']
    
    # 解析JSON字符串
    try:
        commands_data = json.loads(content)
    except json.JSONDecodeError as e:
        print(f"JSONDecodeError: {e}")
        print(f"Failed to parse content: {content}")
        commands_data = []
    

    return commands_data
