import dashscope
import json
import yaml

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
        print(responses)


def checkStep(data, step, image_info=None, short_term_memory=None):
    import os
    # 读取提示词
    prompt_path = os.path.expanduser('~/Desktop/LLMDrone/resources/prompt.yaml')
    with open(prompt_path, 'r', encoding='utf-8') as file:
        prompts = yaml.safe_load(file)
    
    # 获取短期记忆中的内容
    memory_content = "\n".join(short_term_memory.get_recent()) if short_term_memory else ""
    prompt = prompts['stepCheck'].format(data=data, step=step, image_info=image_info, memory_content=memory_content)
    
    response = call_with_messages(prompt)
    print(response)
    if response:
        response_content = json.loads(response['output']['choices'][0]['message']['content'])
        result = response_content[0]['result']
        instruction = response_content[0]['instruction']
        # 将问题和结果保存到短期记忆中
        if short_term_memory:
            short_term_memory.add(f"Prompt: {prompt}\nResult: {result}\nInstruction: {instruction}")
        return result, instruction
    else:
        return None, None