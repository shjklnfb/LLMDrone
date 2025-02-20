import dashscope
import json


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
        return responses
    else:
        print("error")

def choose_drone(subtask):
    pass