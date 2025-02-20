from openai import OpenAI
import base64
import os

def encode_image(image_path):
    """将图像文件编码为Base64格式"""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

def get_image_description(image_path):
    """使用OpenAI API获取图像描述"""
    api_key = "sk-d34cba22d2a04a5c8c191f082106d07e"
    base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    
    base64_image = encode_image(image_path)
    
    client = OpenAI(
        api_key=api_key,
        base_url=base_url
    )
    
    completion = client.chat.completions.create(
        model="qwen-vl-max-latest",
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                    },
                    {"type": "text", "text": "这是无人机捕捉到的图像，已经经过yolo标注，请详细描述图片中的内容？"},
                ],
            }
        ],
    )
    
    return completion.choices[0].message.content


