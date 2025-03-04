from collections import deque

class ShortTermMemory:
    """
    短期记忆模块（对话上下文管理）
    使用双端队列实现容量限制和快速更新
    """
    def __init__(self, max_size=5):
        """
        :param max_size: 最大记忆容量（保存最近N轮对话）
        """
        self.memory = deque(maxlen=max_size)
    
    def add(self, info: str):
        """添加新信息到短期记忆"""
        self.memory.append(info)
    
    def get_recent(self, n: int = None) -> list:
        """
        获取最近的N条记忆
        :param n: 获取条数（默认返回全部）
        """
        return list(self.memory)[-n:] if n else list(self.memory)
    
    def clear(self):
        """清空短期记忆"""
        self.memory.clear()

