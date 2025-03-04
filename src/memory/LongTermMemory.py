import numpy as np
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

class LongTermMemory:
    """
    长期记忆模块（知识库管理）
    使用向量化存储和语义检索
    """
    def __init__(self):
        self.encoder = SentenceTransformer('src/model/paraphrase-MiniLM-L6-v2')  
        self.knowledge = []     # 存储
        self.embeddings = None  # 向量存储矩阵
    
    def _update_embeddings(self, new_embedding):
        """更新向量存储矩阵"""
        if self.embeddings is None:
            self.embeddings = np.array([new_embedding])
        else:
            self.embeddings = np.vstack([self.embeddings, new_embedding])
    
    def add(self, info: str):
        """添加新知识到长期记忆"""
        # 生成语义向量
        embedding = self.encoder.encode(info)
        # 更新存储
        self.knowledge.append(info)
        self._update_embeddings(embedding)
    
    def search(self, query: str, top_k: int = 3) -> list:
        """
        语义检索最相关的知识
        :param query: 查询文本
        :param top_k: 返回最相关的K条结果
        :return: (相似度, 知识) 元组列表
        """
        # 生成查询向量
        query_embedding = self.encoder.encode(query)
        # 计算余弦相似度
        similarities = cosine_similarity([query_embedding], self.embeddings)[0]
        # 获取TopK结果
        sorted_indices = np.argsort(similarities)[::-1][:top_k]
        return [(similarities[i], self.knowledge[i]) for i in sorted_indices]
    
    def batch_add(self, infos: list):
        """批量添加知识"""
        embeddings = self.encoder.encode(infos)
        self.knowledge.extend(infos)
        self.embeddings = np.vstack([self.embeddings, embeddings]) if self.embeddings is not None else embeddings

