from src.code.TaskPlanner import TaskPlanner
from src.code.TaskExecutor import TaskExecutor
from src.code.InterruptListener import InterruptListener
from src.code.TaskAssigner import TaskAssigner

# 主函数
def main():
    res = TaskPlanner().plan_task("无人机搜索居民楼楼顶的人员")
    res = TaskAssigner(None,None,res).task_with_drone()
    task_executor = TaskExecutor()
    for i in res:
        print(i)
        task_executor.add_subtask(i)
    interrupt_listener1 = InterruptListener(task_executor, 5000)
    interrupt_listener1.start()
    task_executor.execute()