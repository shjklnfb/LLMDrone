from src.code.TaskPlanner import TaskPlanner
from src.code.TaskExecutor import TaskExecutor
from src.code.InterruptListener import InterruptListener
from src.code.TaskAssigner import TaskAssigner
import rospy

# 主函数
def main():
    rospy.init_node('control_script', anonymous=True, disable_signals=True)
    res = TaskPlanner().plan_task("无人机搜索红色房顶的加油站，侦查它的情况")
    res = TaskAssigner(None,None,res).task_with_drone()
    task_executor = TaskExecutor()
    for i in res:
        print(i)
        task_executor.add_subtask(i)
    interrupt_listener1 = InterruptListener(task_executor, 5000)
    interrupt_listener1.start()
    task_executor.execute()