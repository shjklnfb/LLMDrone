import sys , os
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(base_dir)
# import rospy  # 添加导入rospy
import time
import queue
import logging
import threading
import unittest
from flask import Flask, request, jsonify
from src.code.Entity import *

# 测试类
class TestModule1(unittest.TestCase):


    # 1. 测试LLM任务分解
    # def test_decompose(self):
        # from src.llm.TaskDecompose import TaskDecompose
        # print(TaskDecompose().task_decompose("航拍"))
        # print(TaskDecompose().generate_instruction(SubTask(task_id=1, name="无人机起飞并飞行到居民楼楼顶", priority=5, dep_id=[], is_interrupt=0, device="typhoon_h480_0")))
        # planner=TaskPlanner()
        # result=planner.plan_task(user_task="在太行山航拍")
        # print(result)

    # 2. 测试任务规划
    # def test_planner(self):
    #     from src.code.TaskPlanner import TaskPlanner
    #     planner=TaskPlanner()
    #     result=planner.plan_task(user_task="在太行山航拍")
    #     from src.llm.TaskMatch import choose_drone
    #     res=choose_drone(result)
    #     for i in res:
    #         print(i)

    # 3. 测试无人机指令执行器
    # def test_droneExecutor(self):
    #     executor = DroneExecutor("typhoon_h480_0","降落0.5m",None)
    #     executor.run()

    # 4. 测试无人机监控器
    # def test_droneMonitor(self):
    #     from src.code.DroneMonitor import DroneMonitor
    #     monitor = DroneMonitor("typhoon_h480_0",{"sensor_data":"fda"})
    #     monitor.run()

    # 5. 测试任务执行器
    # def test_taskExecutor(self):
    #     from src.code.TaskExecutor import TaskExecutor
    #     from src.code.InterruptListener import InterruptListener
    #     subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="iris_0", instructions=[
    #         {
    #         "name": "takeoff",
    #         "description": "无人机垂直起飞到3m"
    #         },
    #         {
    #         "name": "ascend",
    #         "description": "无人机继续垂直攀升到20m"
    #         },
    #         {
    #         "name": "yaw",
    #         "description": "无人机环绕一圈，检查周围情况"
    #         },
    #         {
    #         "name": "land",
    #         "description": "无人机降落到原地"
    #         },
    #     ])
    #     subtask2 = SubTask(
    #         task_id=2, name="subtask2", priority=2, dep_id=[1], is_interrupt=0, device="iris_0", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞"
    #         },
    #         {
    #             "name": "move_forward",
    #             "description": "无人机向前飞行"
    #         },
    #         {
    #             "name": "hover",
    #             "description": "无人机在当前位置悬停"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机从当前位置降落"
    #         },
    #         ]
    #     )
    #     subtask3 = SubTask(
    #         task_id=3, name="subtask3", priority=3, dep_id=[2], is_interrupt=0, device="iris_0", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞"
    #         },
    #         {
    #             "name": "move_left",
    #             "description": "无人机向左飞行"
    #         },
    #         {
    #             "name": "hover",
    #             "description": "无人机在当前位置悬停"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机从当前位置降落"
    #         },
    #         ]
    #     )
    #     subtask4 = SubTask(
    #         task_id=4, name="subtask4", priority=4, dep_id=[], is_interrupt=0, device="typhoon_h480_1", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞"
    #         },
    #         {
    #             "name": "move_right",
    #             "description": "无人机向右飞行"
    #         },
    #         {
    #             "name": "hover",
    #             "description": "无人机在当前位置悬停"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机从当前位置降落"
    #         },
    #         ]
    #     )
    #     subtask5 = SubTask(
    #         task_id=5, name="subtask5", priority=4, dep_id=[], is_interrupt=0, device="typhoon_h480_1", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞"
    #         },
    #         {
    #             "name": "move_backward",
    #             "description": "无人机向后飞行"
    #         },
    #         {
    #             "name": "hover",
    #             "description": "无人机在当前位置悬停"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机从当前位置降落"
    #         },
    #         ]
    #     )

    #     task_executor = TaskExecutor()
    #     task_executor.add_subtask(subtask1)
    #     task_executor.add_subtask(subtask2)
    #     task_executor.add_subtask(subtask3)
    #     task_executor.add_subtask(subtask4)
    #     task_executor.add_subtask(subtask5)

    #     interrupt_listener1 = InterruptListener(task_executor, 5000)

    #     interrupt_listener1.start()

    #     task_executor.execute()
    #     rospy.spin()  # 保持ROS节点运行

    # 6. 测试步骤转指令
    # def test_stepTranslate(self):
    #     from src.llm.StepTranslate import step_to_command
        # step =   [
        #     {
        #         "name": "takeoff",
        #         "description": "无人机垂直起飞到3m"
        #     },
        #     {
        #         "name": "ascend",
        #         "description": "无人机继续垂直攀升到20m"
        #     },
        #     {
        #         "name": "yaw",
        #         "description": "无人机环绕一圈，检查周围情况"
        #     },
        #     {
        #         "name": "land",
        #         "description": "无人机降落到原理"
        #     },
        # ]
    #     command = step_to_command(step)
    #     print(command)

    # 7. 测试无人机指令执行器（只有翻译执行功能）
    # def test_droneExecutor(self):
    #     from src.code.DroneExecutor import DroneExecutor
    #     step =   [
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞到3m"
    #         },
    #     ]
    #     executor = DroneExecutor("typhoon_h480_0", step, None)
    #     executor.run()

    # 8. 测试无人机调度器
    # def test_droneScheduler(self):
    #     from src.code.DroneScheduler import DroneScheduler
    #     scheduler = DroneScheduler("typhoon_h480_0", None, None)
    #     subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="typhoon_h480_0", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞到3m"
    #         },
    #         {
    #             "name": "ascend",
    #             "description": "无人机继续垂直攀升到20m"
    #         },
    #         {
    #             "name": "yaw",
    #             "description": "无人机环绕一圈，检查周围情况"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机降落到原理"
    #         },
    #     ])
    #     scheduler.add_subtask(subtask1)
    #     scheduler.run()

    # 9. 测试无人机控制单元
    # def test_droneUnit(self):
    #     # import rospy
    #     # rospy.init_node('control_script', anonymous=True, disable_signals=True)
    #     from src.code.DroneUnit import DroneUnit
    #     drone_unit = DroneUnit("typhoon_h480_0")
    #     subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="typhoon_h480_0", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞到3m"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机降落到原理"
    #         },
    #     ])
    #     drone_unit.add_subtask(subtask1)
    #     drone_unit.run()

    #     drone_unit1 = DroneUnit("typhoon_h480_1")
    #     subtask2 = SubTask(task_id=2, name="subtask2", priority=5, dep_id=[], is_interrupt=0, device="typhoon_h480_1", instructions=[
    #         {
    #             "name": "takeoff",
    #             "description": "无人机垂直起飞到10m"
    #         },
    #         {
    #             "name": "yaw",
    #             "description": "无人机环绕一圈，检查周围情况"
    #         },
    #         {
    #             "name": "land",
    #             "description": "无人机降落到原理"
    #         },
    #     ])
    #     drone_unit1.add_subtask(subtask2)
    #     drone_unit1.run()

    # 10. 测试突发监听器
    # def test_interruptListener(self):
    #     from src.code.InterruptListener import InterruptListener
    #     from src.code.TaskExecutor import TaskExecutor
    #     task_executor = TaskExecutor()
    #     interrupt_listener1 = InterruptListener(task_executor, 5000)
    #     interrupt_listener1.start()

    # 11. 测试多无人机同时起飞
    # def test_multiDrone(self):
    #     from src.instructions.command_takeoff import command_takeoff
    #     from src.instructions.command_pos import publish_command as command_pos

    #     # 启动两个无人机控制线程
    #     threading.Thread(target=command_takeoff, args=("typhoon_h480_0",)).start()
    #     threading.Thread(target=command_takeoff, args=("typhoon_h480_1",)).start()

    #     # 发送位置指令
    #     time.sleep(5)  # 等待无人机起飞
    #     command_pos("typhoon_h480_0", 0, 0, 3)
    #     command_pos("typhoon_h480_1", 0, 0, 10)

    # 12. 测试单无人机全流程执行
    # def test_singleDrone(self):
    #     from src.code.TaskPlanner import TaskPlanner
    #     from src.code.TaskExecutor import TaskExecutor
    #     from src.code.InterruptListener import InterruptListener
    #     from src.code.TaskAssigner import TaskAssigner
    #     res = TaskPlanner().plan_task("无人机搜索居民楼楼顶的人员")
    #     res = TaskAssigner(None,None,res).task_with_drone()
    #     task_executor = TaskExecutor()
    #     for i in res:
    #         print(i)
    #         task_executor.add_subtask(i)
    #     interrupt_listener1 = InterruptListener(task_executor, 5000)
    #     interrupt_listener1.start()
    #     task_executor.execute()

    # 13. 测试突发任务的执行
    # def test_interruptTask(self):

    # 14. 测试无人机监控器
    # def test_droneMonitor(self):
    #     from src.code.DroneMonitor import DroneMonitor
    #     DroneMonitor("typhoon_h480_0", None).run()

    # 15. 测试多无人机同时执行任务流程
    # def test_multiDrone(self):
    #     from src.code.TaskPlanner import TaskPlanner
    #     from src.code.TaskExecutor import TaskExecutor
    #     from src.code.InterruptListener import InterruptListener
    #     from src.code.TaskAssigner import TaskAssigner
    #     res = TaskPlanner().plan_task("同时启动两台无人机搜索居民楼楼顶的人员")
    #     # res = TaskAssigner(None,None,res).task_with_drone()
    #     for(i) in res:
    #         print(i)

    # 16. 测试启动文件生成
    # def test_generate_launch_file(self):
    #     from src.llm.Initialize import generate_launch_file
    #     generate_launch_file(["typhoon_h480_0","typhoon_h480_1","iris_0"],"world_file")

    # 17. 测试任务阶段流程
    # def test_task_stage(self):
    #     from src.code.TaskPlanner import TaskPlanner
    #     from src.code.TaskExecutor import TaskExecutor
    #     from src.code.InterruptListener import InterruptListener
    #     from src.code.TaskAssigner import TaskAssigner
    #     from src.code.SimulationInitializer import SimulationInitializer 
    #     res = TaskPlanner().plan_task("无人机搜索居民楼楼顶的人员")
    #     res = TaskAssigner(None,None,res).task_with_drone()
    #     maps = "world_file"
    #     drones = [subtask.device for subtask in res]
    #     SimulationInitializer.generate_init_file(drones,maps)
    #     # task_executor = TaskExecutor()
    #     # for i in res:
    #     #     print(i)
    #     #     task_executor.add_subtask(i)
    #     # interrupt_listener1 = InterruptListener(task_executor, 5000)
    #     # interrupt_listener1.start()
    #     # task_executor.execute()

    #  18. 测试步骤反馈
    def test_taskExecutor(self):
        from src.code.TaskExecutor import TaskExecutor
        from src.code.InterruptListener import InterruptListener
        # subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="iris_0", instructions=[
        #     {
        #     "name": "takeoff",
        #     "description": "无人机垂直起飞到3m"
        #     },
        #     {
        #     "name": "ascend",
        #     "description": "飞行到一个没有障碍物的高度,至少20m"
        #     },
        #     {
        #     "name": "search",
        #     "description": "无人机寻找红色小汽车,当发现目标后，悬停在红色小汽车上空.如果没有找到，扩大搜索范围."
        #     },
        #     {
        #     "name": "hover",
        #     "description": "悬停在当前位置,持续一段时间"
        #     },
        #     {
        #     "name": "land",
        #     "description": "无人机降落到当前位置"
        #     },
           
        # ])
    
        subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="iris_0", instructions=[
            {
            "name": "takeoff",
            "description": "无人机垂直起飞到15m"
            },
            {
            "name": "ascend",
            "description": "无人机从15m开始慢慢爬升，不需要搜寻(command_search),开始时可以一次爬升较高的高度，例如5m，后续需要减小每次的爬升高度，例如一次1m,看到小轿车时即满足目标"
            },
            {
            "name": "hover",
            "description": "无人机降落,注:无人机处于auto.land模式，且高度小于1m的时候表示已经降落完成.不需要确保降落完成，不需要重新发送降落指令"
            },     
        ])

        # subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="iris_0", instructions=[
        #     {
        #     "name": "takeoff",
        #     "description": "无人机垂直起飞到3m"
        #     },
        #     {
        #     "name": "ascend",
        #     "description": "无人机从3m开始慢慢爬升，开始时可以一次爬升较高的高度，例如5m，后续需要减小爬升高度，一次1m,直到比房屋的高度高一段距离,确保在当前高度不会撞到房屋"
        #     },
        #     {
        #     "name": "search",
        #     "description": "无人机在当前高度随机飞行，寻找一辆汽车，最多飞行10min，如果找到，无人机返回起飞点，达到最大时间没有找到时也返回起飞点"
        #     }, 
        # ])
        subtask2 = SubTask(task_id=2, name="subtask2", priority=5, dep_id=[1], is_interrupt=0, device="typhoon_h480_1", instructions=[
            {
            "name": "takeoff",
            "description": "无人机垂直起飞到3m"
            },
            {
            "name": "fly_to",
            "description": "无人机上升5m,然后飞行到小轿车所在的位置，注意这个位置是其他无人机确定的，你需要先给一个暂时的保守位置，等待接受确切的位置。"
            },
            {
            "name": "hover",
            "description": "无人机在当前位置悬停"
            },
        ])
        task_executor = TaskExecutor()
        task_executor.add_subtask(subtask1)
        task_executor.add_subtask(subtask2)

        interrupt_listener1 = InterruptListener(task_executor, 5000)

        interrupt_listener1.start()

        task_executor.execute()
        rospy.spin()  # 保持ROS节点运行

    # 19. 测试长短期记忆
    # def test_memory(self):
    #     from src.memory.ShortTermMemory import ShortTermMemory
    #     from src.memory.LongTermMemory import LongTermMemory
    #     # # 初始化短期记忆（保存最近3轮对话）
    #     # st_mem = ShortTermMemory(max_size=3)
    #     # st_mem.add("用户：请问你们有哪些支付方式？")
    #     # st_mem.add("系统：我们支持支付宝、微信支付和银联")
    #     # print("当前短期记忆：", st_mem.get_recent())

    #     # 初始化长期记忆
    #     lt_mem = LongTermMemory()
    #     # 添加知识库数据
    #     lt_mem.batch_add([
    #         "退货政策：签收后7天内支持无理由退货",
    #         "运费规则：订单满99元包邮，普通会员运费10元",
    #         "会员等级：消费累计满500元升级为黄金会员",
    #         "支付方式：支持支付宝、微信支付和银联",
    #         "退款流程：退货后3个工作日内完成退款",
    #         "如果您不满意商品，可以查看我们的退款退货政策"
    #     ])

    #     # 语义检索示例
    #     results = lt_mem.search("如果我不满意商品怎么办？")
    #     print("\n长期记忆检索结果：")
    #     for score, knowledge in results:
    #         print(f"[相似度{score:.3f}] {knowledge}")

    # # 20. main
    # def test_main(self):
    #     from src.code.TaskPlanner import TaskPlanner
    #     from src.code.TaskExecutor import TaskExecutor
    #     from src.code.InterruptListener import InterruptListener
    #     from src.code.TaskAssigner import TaskAssigner
    #     from src.code.SimulationInitializer import SimulationInitializer 
    #     res = TaskPlanner().plan_task("无人机搜索红色房顶的加油站，查看楼顶的情况,并且在楼顶降落")
    #     res = TaskAssigner(None,None,res).task_with_drone()
    #     drones = list(set(subtask.device for subtask in res))
    #     SimulationInitializer.generate_init_file(drones,"outdoor1.world")
    #     time.sleep(10)
    #     import rospy
    #     rospy.init_node('control_script', anonymous=True, disable_signals=True)
    #     task_executor = TaskExecutor()
    #     for i in res:
    #         print(i)
    #         task_executor.add_subtask(i)
    #     interrupt_listener1 = InterruptListener(task_executor, 5000)
    #     interrupt_listener1.start()
    #     task_executor.execute()



if __name__ == '__main__':
    import rospy
    rospy.init_node('control_script', anonymous=True, disable_signals=True)
    unittest.main()

