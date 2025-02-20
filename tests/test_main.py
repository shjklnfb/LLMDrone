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
    #     planner=TaskPlanner()
    #     result=planner.plan_task(user_task="在太行山航拍")
    #     for i in result:
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
    def test_taskExecutor(self):
        from src.code.TaskExecutor import TaskExecutor
        from src.code.InterruptListener import InterruptListener
        subtask1 = SubTask(task_id=1, name="subtask1", priority=5, dep_id=[], is_interrupt=0, device="typhoon_h480_0", instructions=[
            {
            "name": "takeoff",
            "description": "无人机垂直起飞到3m"
            },
            {
            "name": "ascend",
            "description": "无人机继续垂直攀升到20m"
            },
            {
            "name": "yaw",
            "description": "无人机环绕一圈，检查周围情况"
            },
            {
            "name": "land",
            "description": "无人机降落到原地"
            },
        ])
        subtask2 = SubTask(
            task_id=2, name="subtask2", priority=2, dep_id=[1], is_interrupt=0, device="typhoon_h480_0", instructions=[
            {
                "name": "takeoff",
                "description": "无人机垂直起飞"
            },
            {
                "name": "move_forward",
                "description": "无人机向前飞行"
            },
            {
                "name": "hover",
                "description": "无人机在当前位置悬停"
            },
            {
                "name": "land",
                "description": "无人机从当前位置降落"
            },
            ]
        )
        subtask3 = SubTask(
            task_id=3, name="subtask3", priority=3, dep_id=[2], is_interrupt=0, device="typhoon_h480_0", instructions=[
            {
                "name": "takeoff",
                "description": "无人机垂直起飞"
            },
            {
                "name": "move_left",
                "description": "无人机向左飞行"
            },
            {
                "name": "hover",
                "description": "无人机在当前位置悬停"
            },
            {
                "name": "land",
                "description": "无人机从当前位置降落"
            },
            ]
        )
        subtask4 = SubTask(
            task_id=4, name="subtask4", priority=4, dep_id=[], is_interrupt=0, device="typhoon_h480_1", instructions=[
            {
                "name": "takeoff",
                "description": "无人机垂直起飞"
            },
            {
                "name": "move_right",
                "description": "无人机向右飞行"
            },
            {
                "name": "hover",
                "description": "无人机在当前位置悬停"
            },
            {
                "name": "land",
                "description": "无人机从当前位置降落"
            },
            ]
        )
        subtask5 = SubTask(
            task_id=5, name="subtask5", priority=4, dep_id=[], is_interrupt=0, device="typhoon_h480_1", instructions=[
            {
                "name": "takeoff",
                "description": "无人机垂直起飞"
            },
            {
                "name": "move_backward",
                "description": "无人机向后飞行"
            },
            {
                "name": "hover",
                "description": "无人机在当前位置悬停"
            },
            {
                "name": "land",
                "description": "无人机从当前位置降落"
            },
            ]
        )

        task_executor = TaskExecutor()
        task_executor.add_subtask(subtask1)
        task_executor.add_subtask(subtask2)
        task_executor.add_subtask(subtask3)
        task_executor.add_subtask(subtask4)
        task_executor.add_subtask(subtask5)

        # interrupt_listener1 = InterruptListener(task_executor, 5000)

        # interrupt_listener1.start()

        task_executor.execute()
        rospy.spin()  # 保持ROS节点运行

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
    
if __name__ == '__main__':
    unittest.main()

