
# 示例：创建任务并执行
def main():
    drone1 = Drone(1, "Drone1", {"payload": "10kg"})
    drone2 = Drone(2, "Drone2", {"payload": "12kg"})

    task_executor = TaskExecutor()

    # 创建子任务
    sub_task1 = SubTask(1, "SubTask1", 1, None, False, drone1, [Instruction("command_takeoff", {"altitude": 10})])
    sub_task2 = SubTask(2, "SubTask2", 2, 1, False, drone1, [Instruction("command_pos", {"x": 5, "y": 5})])
    sub_task3 = SubTask(3, "SubTask3", 3, 1, True, drone2, [Instruction("command_takeoff", {"altitude": 15})])

    task_executor.add_sub_task(sub_task1)
    task_executor.add_sub_task(sub_task2)
    task_executor.add_sub_task(sub_task3)

    task_executor.execute()

    # 模拟外部子任务反馈
    task_executor.receive_data_feedback(1, {"data": "sample data"})


if __name__ == "__main__":
    # Initialize Task Planner and plan tasks
    task_planner = TaskPlanner()
    subtasks = task_planner.plan_task("Example Task", "Demonstration Purpose")

    # Initialize Task Distributer and add drones
    task_distributer = TaskDistributer()
    drone1 = Drone(drone_id="1", name="Drone 1", capability="basic", is_available=True)
    drone2 = Drone(drone_id="2", name="Drone 2", capability="advanced", is_available=True)
    task_distributer.add_drone(drone1)
    task_distributer.add_drone(drone2)

    # Distribute tasks to drones
    distributed_subtasks = task_distributer.distribute_tasks(subtasks)

    # Initialize Task Executor and add subtasks
    task_executor = TaskExecutor()
    task_executor.add_subtasks(distributed_subtasks)

    # Initialize Drone Executors and schedulers
    drone_executor = DroneExecutor()
    for drone in task_distributer.drones:
        scheduler = DroneScheduler(drone)
        for subtask in distributed_subtasks:
            if subtask.device == drone:
                scheduler.assign_subtask(subtask)
        drone_executor.add_scheduler(scheduler)

    # Run Task Executor
    task_executor.run()

    # Run Drone Executor
    drone_executor.run()