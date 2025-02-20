from enum import Enum

'''
定义需要的实体类
子任务：
| 名称        | 描述           | 范围      |
| ----------- | -------------- | --------- |
| id          | 任务ID         |           |
| name        | 任务名称       |           |
| priority    | 优先级         | 0-10      |
| depId       | 依赖任务ID     | 任务ID    |
| isInterrupt | 是否可以被抢占 | 0/1       |
| state       | 任务状态       | 1,2,3,4,5 |
| device      | 使用的无人机   |           |
基本指令：
| 指令名         | 功能                                 | 参数                         | 示例                                          |
| -------------- | ------------------------------------ | ---------------------------- | --------------------------------------------- |
| control        | 启动无人机                           |                              | python control.py namespace                   |
| command_pos    | 飞行到位置（x,y,z）                  | 坐标x,y,z                    | python command_pos.py namespace x y z         |
| command_vel    | 以某个速度（x,y,z）飞行时间t         | 三个方向的速度x,y,z，时间t   | python command_vel.py namespace x y z t       |
| command_hover  | 悬停在当前位置，持续时间t            | 悬停时间t                    | python command_hover.py namespace t           |
| command_twist  | 围绕z轴以速度rate旋转角度degree      | 旋转角度degree，旋转速度rate | python command_twist.py namespace degree rate |
| command_land   | 降落到当前位置                       |                              | python command_land.py namespace              |
| command_search | 在半径为r的圆形区域内搜索，持续时间t | 半径r，持续时间t             | python command_search.py namespace r t        |
无人机：
| 无人机类型           | 子型号                | 用途                                                         | 自重   | 翼展  | 飞行时间 | 载荷  | 最大速度 |
| -------------------- | --------------------- | ------------------------------------------------------------ | ------ | ----- | -------- | ----- | -------- |
| 四旋翼无人机         | iris_2d_lidar         | 携带2d雷达，用于2d建图，便于后续无人机无人车的路径规划与自主导航。 | 1282g  | 0.55m | 17min    | 400g  | 16m/s    |
|                      | iris_3d_gpu_lidar     | 携带3d雷达，用于3d建图，便于后续无人机无人车的路径规划与自主导航。 | 1282g  | 0.55m | 17min    | 400g  | 16m/s    |
|                      | iris_downward_camera  | 携带下侧摄像头，用于搜索、物体定位、构建地图。               | 1282g  | 0.55m | 17min    | 400g  | 16m/s    |
|                      | iris_fpv_cam          | 携带第一视角摄像头，该无人机速度更快，体积更小，能够在复杂的地形中穿梭，可以使用第一视角摄像头在较为复杂的环境中侦查，目标识别。 | 1282g  | 0.55m | 17min    | 400g  | 16m/s    |
|                      | iris_realsense_camera | 携带深度摄像头，可以在低光环境下精确地捕捉物体的轮廓、深度和形状等信息。常用于物体的识别和跟踪、自动导航、三维建模。 | 1282g  | 0.55m | 17min    | 400g  | 16m/s    |
|                      | iris_with_arm         | 携带机械臂和摄像头，能够通过摄像头进行目标识别和定位，使用机械臂对物体进行拾取和投送。 | 2350g  | 0.55m | 25min    | 2000g | 16m/s    |
| 六旋翼无人机         | typhoon_h480          | 携带云台相机，能够提供更稳定和清晰的图像，可以独立控制相机的运动，用于拍摄、测绘、目标识别。 | 1695g  | 0.70m | 25min    | 1150g | 18m/s    |
| 固定翼无人机         | plane                 | 适用于航测、巡航和长航时飞行，适用于大范围巡航和测绘任务.    | 5500g  | 3m    | 300min   | 5000g | 35m/s    |
| 垂直起降固定翼无人机 | standard_vtol         | 结合了固定翼的长航时和多旋翼的垂直起降特性，适用于复杂环境下的长航时飞行测试 | 19000g | 3.2m  | 360min   | 6500g | 28m/s    |
'''

from enum import Enum

# 定义任务状态
class TaskState(Enum):
    WAITING = 1  # 等待中
    READY = 2  # 待执行
    RUNNING = 3  # 执行中
    INTERRUPTED = 4  # 已中断
    COMPLETED = 5  # 已完成

# 子任务类
class SubTask:
    def __init__(self, task_id, name, priority, dep_id, is_interrupt, device, instructions=None, requirements=None, models=None):
        self.id = task_id
        self.name = name
        self.priority = priority
        self.dep_id = dep_id
        self.is_interrupt = is_interrupt
        self.state = TaskState.WAITING
        self.device = device
        self.instructions = instructions if instructions is not None else []
        self.requirements = requirements if requirements is not None else {}
        self.models = models if models is not None else []

    def add_instruction(self, instruction):
        self.instructions.append(instruction)

    def __lt__(self, other):
        # 比较任务的优先级，优先级高的任务应该排在前面
        return self.priority < other.priority

    def __str__(self):
        instruction_strs = []
        for instruction in self.instructions:
            instruction_strs.append(str(instruction))
        return (f"SubTask(id={self.id}, name={self.name}, priority={self.priority}, "
                f"dep_id={self.dep_id}, is_interrupt={self.is_interrupt}, state={self.state}, "
                f"device={self.device}, instructions=[{', '.join(instruction_strs)}], requirements={self.requirements})")

# 指令类
class Instruction:
    def __init__(self, command, params):
        self.command = command
        self.params = params

    def __str__(self):
        return f"Command: {self.command}, Params: {self.params}"

# 无人机类
class Drone:
    def __init__(self, drone_id, name, capabilities):
        self.id = drone_id
        self.name = name
        self.capabilities = capabilities  # 一个字典，包含无人机的能力信息。例如 荷载：10kg
        self.is_available = True

# 模型类
class Model:
    def __init__(self,name, capability):
        self.name=name # 模型名称
        # self.version = version # 模型版本
        self.capability = capability # 对于模型能力的描述