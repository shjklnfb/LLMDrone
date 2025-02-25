import threading
import time
from src.llm.StepTranslate import step_to_command
from src.llm.CheckStep import checkStep
from src.instructions.command_takeoff import command_takeoff
from src.instructions.command_pos import publish_command as command_pos
from src.instructions.command_vel import send_velocity_command as command_vel
from src.instructions.command_twist import send_rotate_command as command_twist
from src.instructions.command_twist import send_rotate_command as command_random
from src.instructions.command_hover import send_emergency_hover_command as command_hover
from src.instructions.command_land import send_land_command as command_land

# 无人机执行器类
# 将步骤翻译为无人机指令并执行，判断步骤的执行情况，根据步骤的执行情况发布新的指令
class DroneExecutor(threading.Thread):
    def __init__(self, drone_id, step, shared_data):
        super().__init__()
        self.drone_id = drone_id
        self.step = step
        self.shared_data = shared_data

    def run(self):
        # 将步骤翻译为无人机指令并执行
        print(f"Executing step {self.step} on drone {self.drone_id}")
        command = self.translate_to_command(self.step)
        self.execute_command(command)
        # current_state = self.shared_data.get(self.drone_id, {})
        # checkStep(current_state, self.step)
        time.sleep(1)

    # 使用LLM将步骤翻译为无人机指令 
    def translate_to_command(self, step):
        command = step_to_command(step)
        return command

    # 执行无人机指令
    def execute_command(self, command):
        # time.sleep(10)
        print(f"Drone {self.drone_id} executing command: {command}")
        # 根据传来的instruction和drone进行执行
        for c in command:
            if c["command"] == "command_takeoff":
                threading.Thread(target=command_takeoff, args=(self.drone_id,)).start()
            elif c["command"] == "command_pos":
                command_pos(self.drone_id, c["params"]['x'], c["params"]['y'], c["params"]['z'])
            elif c["command"] == "command_vel":
                command_vel(self.drone_id, c["params"]['x'], c["params"]['y'], c["params"]['z'], c["params"]['t'])
            elif c["command"] == "command_twist":
                command_twist(self.drone_id, c["params"]['yaw_rate'], c["params"]['duration'])
            elif c["command"] == "command_random":
                command_random(self.drone_id, c["params"]['radius'], c["params"]['duration'])
            elif c["command"] == "command_hover":
                time.sleep(c["params"]['duration'])
            elif c["command"] == "command_land":
                command_land(self.drone_id)

    
    # 检查无人机状态是否满足步骤的目标
    def check_step_completion(self):
        attempts = 0
        max_attempts = 3

        while attempts < max_attempts:
            # 获取当前无人机状态
            current_state = self.shared_data.get(self.drone_id, {})
            
            if checkstep(current_state, self.step):
                print(f"Drone {self.drone_id} has met all the targets for the step.")
                return True
            
            # 如果没有满足目标，使用大模型生成新的指令并执行
            print(f"Drone {self.drone_id} did not meet the targets, generating new commands.")
            command = newcommand()
            self.execute_command(command)
            time.sleep(1)
            attempts += 1
        
        print(f"Drone {self.drone_id} failed to meet the targets after {max_attempts} attempts.")
        return False
