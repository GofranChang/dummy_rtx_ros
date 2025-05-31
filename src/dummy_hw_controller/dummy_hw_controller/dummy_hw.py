#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
import threading
import math
import serial
import time

class DummyCommand:
    def __init__(self):
        pass

    def stop(self):
        return '!STOP'

    def start(self):
        return '!START'

    def home(self):
        return '!HOME'

    def calibrate(self):
        return '!CALIBRATION'

    def reset(self):
        return '!RESET'

    def disable(self):
        return '!DISABLE'
    
    def get_j_pose(self):
        return '#GETJPOS'
    
    def move_j(self, j0, j1, j2, j3, j4, j5, speed):
        return f'&{j0},{j1},{j2},{j3},{j4},{j5},{speed}'

    def get_l_pose(self):
        return '#GETLPOS'
    
    def move_l(self, x, y, z, a=0, b=0, c=0, speed=100):
        return f'@{x},{y},{z},{a},{b},{c},{speed}'
    
    def set_p(self, node, p):
        return f'#SET_DEC_KP {node} {p}'

    def set_i(self, node, i):
        return f'#SET_DEC_KI {node} {i}'

    def set_d(self, node, d):
        return f'#SET_DEC_KD {node} {d}'
    
    def reboot(self):
        return '#REBOOT'
    
    def set_cmd_mode(self, mode):
        return f'#CMDMODE {mode}'

class DummyHardware:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.zero_joint_angle = [0, -73, 180, 0, 0, 0]
        self.joint_dir = [1, 1, -1, -1, 1, 1]

        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.lock = threading.Lock()  # 保证多线程/模块调用时串口操作安全
        self.serial.flushInput()

        self.command_helper = DummyCommand()

        self.send_command(self.command_helper.start())
        # time.sleep(3)
        # self.send_command(self.command_helper.home())

    def send_command(self, command: str) -> str:
        if not command.endswith('\r\n'):
            command += '\n'

        with self.lock:
            self.serial.write(command.encode('ascii'))

            response = self.serial.readline().decode(errors='ignore').strip()
            return response

        return ''

    def close():
        self.serial.close()

    def set_joint_location(self, radian_location):
        degree_location = [0.0] * 6

        for i in range (0, 6):
            # degree_location[i] = math.degrees(radian_location[i]) + (self.joint_dir[i] * self.zero_joint_angle[i])
            degree_location[i] = self.zero_joint_angle[i] + (self.joint_dir[i] * math.degrees(radian_location[i]))

        cmd = self.command_helper.move_j(degree_location[0], 
                                         degree_location[1], 
                                         degree_location[2], 
                                         degree_location[3], 
                                         degree_location[4], 
                                         degree_location[5],
                                         100
                                         )
        print(f'>>> {cmd}')
        ack = self.send_command(cmd)
        print(f'<<< {ack}')


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        
        # 机械臂关节名称 - 必须与URDF一致
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        
        # # 创建Action客户端 - 连接硬件控制器
        # self.hw_client = ActionClient(
        #     self, 
        #     FollowJointTrajectory, 
        #     '/arm_trajectory_controller/follow_joint_trajectory',
        #     callback_group=ReentrantCallbackGroup()
        # )
        
        # # 创建Action服务器 - 接收来自MoveIt的执行请求
        # self.execute_server = ActionServer(
        #     self,
        #     ExecuteTrajectory,
        #     '/execute_trajectory',
        #     self.execute_callback,
        #     callback_group=ReentrantCallbackGroup()
        # )
        
        # 订阅关节状态 - 用于监控
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 当前关节状态
        self.current_joint_positions = [0.0] * len(self.joint_names)

        self.dummy_robot = DummyHardware(port='/dev/ttyACM0')
        
        # # 等待硬件控制器
        # self.get_logger().info("等待硬件控制器...")
        # if not self.hw_client.wait_for_server(timeout_sec=10.0):
        #     self.get_logger().error("硬件控制器连接超时！")
        #     self.get_logger().info("请检查: ros2 action list")
        #     self.get_logger().info("确保存在: /joint_trajectory_controller/follow_joint_trajectory")
        # else:
        #     self.get_logger().info("硬件控制器已连接")
        
        self.get_logger().info("通信节点已就绪，等待RVIZ执行命令...")

    def joint_state_callback(self, msg):
        """更新当前关节状态"""
        current_joint_positions = [0.0] * 6

        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                current_joint_positions[idx] = msg.position[i]

        if current_joint_positions == self.current_joint_positions:
            return

        self.current_joint_positions = current_joint_positions
        self.dummy_robot.set_joint_location(radian_location = self.current_joint_positions)

    async def execute_callback(self, goal_handle):
        """处理来自MoveIt的执行请求"""
        self.get_logger().info("收到轨迹执行请求")
        
        # 从MoveIt获取轨迹
        moveit_trajectory = goal_handle.request.trajectory
        
        # 转换为硬件控制器需要的格式
        hw_goal = FollowJointTrajectory.Goal()
        hw_goal.trajectory = self.convert_trajectory(moveit_trajectory)
        
        # 发送给硬件控制器
        self.get_logger().info("发送轨迹到硬件...")
        hw_future = self.hw_client.send_goal_async(hw_goal)
        await hw_future
        hw_goal_handle = hw_future.result()
        
        if not hw_goal_handle.accepted:
            self.get_logger().error("硬件控制器拒绝轨迹")
            goal_handle.abort()
            return ExecuteTrajectory.Result(result_code=ExecuteTrajectory.Result.TRAJECTORY_ERROR)
        
        # 等待硬件执行完成
        self.get_logger().info("轨迹已接受，等待执行完成...")
        result_future = hw_goal_handle.get_result_async()
        await result_future
        hw_result = result_future.result().result
        
        # 处理执行结果
        if hw_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("硬件执行成功")
            goal_handle.succeed()
            return ExecuteTrajectory.Result(result_code=ExecuteTrajectory.Result.SUCCESS)
        else:
            self.get_logger().error(f"硬件执行失败，错误码: {hw_result.error_code}")
            goal_handle.abort()
            return ExecuteTrajectory.Result(result_code=ExecuteTrajectory.Result.TRAJECTORY_ERROR)

    def convert_trajectory(self, moveit_traj):
        """将MoveIt轨迹转换为FollowJointTrajectory格式"""
        hw_traj = moveit_traj.joint_trajectory
        
        # 确保关节顺序正确
        if list(hw_traj.joint_names) != self.joint_names:
            self.get_logger().warn("关节名称不匹配，重新排序...")
            # 创建映射关系
            joint_map = {name: i for i, name in enumerate(self.joint_names)}
            
            # 重新排序所有轨迹点
            for point in hw_traj.points:
                if len(point.positions) == len(self.joint_names):
                    # 重新排序位置
                    point.positions = [point.positions[joint_map[name]] for name in self.joint_names]
                    
                    # 重新排序速度（如果有）
                    if point.velocities:
                        point.velocities = [point.velocities[joint_map[name]] for name in self.joint_names]
                    
                    # 重新排序加速度（如果有）
                    if point.accelerations:
                        point.accelerations = [point.accelerations[joint_map[name]] for name in self.joint_names]
            
            # 更新关节名称
            hw_traj.joint_names = self.joint_names
        
        return hw_traj

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    bridge = HardwareBridge()
    
    # 使用多线程执行器处理并发请求
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()