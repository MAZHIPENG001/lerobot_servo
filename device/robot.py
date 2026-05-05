from piper_sdk import *

import openarm_can as oa
import numpy as np
from processer.transfer import euler_to_quaternion,transform_matrix

class robot_piper(C_PiperInterface_V2):
    """
    _a/_p:关节控制/末端控制
    gripper:夹爪
    read:信息读取
    """
    def __init__(self,can='can0'):
        super().__init__(can)
        # 初始位置
        self.init = [0, 0.4, -0.7, 0.0, 1.1, 0.0]
        factor = 180 * 1000 / 3.14
        self.joint_0_init = round(self.init[0] * factor)
        self.joint_1_init = round(self.init[1] * factor)
        self.joint_2_init = round(self.init[2] * factor)
        self.joint_3_init = round(self.init[3] * factor)
        self.joint_4_init = round(self.init[4] * factor)
        self.joint_5_init = round(self.init[5] * factor)
        self.T_flange_tcp = np.array([
                            [1, 0, 0, 0],#大前小后
                            [0, 1, 0, 0],#大右
                            [0, 0, 1, -0.10],#大高小矮
                            [0, 0, 0, 1]])

    def start(self):
        self.ConnectPort()
        self.EnableArm()

    def enable_arm(self):#使能
        self.EnableArm()
    def disable_arm(self):#失能
        self.DisableArm()

    def move_a(self,j1,j2,j3,j4,j5,j6):
        self.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.JointCtrl(j1,j2,j3,j4,j5,j6)
    def move_p(self,X_target,Y_target,Z_target,RX_target, RY_target, RZ_target):
        self.MotionCtrl_2(0x01, 0x00, 20, 0x00)
        self.EndPoseCtrl(round(X_target), round(Y_target), round(Z_target),
                          RX_target, RY_target, RZ_target)
    def move_zero_a(self):#关节控制移动到零点
        self.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        self.JointCtrl(0, 0, 0, 0, 0, 0)
    def move_init_p(self):#末端控制移动到初始位置
        self.MotionCtrl_2(0x01, 0x00, 50, 0x00)
        self.EndPoseCtrl(40000, 0, 333000, 180000, 58000, 180000)
    def move_init_a(self):#关节控制移动到初始位置
        self.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        self.JointCtrl(self.joint_0_init, self.joint_1_init, self.joint_2_init,
                             self.joint_3_init, self.joint_4_init, self.joint_5_init)

    def gripper(self,width):
        self.GripperCtrl(width,1000,0x01,0)
    def gripper_on(self):
        self.gripper(70000)
    def gripper_off(self):
        self.gripper(0)

    def grasp_end(self):
        '''收尾工作'''
        # 夹取物体
        self.gripper_off()
        time.sleep(1)
        # 复位
        # self.move_a(0, 95000, -80000, 0, 0, 0)
        # time.sleep(0.3)
        self.move_a(self.joint_0_init, self.joint_1_init, self.joint_2_init,
                    self.joint_3_init, self.joint_4_init, self.joint_5_init)
        time.sleep(1)
        # 移动到盒子上方
        self.move_a(-90000, 80000, -50000, 0, 60000, 0)
        time.sleep(1)
        # 松开夹爪
        self.gripper_on()
        time.sleep(1)
        self.move_a(self.joint_0_init, self.joint_1_init, self.joint_2_init,
                    self.joint_3_init, self.joint_4_init, self.joint_5_init)
        self.gripper_off()

    def read_end_pose(self):
        ArmEndPoseMsgs=self.GetArmEndPoseMsgs()
        return ArmEndPoseMsgs
    def read_joint(self):
        ArmJointMsgs=self.GetArmJointMsgs()
        return ArmJointMsgs
    def read_gripper(self):
        GetArmGripperMsgs=self.GetArmGripperMsgs()
        return GetArmGripperMsgs
    def cal_T0(self):
        ArmEndPoseMsgs=self.read_end_pose()
        '''输入机械臂位置信息->计算T0'''
        X = ArmEndPoseMsgs.end_pose.X_axis / 1000 / 1000
        Y = ArmEndPoseMsgs.end_pose.Y_axis / 1000 / 1000
        Z = ArmEndPoseMsgs.end_pose.Z_axis / 1000 / 1000
        RX = ArmEndPoseMsgs.end_pose.RX_axis / 180 * 3.1415926 / 1000
        RY = ArmEndPoseMsgs.end_pose.RY_axis / 180 * 3.1415926 / 1000
        RZ = ArmEndPoseMsgs.end_pose.RZ_axis / 180 * 3.1415926 / 1000

        rotation_euler = [RX, RY, RZ]
        rotation_quat = euler_to_quaternion(rotation_euler)
        # print("末端位置：", X, Y, Z, rotation_quat)

        t0 = transform_matrix([X, Y, Z], rotation_quat=rotation_quat)
        return t0

    def grasp_order(self,translations,rotation_mats_3x3,Pixel):
        # 从新排序：由近及远
        translations = np.array(translations)
        rotation_mats_3x3 = np.array(rotation_mats_3x3)
        Pixel = np.array(Pixel)
        # 顺序
        index = np.argsort(translations[:, 2])  # 第三分量索引是2
        # 按照排序索引重新排列数据
        translations = translations[index]
        rotation_mats_3x3 = rotation_mats_3x3[index]
        Pixel = Pixel[index]
        return translations, rotation_mats_3x3, Pixel

class robot_openarm():
    def __init__(self,can="can0",num_joints=7,enabel=False):
        self.arm = oa.OpenArm(can, True)
        self.num_joints=num_joints
        # Initialize arm motors
        '''
        self.motor_types=[oa.MotorType.DM4310, oa.MotorType.DM4310]
        self.send_ids = [0x01, 0x02]
        self.recv_ids = [0x11, 0x12]
        '''
        self.motor_types = oa.MotorType.DM4310
        # self.arm_motor_types = self.motor_types*self.num_joints
        self.send_ids = [i+1 for i in range(self.num_joints)]
        self.recv_ids = [0x10 + sid for sid in self.send_ids]
        self.gripper_send_id = self.num_joints + 1
        self.gripper_recv_id = self.gripper_send_id + 0x10

        # [oa.ControlMode.MIT]          MIT 模式 / 阻抗控制模式
        # [oa.ControlMode.POS_VEL]      *位置-速度模式
        # [oa.ControlMode.POS_FORCE]    位置-力矩模式
        self.control_modes_mit = oa.ControlMode.MIT
        self.control_modes_vel = oa.ControlMode.POS_VEL
        self.control_modes_force = oa.ControlMode.POS_FORCE
        # [oa.CallbackMode.IGNORE]      忽略/静默模式
        # [oa.CallbackMode.PARAM]       参数反馈模式
        # [oa.CallbackMode.STATE]       *状态反馈模式
        self.callback_modes_ignore = oa.CallbackMode.IGNORE
        self.callback_modes_param = oa.CallbackMode.PARAM
        self.callback_modes_state = oa.CallbackMode.STATE

        self.arm_motor_types=[self.motor_types]*self.num_joints
        self.gripper_types=self.motor_types
        self.arm_control_modes=[self.control_modes_vel]*self.num_joints
        self.gripper_control_mode=self.control_modes_force
        # Initialize joints & gripper
        self.arm.init_arm_motors(
            self.arm_motor_types,
            self.send_ids,
            self.recv_ids,
            self.arm_control_modes  # 如果你的 API 确实支持在这里传入控制模式列表
        )
        self.arm.init_gripper_motor(
            self.gripper_types,
            self.gripper_send_id,
            self.gripper_recv_id,
            self.gripper_control_mode
        )
        # self.arm.init_arm_motors(self.motor_types * self.num_joints,
        #                          self.send_ids,self.recv_ids,
        #                          self.control_modes_vel * self.num_joints,)
        # self.arm.init_gripper_motor(self.motor_types, self.num_joints+1,
        #                             self.num_joints+10, self.control_modes_force)

        self.gripper = self.arm.get_gripper()
        self.joints = self.arm.get_arm()

        self.arm.set_callback_mode_all(self.callback_modes_state)
        # Use high-level operations
        if enabel:
            self.arm.enable_all()
        else:
            self.arm.disable_all()
        self.arm.recv_all()

    def disable_arm(self):
        self.arm.disable_all()
    def enable_arm(self):
        self.arm.enable_all()
    def move_zero(self):
        # return to zero position
        self.arm.set_callback_mode_all(self.callback_modes_state)
        self.arm.get_arm().posvel_control_all([oa.PosVelParam(3.14 * 0.0, 0.0)])

    def open_arm(self):
        pass
    def mit_controll(self,kp=2.0, kd=0.5, aim_p=None, aim_v=0.0, aim_t=0.0):
        """需根据关节数量修改aim_p、aim_v等参数"""
        if aim_p is None:
            aim_p = [0.0] * len(self.num_joints)  # 默认为全 0
        if len(aim_p) != self.num_joints:
            raise ValueError(f"aim_p length ({len(aim_p)}) must match joint count ({self.num_joints})")
        params = [oa.MITParam(kp, kd, p, aim_v, aim_t) for p in aim_p]
        # 位置比例增益(Kp) = 2，速度微分增益(Kd) = 0.5，目标位置 = 0，目标速度 = 0，前馈力矩 = 0
        self.joints.mit_control_all(*params)

    def gripper_controll(self,position=0.0,speed=25.0,torque=1.5):
        self.gripper.set_position(position, speed_rad_s=speed, torque_pu=torque/10)
    def gripper_on(self):
        self.gripper_controll(position=3.14/2)
    def gripper_off(self):
        self.gripper_controll(position=0)

    def read_end_pose(self):
        pass
    def read_joint(self):
        self.arm.refresh_all()
        self.arm.recv_all()
        for motor in self.joints.get_motors():
            print(motor.get_position())
    def read_gripper(self):
        self.arm.refresh_all()
        self.arm.recv_all()
        for motor in self.gripper.get_motors():
            print(motor.get_position())
    def read_msg(self):
        self.arm.refresh_all()
        self.arm.recv_all()
        for i, motor in enumerate(self.joints.get_motors()):
            print(f'joint \33[92m{i}\33[0m: \33[92m{motor.get_position()}\33[0m')
        for motor in self.gripper.get_motors():
            # print(motor.get_position())
            print(f'\33[93mgripper:{motor.get_position()}\33[0m')

if __name__ == '__main__':
    from device.keyboard import KeystrokeCounter, KeyCode
    import time
    piper = robot_piper()
    piper.ConnectPort()
    while True:
        with KeystrokeCounter() as key_counter:
            try:
                while True:
                    press_events = key_counter.get_press_events()
                    for key_stroke in press_events:
                        # 按下o：归零(关节控制)
                        if key_stroke == KeyCode(char='o'):
                            print("按下o：归零(关节控制)")
                            piper.move_zero_a()
                        # 按下i: 复位(关节控制)
                        elif key_stroke == KeyCode(char='i'):
                            print("按下i:复位(关节控制)")
                            piper.move_init_p()
                        elif key_stroke == KeyCode(char='p'):
                            print("按下i:复位(末端控制)")
                            piper.move_init_p()
                        elif key_stroke == KeyCode(char='m'):
                            print("按下m:读取末端位置")
                            piper.read_end_pose()
                        elif key_stroke == KeyCode(char='e'):
                            print("按下e:使能")
                            piper.enable_arm()
                        elif key_stroke == KeyCode(char='d'):
                            print("按下d:失能")
                            piper.disable_arm()
            except KeyboardInterrupt:
                events = key_counter.get_press_events()
                print(events)