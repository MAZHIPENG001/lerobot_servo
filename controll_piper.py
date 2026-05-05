import  time

from device.robot import robot_piper
from device.servo import Servo

'''
bash find_all_can_port.sh

bash can_activate.sh can0 1000000 "1-9.4:1.0"
bash can_activate.sh can1 1000000 "1-9.1:1.0"
'''
def main():
    piper_l = robot_piper('can0')
    piper_l.ConnectPort()
    piper_l.EnableArm()

    piper_r = robot_piper('can1')
    piper_r.ConnectPort()
    piper_r.EnableArm()

    # 红色
    in_min_red = [2190, 870, 770, 800, 1950, 1900, 800]
    in_max_red = [790, 2240, 1850, 2000, 2400, 1000, 500]
    # 橙色
    in_min_orange = [2180, 1059, 880, 900, 2000, 1980, 1580]
    in_max_orange = [780, 2321, 2000, 1900, 2500, 1020, 1280]
    servo_l = Servo(port='/dev/ttyUSB0', in_min=in_min_orange, in_max=in_max_orange)
    servo_r = Servo(port='/dev/ttyUSB1', in_min=in_min_red, in_max=in_max_red)

    servo_l.start_auto_read()
    servo_r.start_auto_read()
    print("⏳ 等待初始数据填充...")
    time.sleep(0.5)

    while True:
        servo_l_joints = servo_l.get_latest_angles()
        servo_r_joints = servo_r.get_latest_angles()
        if None in servo_l_joints or None in servo_r_joints:
            print("⚠️ 捕获到不完整数据帧，跳过本次循环...")
            time.sleep(0.01)
            continue
        print(f"\33[93m servo_l_joints:{servo_l_joints}\33[0m")
        print(f"\33[94m servo_r_joints:{servo_r_joints}\33[0m")
        # print(f"\33[92m piper_joints:{piper_joints}\33[0m")
        *map_joints_l, map_gripper_l = servo_l.map_angle_piper(val=servo_l_joints)
        *map_joints_r, map_gripper_r = servo_r.map_angle_piper(val=servo_r_joints)

        piper_l.move_a(*map_joints_l)
        piper_l.gripper(map_gripper_l)
        piper_r.move_a(*map_joints_r)
        piper_r.gripper(map_gripper_r)
        time.sleep(0.5)

if __name__ == '__main__':
    main()
