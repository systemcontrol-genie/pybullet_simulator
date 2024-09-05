import pybullet as p
import pybullet_data
import time

class robot_upload:
    def __init__(self):
        # simulator gui 설정, 물리 중력 설정 
        p.connect(p.GUI)
        p.setGravity(0,0,-9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF('plane.urdf')
        robot = p.loadURDF('/home/zenbook/pybullet_ros/Lite6/lite6.urdf', basePosition  = [0,0,0])
    def main(self):
        while True:   
            p.stepSimulation()
            time.sleep(1/240)
        p.disconnect()


if __name__ =="__main__":
    control_arm = robot_upload()
    control_arm.main()
