import pybullet as p
import pybullet_data
import time

class agv_example:
    def __init__(self):
        p.connect(p.GUI)
        p.setGravity(0,0,-9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planed = p.loadURDF("plane.urdf")
        self.car = p.loadURDF('/home/zenbook/pybullet_ros/agv_example/robot2.urdf', basePosition = [0,0,0])
        p.changeDynamics(self.car, -1, mass=100)
    
    def main(self):
        while True:
            #pos , orient = p.getBasePositionAndOrientation(self.car)
            #theat = p.getEulerFromQuaternion(orient)
            p.stepSimulation()
            time.sleep(1/24)
            
            lin_vel =0.5
            ang_vel = 0.5
            maxForce = 10
            vr = (2*lin_vel - ang_vel*0.3)/(2*0.04)
            vl = (2*lin_vel + ang_vel*0.3)/(2*0.04)

            p.setJointMotorControl2(self.car,jointIndex=1,controlMode=p.VELOCITY_CONTROL,targetVelocity=vr,
                            force=maxForce)

            p.setJointMotorControl2(self.car,jointIndex=2,controlMode=p.VELOCITY_CONTROL, targetVelocity=vl,force=maxForce)


if __name__ == "__main__":
    car_main = agv_example()
    car_main.main()