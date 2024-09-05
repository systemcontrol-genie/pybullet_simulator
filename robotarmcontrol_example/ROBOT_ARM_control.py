import pybullet as p
import pybullet_data
import time

class robot_control:
    def __init__(self):
        # 시뮬레이터 GUI 설정, 중력 설정
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 바닥과 로봇 로드
        self.planeId = p.loadURDF('plane.urdf')
        self.robot = p.loadURDF('/home/zenbook/pybullet_ros/Lite6/lite6.urdf', basePosition=[0, 0, 0])
        
        # 로봇 베이스 무게 추가 (로봇이 넘어지지 않도록)
        p.changeDynamics(self.robot, -1, mass=100)
    
    def joint_position(self, robot, joint_indices, target_positions):
        # 각 조인트에 대해 목표 위치를 설정
        for i, joint_idx in enumerate(joint_indices):
            p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_positions[i]
            )
    
    def main(self):
        while True:
            # 시뮬레이션 단계 실행240
            p.stepSimulation()
            
            # 제어할 조인트와 목표 위치 설정
            joint_indices = [0, 1, 2, 3, 4, 5]  # 로봇의 모든 조인트
            target_positions = [1.0, 0.5, 2.0, 1.2, -1.0, 0.8]  # 목표 위치
            
            # 조인트를 목표 위치로 이동
            self.joint_position(self.robot, joint_indices, target_positions)
            
            # 시뮬레이션 속도 조정
            time.sleep(1/20)

if __name__ == "__main__":
    control_arm = robot_control()
    control_arm.main()