import pybullet as p
import pybullet_data
import time

p.connect(p.GUI) # 시뮬레이터 GUI 형성
p.setGravity(0,0,-9.8) # 중력 적용

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # URDF 파일 경로 설정
planeId = p.loadURDF("plane.urdf")

ROBOT = p.loadURDF('/home/zenbook/pybullet_ros/Lite6/lite6.urdf', basePosition = [0,0,0])
#urdf 파일 업로드
while True:
    p.stepSimulation()
    time.sleep(1/240)
p.disconnect()