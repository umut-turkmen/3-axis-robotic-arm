import pybullet as p
import pybullet_data
import time
import numpy as np
import random

# PyBullet simülasyon ortamını başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Yerçekimi ve zemin ekle
p.setGravity(0, 0, -9.81)
plane_id = p.loadURDF("plane.urdf")

# Robot kolunu ve küpü oluştur
robot_arm = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
cube_start_pos = [random.uniform(0.4, 0.6), random.uniform(-0.2, 0.2), 0.1]
cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
cube_id = p.loadURDF("cube.urdf", cube_start_pos, cube_start_orientation, globalScaling=0.2)

# Hedef pozisyon (10 cm kuzeybatı ve yukarı)
target_offset = [-0.1, 0.1, 0.05]  # 10 cm kuzeybatı ve 5 cm yukarı

def move_arm_to_position(position, gripper_open=True):
    joint_positions = p.calculateInverseKinematics(robot_arm, 6, position)
    for i in range(len(joint_positions)):
        p.setJointMotorControl2(robot_arm, i, p.POSITION_CONTROL, joint_positions[i])
    grip_position = 0.3 if gripper_open else 0.1
    p.setJointMotorControl2(robot_arm, 6, p.POSITION_CONTROL, grip_position)

# Ana döngü
grasped = False
moved = False
for i in range(2000):
    # Küpün mevcut pozisyonunu al
    cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
    
    if not grasped:
        # Küpe yaklaş ve kavra
        approach_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.1]
        move_arm_to_position(approach_pos, gripper_open=True)
        
        # Yakınlaşıldığında gripper kapat ve küpü kavra
        distance_to_cube = np.linalg.norm(np.array(approach_pos) - np.array(cube_pos))
        if distance_to_cube < 0.05:
            move_arm_to_position(cube_pos, gripper_open=False)  # Küpü kavra
            grasped = True
            time.sleep(0.5)  # Kavrama sonrası kısa bir bekleme
    
    elif grasped and not moved:
        # Kavrama sonrası küpü yukarı kaldır
        lifted_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.15]
        move_arm_to_position(lifted_pos, gripper_open=False)
        time.sleep(0.5)
        
        # Hedef pozisyona taşın
        target_pos = [
            cube_start_pos[0] + target_offset[0],
            cube_start_pos[1] + target_offset[1],
            cube_start_pos[2] + target_offset[2]
        ]
        move_arm_to_position(target_pos, gripper_open=False)
        moved = True  # Taşıma işlemi tamamlandı

    elif moved:
        # Küpü yeni pozisyona bırak
        move_arm_to_position(target_pos, gripper_open=True)  # Gripper aç
        grasped = False  # Döngüyü yeniden başlat
        moved = False

    p.stepSimulation()
    time.sleep(1./240.)
    
p.disconnect()
