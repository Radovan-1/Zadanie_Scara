import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time
# Inicializácia PyBullet (GUI)
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
# Načítanie roviny
plane = p.loadURDF("plane.urdf")
# Načítanie robota
robot = p.loadURDF("reno.urdf.xml", basePosition=[0, 0, 0], useFixedBase=True)
# Výpis kĺbov a zistenie indexu koncového článku (lk4)
end_effector_index = -1
for i in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, i)
    joint_name = joint_info[1].decode('utf-8')
    link_name = joint_info[12].decode('utf-8')
    print(f"{i}: joint={joint_name} -> link={link_name}")
    if link_name == "lk4":
        end_effector_index = i
        print(f"✅ Koncový článok (lk4) má index: {end_effector_index}")
if end_effector_index == -1:
    raise Exception("❌ Nenašiel som koncový článok 'lk4'!")
# Indexy kĺbov podľa URDF
joint_indices = [0, 1, 2]  # q1, q2, q3 - rotačné
z_joint_index = 3          # q4 - prismatický
# Rozsahy pohybov
theta_vals = np.linspace(-np.pi/2, np.pi/2, 15)
z_vals = np.linspace(-0.05, 0.05, 5)
# Zber pozícií TCP
workspace_points = []
for t1 in theta_vals:
    for t2 in theta_vals:
        for t3 in theta_vals:
            for z in z_vals:
                # Nastavenie stavov kĺbov
                p.resetJointState(robot, joint_indices[0], t1)
                p.resetJointState(robot, joint_indices[1], t2)
                p.resetJointState(robot, joint_indices[2], t3)
                p.resetJointState(robot, z_joint_index, z)
                # Získanie pozície koncového článku
                state = p.getLinkState(robot, end_effector_index)
                if state:
                    pos = state[0]
                    workspace_points.append(pos)
# Vykreslenie bodov v PyBullete
if workspace_points:
    p.addUserDebugPoints(
        workspace_points,
        pointColorsRGB=[[1, 0, 0]] * len(workspace_points),
        pointSize=4,
        lifeTime=0
    )
# Konvertuj body na numpy array
points = np.array(workspace_points)
# ===== VYKRESLENIE cez matplotlib =====
# Pohľad zhora (X-Y)
plt.figure(figsize=(6, 6))
plt.scatter(points[:, 0], points[:, 1], s=2, c='red')
plt.title("Pracovný priestor SCARA – Pohľad zhora (X vs Y)")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis('equal')
plt.grid(True)
plt.show()
# Bežná PyBullet simulácia, ak chceš nechať okno otvorené
while True:
    p.stepSimulation()
    time.sleep(1. / 240)