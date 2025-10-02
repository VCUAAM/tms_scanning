import rtde_control
import rtde_receive
import numpy as np

rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")

pos = [.225,-.139,.286,2.286,-2.184,0.072]
joint_q = rtde_c.getInverseKinematics(pos)
offset = [0,0,.1,0,0,0]
# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)
current_tcp = rtde_r.getActualTCPPose()
print("Current TCP pose: ", current_tcp)
target_tcp = np.add(current_tcp,offset)
target_tcp = [float(i) for i in target_tcp]
print("Target TCP pose: ", target_tcp)
target_tcp = rtde_c.getInverseKinematics(target_tcp)
rtde_c.moveJ(target_tcp)

rtde_c.servoStop()
rtde_c.stopScript()