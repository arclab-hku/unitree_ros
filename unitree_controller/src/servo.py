#!/usr/bin/env python3
import rospy
import numpy as np
from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import LowCmd
from unitree_legged_msgs.msg import LowState
from unitree_legged_msgs.msg import MotorState

import rospy
from std_msgs.msg import String

class ServoStand():
    def __init__(self):
        robot_name = "go1"
        # servo publisher
        self.servo_pub = []
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/FR_hip_controller/command", MotorCmd, queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/FR_thigh_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/FR_calf_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/FL_hip_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/FL_thigh_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/FL_calf_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/RR_hip_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/RR_thigh_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/RR_calf_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/RL_hip_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/RL_thigh_controller/command", MotorCmd,  queue_size=1) )
        self.servo_pub.append( rospy.Publisher("/" + robot_name + "_gazebo/RL_calf_controller/command", MotorCmd,  queue_size=1) )

        self.lowCmd = LowCmd()
        self.lowState = LowState()
        self.lowState_pub = rospy.Publisher("/" + robot_name + "_gazebo/lowState/state", LowState,  queue_size=1)

        # servo subscriber
        self.servo_sub = []
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/FR_hip_controller/state", MotorState, self.FRhipCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/FR_thigh_controller/state", MotorState, self.FRthighCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/FR_calf_controller/state", MotorState, self.FRcalfCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/FL_hip_controller/state", MotorState, self.FLhipCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/FL_thigh_controller/state", MotorState, self.FLthighCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/FL_calf_controller/state", MotorState, self.FLcalfCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/RR_hip_controller/state", MotorState, self.RRhipCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/RR_thigh_controller/state", MotorState, self.RRthighCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/RR_calf_controller/state", MotorState, self.RRcalfCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/RL_hip_controller/state", MotorState, self.RLhipCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/RL_thigh_controller/state", MotorState, self.RLthighCallback, queue_size=1) )
        self.servo_sub.append( rospy.Subscriber("/" + robot_name + "_gazebo/RL_thigh_controller/state", MotorState, self.RLcalfCallback, queue_size=1) )


    def FRhipCallback(self, msg):
        self.lowState.motorState[0].mode = msg.mode
        self.lowState.motorState[0].q = msg.q
        self.lowState.motorState[0].dq = msg.dq
        self.lowState.motorState[0].tauEst = msg.tauEst

    def FRthighCallback(self, msg):
        self.lowState.motorState[1].mode = msg.mode
        self.lowState.motorState[1].q = msg.q
        self.lowState.motorState[1].dq = msg.dq
        self.lowState.motorState[1].tauEst = msg.tauEst

    def FRcalfCallback(self, msg):
        self.lowState.motorState[2].mode = msg.mode
        self.lowState.motorState[2].q = msg.q
        self.lowState.motorState[2].dq = msg.dq
        self.lowState.motorState[2].tauEst = msg.tauEst

    def FLhipCallback(self, msg):
        self.lowState.motorState[3].mode = msg.mode
        self.lowState.motorState[3].q = msg.q
        self.lowState.motorState[3].dq = msg.dq
        self.lowState.motorState[3].tauEst = msg.tauEst

    def FLthighCallback(self, msg):
        self.lowState.motorState[4].mode = msg.mode
        self.lowState.motorState[4].q = msg.q
        self.lowState.motorState[4].dq = msg.dq
        self.lowState.motorState[4].tauEst = msg.tauEst

    def FLcalfCallback(self, msg):
        self.lowState.motorState[5].mode = msg.mode
        self.lowState.motorState[5].q = msg.q
        self.lowState.motorState[5].dq = msg.dq
        self.lowState.motorState[5].tauEst = msg.tauEst

    def RRhipCallback(self, msg):
        self.lowState.motorState[6].mode = msg.mode
        self.lowState.motorState[6].q = msg.q
        self.lowState.motorState[6].dq = msg.dq
        self.lowState.motorState[6].tauEst = msg.tauEst

    def RRthighCallback(self, msg):
        self.lowState.motorState[7].mode = msg.mode
        self.lowState.motorState[7].q = msg.q
        self.lowState.motorState[7].dq = msg.dq
        self.lowState.motorState[7].tauEst = msg.tauEst

    def RRcalfCallback(self, msg):
        self.lowState.motorState[8].mode = msg.mode
        self.lowState.motorState[8].q = msg.q
        self.lowState.motorState[8].dq = msg.dq
        self.lowState.motorState[8].tauEst = msg.tauEst

    def RLhipCallback(self, msg):
        self.lowState.motorState[9].mode = msg.mode
        self.lowState.motorState[9].q = msg.q
        self.lowState.motorState[9].dq = msg.dq
        self.lowState.motorState[9].tauEst = msg.tauEst

    def RLthighCallback(self, msg):
        self.lowState.motorState[10].mode = msg.mode
        self.lowState.motorState[10].q = msg.q
        self.lowState.motorState[10].dq = msg.dq
        self.lowState.motorState[10].tauEst = msg.tauEst

    def RLcalfCallback(self, msg):
        self.lowState.motorState[11].mode = msg.mode
        self.lowState.motorState[11].q = msg.q
        self.lowState.motorState[11].dq = msg.dq
        self.lowState.motorState[11].tauEst = msg.tauEst


    def paramInit(self):
        for i in range(4):
            self.lowCmd.motorCmd[i * 3 + 0].mode = 0x0A
            self.lowCmd.motorCmd[i * 3 + 0].Kp = 70
            self.lowCmd.motorCmd[i * 3 + 0].dq = 0
            self.lowCmd.motorCmd[i * 3 + 0].Kd = 3
            self.lowCmd.motorCmd[i * 3 + 0].tau = 0
            self.lowCmd.motorCmd[i * 3 + 1].mode = 0x0A
            self.lowCmd.motorCmd[i * 3 + 1].Kp = 180
            self.lowCmd.motorCmd[i * 3 + 1].dq = 0
            self.lowCmd.motorCmd[i * 3 + 1].Kd = 8
            self.lowCmd.motorCmd[i * 3 + 1].tau = 0
            self.lowCmd.motorCmd[i * 3 + 2].mode = 0x0A
            self.lowCmd.motorCmd[i * 3 + 2].Kp = 300
            self.lowCmd.motorCmd[i * 3 + 2].dq = 0
            self.lowCmd.motorCmd[i * 3 + 2].Kd = 15
            self.lowCmd.motorCmd[i * 3 + 2].tau = 0

        for i in range(12):
            self.lowCmd.motorCmd[i].q = self.lowState.motorState[i].q


    def sendServoCmd(self):
        for m in range(12):
            self.servo_pub[m].publish(self.lowCmd.motorCmd[m])


    def stand(self, targetPos, duration, i):
        pos, lastPos = np.zeros(12), np.zeros(12)
        for j in range(12): lastPos[j] = self.lowState.motorState[j].q
        print(i)
        percent = i / duration
        for j in range(12):
            self.lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent
        self.sendServoCmd()


    def run(self):

        self.paramInit()
        targetPos = [0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                     0.0, 0.67, -1.3, -0.0, 0.67, -1.3]
        duration = 2000

        i = 0
        rate = rospy.Rate(1000)  # 1000hz
        while not rospy.is_shutdown():
            self.lowState_pub.publish(self.lowState)

            self.stand(targetPos, duration, i) # only increase one step
            i += 1

            if i >= duration:
                break

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("unitree_gazebo_servo")
    servo_stand = ServoStand()
    servo_stand.run()
    rospy.spin()


# # ---------- second method ----------
#     def run(self):
#
#         self.paramInit()
#         targetPos = [0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
#                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3]
#         duration = 2000
#
#         i = 0
#         rate = rospy.Rate(1000)  # 1000hz
#         while (1): # or while not rospy.is_shutdown():
#             self.lowState_pub.publish(self.lowState)
#
#             self.stand(targetPos, duration, i) # only increase one step
#             i += 1
#
#             if i >= duration:
#                 break
#
#             rate.sleep()
#
# if __name__ == '__main__':
#     rospy.init_node("unitree_gazebo_servo")
#     servo_stand = ServoStand()
#     servo_stand.run()
#     # rospy.spin()