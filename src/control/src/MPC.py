from rockit import *
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan
import rospy
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import math

class MPC:

    def __init__(self):

        self.ocp = Ocp(T=5)
        self.setStates()
        self.setEquations()
        self.setObjectives()
        self.setConstraint()

    def setConstraint(self):
        self.ocp.subject_to(-0.5 <= (self.left <= 0.5 ))
        self.ocp.subject_to(-0.5 <= (self.right <= 0.5 ))



    def setStates(self):
        # Set states

        self.x = self.ocp.state()
        self.y = self.ocp.state()
        self.vel = self.ocp.state()
        self.theta = self.ocp.state()
        self.thetadot = self.ocp.state()
        self.totDist = self.ocp.state()

        self.left = self.ocp.control()
        self.right= self.ocp.control()

    def setEquations(self):
        # Set equations

        self.ocp.set_der(self.x, self.vel*cos(self.theta))
        self.ocp.set_der(self.y, self.vel*sin(self.theta))
        self.ocp.set_der(self.vel, 0.5*self.left + 0.5*self.right)
        self.ocp.set_der(self.theta, self.thetadot)
        self.ocp.set_der(self.thetadot, self.right - self.left)
        self.ocp.set_der(self.totDist, self.vel)

    def setObjectives(self):
        # Add objectives

        self.ocp.add_objective(self.ocp.integral(self.totDist**2))



    def setGoal(self,endX,endY,endVel,endTheta,endThetadot):
        # Final constraint
        self.ocp.subject_to(self.ocp.at_tf(self.x)==endX)
        self.ocp.subject_to(self.ocp.at_tf(self.y)==endY)
        self.ocp.subject_to(self.ocp.at_tf(self.vel) == endVel)
        self.ocp.subject_to(self.ocp.at_tf(self.theta) == endTheta)
        self.ocp.subject_to(self.ocp.at_tf(self.thetadot) == endThetadot)

    def startState(self,x,y,vel,theta,thetadot):
        # Initial constraints

        self.ocp.subject_to(self.ocp.at_t0(self.x) == x)
        self.ocp.subject_to(self.ocp.at_t0(self.y) == y)
        self.ocp.subject_to(self.ocp.at_t0(self.vel) == vel)
        self.ocp.subject_to(self.ocp.at_t0(self.theta) == theta)
        self.ocp.subject_to(self.ocp.at_t0(self.thetadot) == thetadot)

    def solver(self):


        self.ocp.solver('ipopt')

        method = MultipleShooting(N=40, M=1, intg='rk')
        # method = DirectCollocation(N=40, M=8)
        self.ocp.method(method)

        self.sol = self.ocp.solve()

        # ocp.spy()
        # plt.show()
    
    def display(self):

        self.tsa, self.x1a = self.sol.sample(self.x, grid='control')
        self.tsb, self.y1a = self.sol.sample(self.y, grid='control')

        plt.plot(self.tsa, self.x1a, '-')
        plt.plot(self.tsb, self.y1a, 'o')
        # plt.plot(tsb, x1b, 'o')
        # plt.plot(tsc, x1c, '.')
        plt.show()
    
    def getControl(self):
        self.tleft, self.leftControl = self.sol.sample(self.left, grid='control')
        self.tright, self.rightControl = self.sol.sample(self.right, grid='control')
        print(len(self.tleft))
        return self.tleft, self.leftControl, self.tright, self.rightControl


class rosClass:

    def __init__(self):

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.getTelemetries, queue_size=1)

        self.control = rospy.Publisher("/rover/set_control_Input", Float32MultiArray, queue_size=10)
    
        self.X = 0
        self.Y = 0
        self.Vel = 0
        self.Theta = 0
        self.Thetadot = 0

    def getTelemetries(self,data):

        self.X = data.pose[1].position.x
        self.Y = data.pose[1].position.y
        self.Theta = tf.transformations.euler_from_quaternion(
            [
                data.pose[1].orientation.x,
                data.pose[1].orientation.y,
                data.pose[1].orientation.z,
                data.pose[1].orientation.w,
            ]
        )[1]
        self.Thetadot = data.twist[1].angular.y
        self.meh = Vector3(1,1,1)
        self.Vel = math.sqrt(data.twist[1].linear.x**2 + data.twist[1].linear.y**2 + data.twist[1].linear.z**2)
    def returnTelemetries(self):
        return self.X,self.Y, self.Vel, self.Theta, self.Thetadot

    def publishControl(self,controlIn):
        out = Float32MultiArray()
        data = []
        for x in range(20):
            data.append(controlIn[0][x])
            data.append(controlIn[1][x])
            data.append(controlIn[2][x])
            data.append(controlIn[3][x])
        out.data = data

        self.control.publish(out)


if __name__ == "__main__":
    rospy.init_node("MPC")
    ros = rosClass()
    while True:
        mpc = MPC()
        mpc.setGoal(1,1,0,0,0)
        X, Y, Vel, Theta, Thetadot = ros.returnTelemetries()
        mpc.startState(X, Y, Vel, Theta, Thetadot)
        mpc.solver()
        ros.publishControl(mpc.getControl())
        # mpc.display()
    rospy.spin()
