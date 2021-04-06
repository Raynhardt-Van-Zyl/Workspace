import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Vector3
import numpy as np


class Lidar:
    def __init__(self):
        self.scan = LaserScan()
        self.scanOut = LaserScan()
        self.lidar = Vector3()
        self.angleLidar = Vector3()
        self.body = Vector3()
        self.angleBody = Vector3()

        # self.angle.pose[0].orientation

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.getAngle, queue_size=1)
        rospy.Subscriber("/example/laser/scan", LaserScan, self.laserIn, queue_size=1)

        self.outCloud = rospy.Publisher("outCloud", PointCloud, queue_size=10)

        self.start = rospy.Time.now().to_sec()

    def getAngle(self, data):
        self.lidar = data.pose[4].position
        self.angleLidar = tf.transformations.euler_from_quaternion(
            [
                data.pose[4].orientation.x,
                data.pose[4].orientation.y,
                data.pose[4].orientation.z,
                data.pose[4].orientation.w,
            ]
        )
        self.body = data.pose[1].position
        self.angleBody = tf.transformations.euler_from_quaternion(
            [
                data.pose[1].orientation.x,
                data.pose[1].orientation.y,
                data.pose[1].orientation.z,
                data.pose[1].orientation.w,
            ]
        )

    def laserIn(self, data):
        self.scan = data
        self.update()

    def rotatePitch(self, pitch):

        out = np.matrix([
            [math.cos(pitch), 0, -math.sin(pitch)],
            [0, 1, 0],
            [math.sin(pitch), 0, math.cos(pitch)]]
        )
        return out

    def rotateRoll(self, roll):

        out = np.matrix([
            [1, 0, 0],
            [0, math.cos(roll), math.sin(roll)],
            [0, -math.sin(roll), math.cos(roll)]]
        )
        return out

    def rotateYaw(self, yaw):

        out = np.matrix([[math.cos(yaw), math.sin(yaw), 0],[-math.sin(yaw), math.cos(yaw), 0],[0, 0, 1]])
        return out

    def translate(self, point, change):
        out = Vector3()
        out.x = point.x + change.x
        out.y = point.y + change.y
        out.z = point.z + change.z

        return out

    def worldPoint(self, point):
        out = point
        # X rotation
        out.x = out.x
        out.y = out.y * math.cos(self.angleBody[0]) - out.z * math.sin(
            self.angleBody[0]
        )
        out.z = out.y * math.sin(self.angleBody[0]) + out.z * math.cos(
            self.angleBody[0]
        )

        # Y rotation
        out.x = out.x * math.cos(self.angleBody[1]) + out.z * math.sin(
            self.angleBody[1]
        )
        out.y = out.y
        out.z = -out.x * math.sin(self.angleBody[1]) + out.z * math.cos(
            self.angleBody[1]
        )

        # Z rotation
        out.x = out.x * math.cos(self.angleBody[2]) - out.y * math.sin(
            self.angleBody[2]
        )
        out.y = out.x * math.sin(self.angleBody[2]) + out.y * math.cos(
            self.angleBody[2]
        )
        out.z = out.z

        return out

    def changeSteps(self, dist, pitch, yaw):
        
        # out = self.rotate(out, Vector3(
        #     self.angleBody[0], self.angleBody[1], self.angleBody[2]))
        # out = self.translate(out, Vector3(
        #     self.body.x, self.body.y, self.body.z))

        return 0

    def update(self):
        cloud = PointCloud()
        cloud.header.frame_id = "world"
        cloud.header.stamp = rospy.Time.now()
        data = []

        for i in range(0, 64):
         
            temp = dataStruct()
            temp.create(self.scan.ranges[i],self.scan.angle_min + self.scan.angle_increment * i)
            temp.updatePitch(-self.angleLidar[1])
            temp.translate(Vector3(0.249, 0, 0.505141))
            temp.updateYaw(-self.angleBody[2])
            temp.updatePitch(self.angleBody[1])
            temp.updateRoll(self.angleBody[0])
            temp.translate(Vector3(self.body.x, self.body.y, self.body.z))
            data.append(temp)
            cloud.points.append(temp.getPoint())
        self.outCloud.publish(cloud)

class dataStruct:

    def __init__(self):
        self.point = Vector3()
        self.dist = 0.0
    
    def create(self,dist,angle):
        self.point.x = math.cos(angle)*dist
        self.point.y = math.sin(angle)*dist
        self.point.z = 0
        self.dist = dist
    
    def updatePitch(self,angle):
        temp = self.point.x
        self.point.x = math.cos(angle)*temp - math.sin(angle)*self.point.z
        self.point.y = self.point.y
        self.point.z = temp*math.sin(angle) + math.cos(angle)*self.point.z
    
    def updateRoll(self,angle):
        temp = self.point.y
        self.point.x = self.point.x
        self.point.y = math.cos(angle)*temp - math.sin(angle)*self.point.z
        self.point.z = temp*math.sin(angle) + math.cos(angle)*self.point.z
    
    def updateYaw(self,angle):
        temp = self.point.x
        self.point.x = math.cos(angle)*temp - math.sin(angle)*self.point.y
        self.point.y = temp*math.sin(angle) + math.cos(angle)*self.point.y
        self.point.z = self.point.z
    
    def translate(self,delta):
        self.point.x += delta.x
        self.point.y += delta.y
        self.point.z += delta.z

    
    def getPoint(self):
        return self.point


if __name__ == "__main__":
    rospy.init_node("Lidar")
    rover = Lidar()
    rospy.spin()
