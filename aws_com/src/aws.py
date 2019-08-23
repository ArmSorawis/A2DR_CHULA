#!/usr/bin/python

# this source is part of my Hackster.io project:  https://www.hackster.io/mariocannistra/radio-astronomy-with-rtl-sdr-raspberrypi-and-amazon-aws-iot-45b617

# use this program to test the AWS IoT certificates received by the author
# to participate to the spectrogram sharing initiative on AWS cloud

# this program will publish test mqtt messages using the AWS IoT hub
# to test this program you have to run first its companion awsiotsub.py
# that will subscribe and show all the messages sent by this program
import rospy
import json
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String, Float32MultiArray
import math
import paho.mqtt.client as paho
import os
import socket
import ssl
import time
from time import sleep
from random import uniform

import math

connflag = False
class robot_subscriber(object):
    isUpdate = False
    isGetPosition = False
    pose_x = 0
    pose_y = 0
    pose_z = 0
    roll = 0
    pitch = 0
    yaw = 0
    strJson = ""
    def __init__(self):
        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        self.ori_x = 0
        self.ori_y = 0
        self.ori_z = 0
        self.ori_w = 0
        
    def callback(self, data):
        self.pose_x = data.pose.pose.position.x 
        self.pose_y = data.pose.pose.position.y 
        self.pose_z = data.pose.pose.position.z
        self.ori_x = data.pose.pose.orientation.x 
        self.ori_y = data.pose.pose.orientation.y
        self.ori_z = data.pose.pose.orientation.z 
        self.ori_w = data.pose.pose.orientation.w 
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        #print("sub topic")
        #print(self.pose_x,self.pose_y,self.pose_z,self.roll,self.pitch,self.yaw)
        isUpdatePose = True
        #x = {   "method":"position",
        #        "type":"res",
        #        "pos_x":self.pose_x,
        #        "pos_y":self.pose_y,
        #        "pos_z":self.pose_z,
        #        "ori_x":self.roll,
        #        "ori_y":self.pitch,
        #        "ori_z":self.yaw
        #    }
        #strJson = json.dumps(x)
        #print("Print Json")
        #print(strJson)
        #print("<------>")
        #mqttc.publish("$aws/things/curobot/mobile", strJson, qos=1)
        #print(self.left_wheel_velocity,self.right_wheel_velocity)
    
    def listener(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)

    def getPositionJson(self):
        return strJson

class robot_move_base_goal_sub(object):
    def __init__(self):
        self.move_pose_x = 0
    def callback(self, data):
        self.move_pose_x = data.pose.position.x
        self.move_pose_y = data.pose.position.y
        self.move_pose_z = data.pose.position.z
        print("sss")
        print(self.move_pose_x,self.move_pose_y,self.move_pose_z)
    def listener(self):
        print("1")
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
        print("2")

def on_connect(client, userdata, flags, rc):
    global connflag
    connflag = True
    print("Connection returned result: " + str(rc) )
    client.subscribe("$aws/things/curobot/mobile" , 1 )

def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.payload))
    #print("topic: "+msg.topic)
    #print(str(msg.payload))
    y = json.loads(str(msg.payload))
    _type = y["TYPE"]
    if _type == "Req":
        _msg = y["MSG"]
        print(_msg)
        if _msg == "GetPosition":
            detail = {
                    "pos_x":robot_node.pose_x,
                    "pos_y":robot_node.pose_y,
                    "pos_z":robot_node.pose_z,
                    "ori_x":robot_node.roll,
                    "ori_y":robot_node.pitch,
                    "ori_z":robot_node.yaw    
                    }
            detailJson = json.dumps(detail)
            x = {
                    "TYPE":"Res",   
                    "MSG":"GetPosition",
                    "DETAIL":detailJson
                }
            strJson = json.dumps(x)
            mqttc.publish("$aws/things/curobot/mobile", strJson, qos=1)
        if _msg == "PutTargetPosition":
            global isPutTargetPositiontarget
            isPutTargetPosition = True
            #print("PutTarget")
            _detail = y["DETAIL"]
            #print("PutTarget")
            jsonDetail = json.loads(_detail)
            #print(_detail)
            nav_goal = PoseStamped()
            
            nav_goal.header.frame_id = "map"
            nav_goal.header.stamp = rospy.get_rostime()
            nav_goal.pose.position.x = jsonDetail["pos_x"]
            nav_goal.pose.position.y = jsonDetail["pos_y"]
            nav_goal.pose.position.z = jsonDetail["pos_z"]
            #print()
            [q1,q2,q3,q4] = quaternion_from_euler(jsonDetail["ori_x"],jsonDetail["ori_y"],jsonDetail["ori_z"])
            nav_goal.pose.orientation.x = q1
            nav_goal.pose.orientation.y = q2
            nav_goal.pose.orientation.z = q3
            nav_goal.pose.orientation.w = q4
            #print(q1)
            pubGoal.publish(nav_goal)

            #print("Pub Goal")
        if _msg == "PutInitialPosition":
            _detail = y["DETAIL"]
            print("PutInitialPosition stage 1")
            jsonDetail = json.loads(_detail)
            print("PutInitialPosition stage 2")
            print(_detail)
            nav_initial_pose = PoseWithCovarianceStamped()
            print("PutInitialPosition stage 3")
            nav_initial_pose.header.frame_id = "map"
            print("PutInitialPosition stage 4")
            nav_initial_pose.header.stamp = rospy.get_rostime()
            print("PutInitialPosition stage 5")
            print(jsonDetail["pos_x"])
            [q1,q2,q3,q4] = quaternion_from_euler(int(jsonDetail["ori_x"]),int(jsonDetail["ori_y"]),int(jsonDetail["ori_z"]))
            p   = PoseWithCovarianceStamped()
            print("pub initial pose 1")
            msg = PoseWithCovariance()
            #print("pub initial pose 2")
            msg.pose = Pose(Point(float(jsonDetail["pos_x"]), float(jsonDetail["pos_y"]), float(jsonDetail["pos_z"])), Quaternion(q1,q2,q3,q4))
            print("pub initial pose 3")
            msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            print("pub initial pose 4")
            p.pose = msg
            print("pub initial pose")
            #pwc = PoseWithCovariance()
            #pwc.pose = Pose(Point(float(jsonDetail["pos_x"]),float(jsonDetail["pos_y"]),float(jsonDetail["pos_z"])), Quaternionquaternion_from_euler(q1,q2,q3,q4))
            #nav_initial_pose.pose.position.x = 12#float(jsonDetail["pos_x"])
            #print("PutInitialPosition stage 6")
            #nav_initial_pose.pose.position.y = int(jsonDetail["pos_y"])
            #nav_initial_pose.pose.position.z = int(jsonDetail["pos_z"])
            #print()
            #print("PutInitialPosition stage 2")
            #nav_initial_pose.pose.orientation.x = q1
            #nav_initial_pose.pose.orientation.y = q2
            #nav_initial_pose.pose.orientation.z = q3
            #nav_initial_pose.pose.orientation.w = q4
            #print("PutInitialPosition stage 3")
            
            pubInitialPose.publish(p)



#def on_log(client, userdata, leveml, buf):
#    print(msg.topic+" "+str(msg.payload))
if __name__ == "__main__":
    rospy.init_node("aws_communication")
    robot_move_base_node = robot_move_base_goal_sub()
    robot_move_base_node.listener()
    robot_node = robot_subscriber()
    robot_node.listener()
    pubGoal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
    pubInitialPose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size = 10)
    mqttc = paho.Client()
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    #mqttc.on_log = on_log

    awshost = "a3rv5sgmirt3cn-ats.iot.ap-southeast-1.amazonaws.com"
    awsport = 8883
    clientId = "myThingName"
    thingName = "myThingName"
    caPath = r'/home/ohm/A2DR/src/aws_com/cer/RootCA.pem'
    certPath = r'/home/ohm/A2DR/src/aws_com/cer/certificate.crt'
    keyPath = r'/home/ohm/A2DR/src/aws_com/cer/private.key'

    #mqttc.tls_set(caPath, certPath, keyPath, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)
    mqttc.tls_set(caPath, certPath, keyPath,cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)
    #mqttc.tls_set("C:/Users/kitti/source/repos/AWSNode/AWSNode/RootCA.pem", "C:/Users/kitti/source/repos/AWSNode/AWSNode certificate.crt", "C:/Users/kitti/source/repos/AWSNode/AWSNode pravate.key", cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)

    mqttc.connect(awshost, awsport, keepalive=60)

    mqttc.loop_start()
    t = time.time()
    #global isGetPosition
    isGetPosition = False
    while (not rospy.is_shutdown()):
        if time.time()-t>=0.001:
            t = time.time()
            #print()
                