#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList
from geometry_msgs.msg import Pose2D
import numpy as np
import math
import time
#import matplotlib.pyplot as plt

class AutoNav:
    def __init__(self):

        self.yaw = 0
        self.obj_list = []
        self.activated = True
        self.state = -1
        self.ang = 0
        self.desired_speed = 0
        self.distance = 0
        self.InitTime = rospy.Time.now().secs
        self.desired_heading = 0

        rospy.Subscriber("/vectornav/ins_2d/ins_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber('/usv_perception/yolo_zed/objects_detected', ObjDetectedList, self.objs_callback)
        self.path_pub = rospy.Publisher('/mission/waypoints', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher("/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/test", Int32, queue_size=10)

    def ins_pose_callback(self,pose):
        self.yaw = pose.theta

    def objs_callback(self,data):
        #print("a")
        self.obj_list = []
        for i in range(data.len):
            if str(data.objects[i].clase) == 'marker':
                self.obj_list.append({'X' : data.objects[i].X + 0.55, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})

    def punto_medio(self):

        distances_list = []
        y_list = []
        class_list = []
        for i in range(len(self.obj_list)):
            distances_list.append(self.obj_list[i]['X'])
            y_list.append(self.obj_list[i]['Y'])
            class_list.append(self.obj_list[i]['class'])

        ind_x1 = np.argsort(distances_list)[0]
        ind_x2 = np.argsort(distances_list)[1]

        x1 = distances_list[ind_x1]
        y1 = -1*y_list[ind_x1]
        x2 = distances_list[ind_x2]
        y2 = -1*y_list[ind_x2]
        xc = min([x1,x2]) + abs(x1 - x2)/2
        yc = min([y1,y2]) + abs(y1 - y2)/2
        self.distance = xc
        offset = .55
        yc = 0.00001 if yc == 0 else yc

        relative_heading = math.atan((xc+offset)/yc)

        obj = Float32MultiArray()
        obj.layout.data_offset = 5
        obj.data = [xc,yc,xm,ym,2]

        self.d_heading_pub.publish(heading)

def main():
    rospy.init_node('auto_nav_position', anonymous=True)
    rate = rospy.Rate(100)
    E = AutoNav()
    while not rospy.is_shutdown() and E.activated:

        if E.state == -1:
            while not rospy.is_shutdown() and len(E.obj_list) < 2:
                E.test.publish(E.state)
                rate.sleep()
            E.state = 0

        if E.state == 0:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2:
                E.punto_medio()
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list) < 2 or E.distance < 2):
                    if rospy.Time.now().secs - initTime > 5:
                        E.state = 1
                        rate.sleep()
                        break

        if E.state == 1:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2:
                E.state = 2
            else:
                E.farther(2)

        if E.state == 2:
            E.test.publish(E.state)
            if len(E.obj_list) >= 2:
                E.punto_medio()
            else:
                initTime = rospy.Time.now().secs
                while not rospy.is_shutdown() and (len(E.obj_list) < 2 or E.distance < 2):
                    if rospy.Time.now().secs - initTime > 5:
                        E.state = 3
                        rate.sleep()
                        break

        if E.state == 3:
            E.test.publish(E.state)
            time.sleep(1)
            E.status_pub.publish(1)

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
