#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import socket
import time
import json
import rospy
import mavros
import speedtest
import math
import serial

from std_msgs.msg import String, Bool
from std_srvs.srv import EmptyResponse, TriggerRequest, SetBool

from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandInt, CommandLong
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class InetPing:
    def __init__(self):
        self.publisher = rospy.Publisher("/ping_dns", String, queue_size=1)
        self.ping_data = ''
    
    def do_ping(self, event=None):
        host = "8.8.8.8"
        port = 53
        timeout = 3 
        # self.ping_data = 'TEST'       
        # return
        
        try:
            socket.setdefaulttimeout(timeout)
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
            self.ping_data = 'PING OK'
        except socket.error as ex:
            self.ping_data = 'PING error: '+ str(ex)

    def publish_ping(self, event=None):
        msg = String()
        msg.data = self.ping_data
        self.publisher.publish(msg)

class SpeedTest:
    def __init__(self):
        self.service = rospy.Service('speed_test', SetBool, self.speed_test)
        self.testOjb = speedtest.Speedtest()

    def speed_test(self, req):
        down = self.testOjb.download()
        up = self.testOjb.upload()
        return [ True, 'Скачивание: ' + str(round(down/1024/1024)) + ' МБ, загрузка: ' + str(round(up/1024/1024)) + ' МБ' ]

class CopterManager:
    def __init__(self, towerManager):
        self.towerManager = towerManager
        self.service = rospy.Service('start_stop_copter', SetBool, self.manage)                
        self.pos_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)        
        self.height = 0
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.curr_height)


    def takeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            rospy.wait_for_service('mavros/cmd/takeoff')
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
            takeoffService(altitude = rospy.get_param('~height'), min_pitch = 0.1, yaw = 0.1)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" %e)

    def curr_height(self, msg):
        self.height = msg.pose.position.z

    def pos_publish(self, event = None):                                        
        if (self.height < 1):
            return

        goal_height = rospy.get_param('~height')
        tower_id = rospy.get_param('/tower_id', None)
        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = goal_height

        if (tower_id):
            tower = self.towerManager.getTowerById(tower_id)
            # print(tower)
            azimuthDeg = tower['azimuth']+90 #NED to EUN
            azimuthRad = azimuthDeg/360*math.pi

            q = quaternion_from_euler(0, 0, azimuthRad)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            # goal.pose.orientation.x = 0.0
            # goal.pose.orientation.y = 0.0
            # goal.pose.orientation.z = 0.0
            # goal.pose.orientation.w = 1.0
            
        else:
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
        
        self.pos_publisher.publish(goal) 

    def manage(self, req):
        rospy.wait_for_service('mavros/set_mode')        
        rospy.wait_for_service('mavros/cmd/arming')        
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        if (req.data):                   
            set_mode(custom_mode='guided')
            arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            arming(True)
            self.takeoff()            
        else:            
            set_mode(custom_mode='rtl')
        return [True, '2']

class TowerList():
    def __init__(self):
        self.service = rospy.Service('towers', SetBool, self.list_towers)
        self.towers = []
    
    def list_towers(self, req):
        # # test loaction
        # towers = [
        #     {'id': 1, 'area': 9011, 'cell_id': 10091578, 'lat': 55.72534971202, 'long': 36.5391388394},
        #     {'id': 2, 'area': 9011, 'cell_id': 52561, 'lat': 55.67285720976, 'long': 36.75367633687},
        #     {'id': 3, 'area': 5030, 'cell_id': 198478860, 'lat': 55.63610848072, 'long': 36.71853923459},
        # ]

        # dedovsk
        self.towers = [
            {'id': 1, 'area': '', 'cell_id': '', 'lat': 55.888573, 'long': 37.096052, 'azimuth': 30.444183, 'name': 'Турбодиспансер'},
            {'id': 2, 'area': '', 'cell_id': '', 'lat': 55.860412, 'long': 37.118994, 'azimuth': 202.676375, 'name': 'Дедовск'},
        ]
        return True, json.dumps(self.towers)

    def getTowerById(self, tower_id):
        for tower in self.towers:
            if tower['id'] == tower_id:
                return tower

class MonitorClientConnetion():
    def __init__(self):
        self.last_connect_time = 0
        self.sub = rospy.Subscriber('/web_client_connection2', Bool, self.update)

    def update(self, msg):        
        self.last_connect_time = time.time()
        # print(self.last_connect_time)
    
    def check(self, event = None):
        now = time.time()
        diff = now - self.last_connect_time
        if diff > 20: 
            rospy.wait_for_service('mavros/set_mode')            
            set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            set_mode(custom_mode='rtl')

class ModemManager():
    def __init__(self, port):        
        self.ser = serial.Serial(port=port, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1,  rtscts=False, dsrdtr=False)
        self.service = rospy.Service('/modem_connect', SetBool, self.make_connect)

    def checkModem(self):
        cmd = "AT\r"
        self.ser.write(cmd.encode())
        msg = self.ser.read(64)
        # print(msg)
        
    def make_connect(self, req):
        cmd = "AT+CREG=2\r"
        self.ser.write(cmd.encode())
        msg = self.ser.read(64)
        # print(msg)
        
        cmd = "AT+CREG?\r"
        self.ser.write(cmd.encode())
        msg = self.ser.read(64)
        # print(repr(msg))
        # msg = "\r\n+CREG: 2,0\r\n\r\nOK\r\n"
        return [True, msg]

def main():      
    rospy.init_node("copter", anonymous=True, disable_signals=True)
    # mavros.set_namespace() 

    # parameters
    if not rospy.get_param('~height', None):
        rospy.set_param('~height', 15)
        
    SpeedTest()
    towerManager = TowerList()
    manager = CopterManager(towerManager)    
    ping = InetPing()
    up = MonitorClientConnetion()

    port = rospy.get_param('~modem_dev', None)    

    if port and port != 'None':
        modemManager = ModemManager(port)    

    rospy.Timer(rospy.Duration(30.0), ping.do_ping)
    rospy.Timer(rospy.Duration(30.0), ping.publish_ping)  

    rospy.Timer(rospy.Duration(1.0), up.check)

    rospy.Timer(rospy.Duration(2.0), manager.pos_publish)
   
    # rospy.spin()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():                 
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        set_mode(custom_mode='rtl')
