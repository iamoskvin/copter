#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import os
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

from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandInt, CommandLong, StreamRate, StreamRateRequest
from geometry_msgs.msg import PoseStamped
# from tf.transformations import quaternion_from_euler
from tf_conversions import transformations

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

    def speed_test(self, req):
        testOjb = speedtest.Speedtest()
        down = testOjb.download()
        up = testOjb.upload()
        return [ True, 'Скачивание: ' + str(round(down/1024/1024)) + ' МБ, загрузка: ' + str(round(up/1024/1024)) + ' МБ' ]

class CopterManager:
    def __init__(self, towerManager):
        self.towerManager = towerManager
        self.service = rospy.Service('start_stop_copter', SetBool, self.manage)                
        self.pos_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)        
        self.height = 0
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.curr_height)


    def takeoff(self):        
        rospy.wait_for_service('mavros/set_mode')        
        rospy.wait_for_service('mavros/cmd/arming')                
        rospy.wait_for_service('mavros/cmd/takeoff')

        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        set_mode(custom_mode='guided')
        arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        arming(True)
        try:            
            rospy.wait_for_service('mavros/cmd/takeoff')
            print("COPTER: takeoff started")
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
            print(azimuthDeg)
            azimuthRad = azimuthDeg/360.0*math.pi*2
            print(azimuthRad)

            q = transformations.quaternion_from_euler(0, 0, azimuthRad)
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
        if req.data:                               
            self.takeoff()            
        else:                        
            self.land()

        return [True, '2']
    
    def land(self):
        print("COPTER: land started")
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        set_mode(custom_mode='land')

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
            
            {'id': 3, 'area': '', 'cell_id': '', 'lat': 55.860412, 'long': 37.118994, 'azimuth': 0, 'name': 'Север0'},
            {'id': 4, 'area': '', 'cell_id': '', 'lat': 55.860412, 'long': 37.118994, 'azimuth': 90, 'name': 'Запад90'},
            {'id': 5, 'area': '', 'cell_id': '', 'lat': 55.860412, 'long': 37.118994, 'azimuth': 180, 'name': 'Юг180'},
            {'id': 6, 'area': '', 'cell_id': '', 'lat': 55.860412, 'long': 37.118994, 'azimuth': 270, 'name': 'Восток270'},
            {'id': 7, 'area': '', 'cell_id': '', 'lat': 55.860412, 'long': 37.118994, 'azimuth': -90, 'name': 'Восток-90'},

        ]
        return True, json.dumps(self.towers)

    def getTowerById(self, tower_id):
        for tower in self.towers:
            if tower['id'] == tower_id:
                return tower

class MonitorClientConnetion():
    def __init__(self, manager):
        self.last_connect_time = 0
        self.copter_manager = manager
        self.sub = rospy.Subscriber('/web_client_connection2', Bool, self.update)

    def update(self, msg):        
        self.last_connect_time = time.time()
        # print(self.last_connect_time)
    
    def check(self, event = None):
        now = time.time()
        diff = now - self.last_connect_time
        
        max_time = rospy.get_param('~client_disconnect_time')
        if diff > max_time: 
            self.copter_manager.land()
            

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
        if sys.version_info[0] > 2:
            msg = msg.decode('utf8')
        return [True, msg]

class SystemdReloader():
    def __init__(self):
        self.service = rospy.Service('~restart_program', SetBool, self.restart)
    def restart(self):
        os.system("sudo systemctl restart copter.service")

def main():
    global manager      
    rospy.init_node("copter", anonymous=True, disable_signals=True)
    # mavros.set_namespace() 

    # parameters in case of standalone script run withput launch file

    if not rospy.get_param('~height', None):
        rospy.set_param('~height', 15)
    if not rospy.get_param('~client_disconnect_time', None):
        rospy.set_param('~client_disconnect_time', 20)
    
    # set message rate for our controller
    setRate = rospy.ServiceProxy('/mavros/set_stream_rate',StreamRate)
    setRate(stream_id=StreamRateRequest.STREAM_ALL, message_rate=10, on_off=True)
        
    SpeedTest()
    ping = InetPing()

    towerManager = TowerList()
    manager = CopterManager(towerManager)        
    up = MonitorClientConnetion(manager)

    port = rospy.get_param('~modem_dev', None)    

    if port and port != 'None':
        modemManager = ModemManager(port)    

    SystemdReloader()

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
        manager.land()
