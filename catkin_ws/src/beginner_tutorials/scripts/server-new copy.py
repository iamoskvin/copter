#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import socket
import speedtest
import time
import threading
import rospy
import mavros
from mavros import command

from std_msgs.msg import String
# from std_msgs.msg import Bool
from std_srvs.srv import EmptyResponse, TriggerRequest, SetBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool, CommandTOL, CommandInt, CommandLong
from sensor_msgs.msg import NavSatFix


class InetPing:
    def __init__(self):
        self.publisher = rospy.Publisher("/ping_dns", String, queue_size=1)
        self.ping_data = ''
    
    def do_ping(self, event=None):
        host = "8.8.8.8"
        port = 53
        timeout = 3 
        self.ping_data = 'TEST'       
        return
        
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
    


def _check_ret(args, ret):
    if not ret.success:
        pass
        # fault("Request failed. Check mavros logs. ACK:", ret.result)

    # print_if(args.verbose, "Command ACK:", ret.result)

def _find_gps_topic(args, op_name):
    # XXX: since 0.13 global position always exists. need redo that.
    global_fix = mavros.get_topic('global_position', 'global')
    gps_fix = mavros.get_topic('global_position', 'raw', 'fix')

    topics = rospy.get_published_topics()
    # need find more elegant way
    if len([topic for topic, type_ in topics if topic == global_fix]):
        return global_fix
    elif len([topic for topic, type_ in topics if topic == gps_fix]):
        # print_if(args.verbose, "Use GPS_RAW_INT data!")
        return gps_fix
    elif args.any_gps:
        t = [topic for topic, type_ in topics if type_ == 'sensor_msgs/NavSatFix']
        if len(t) > 0:
            print("Use", t[0], "NavSatFix topic for", op_name)
            return t[0]

    return None

def do_takeoff_cur_gps(args):
    done_evt = threading.Event()
    def fix_cb(fix):
        global sub
        print("Taking-off from current coord: Lat:", fix.latitude,
              "Long:", fix.longitude)
        # print_if(args.verbose, "With desired Altitude:", args.altitude,
                #  "Yaw:", args.yaw, "Pitch angle:", args.min_pitch)

        try:
            ret = command.takeoff(min_pitch=args['min_pitch'],
                             yaw=args['yaw'],
                             latitude=fix.latitude,
                             longitude=fix.longitude,
                             altitude=args['altitude'])
        except rospy.ServiceException as ex:
            pass
            # fault(ex)

        _check_ret(args, ret)
        done_evt.set()
        sub.unregister()
        time.sleep(20)
        cmd_serv = rospy.ServiceProxy('mavros/cmd/command', CommandLong)        

        ret = cmd_serv(command=195,
                             param5=-35.36320329,
                             param6= 149.16504644,
                             param7=15
                             )

        time.sleep(5)
        ret = cmd_serv(command=195,
                             param5=-35.36318715,
                             param6= 149.16545067,
                             param7=15
                             )
        time.sleep(3)

        ret = cmd_serv(command=195,
                             param5=-35.36320329,
                             param6= 149.16504644,
                             param7=15
                             )

        time.sleep(3)
        ret = cmd_serv(command=195,
                             param5=-35.36318715,
                             param6= 149.16545067,
                             param7=15
                             )
    topic = _find_gps_topic(args, "takeoff")
    if topic is None:
        pass
        # fault("NavSatFix topic not exist")
    global sub
    sub = rospy.Subscriber(topic, NavSatFix, fix_cb)
    
    
    if not done_evt.wait(10.0):
        pass
        # fault("Something went wrong. Topic timed out.")
        
def manage(action):
    print('action')
    print(action)
    if (bool(action.data)):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        set_mode(custom_mode='guided')
        arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        arming(True)

        do_takeoff_cur_gps({'min_pitch' : 0.1,
            'yaw' : 0.1,
            'altitude' : 15})
    else:
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        set_mode(custom_mode='rtl')
    return [True, '2']


def main():  
    rospy.init_node("copter", anonymous=True, disable_signals=True)
    mavros.set_namespace()  
        
    ping = InetPing()
    SpeedTest()

    service = rospy.Service('start_stop_copter', SetBool, manage)

    rospy.Timer(rospy.Duration(10.0), ping.do_ping)
    rospy.Timer(rospy.Duration(10.0), ping.publish_ping)  
   
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
