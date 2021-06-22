#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from mavros_msgs.msg import Mavlink
from mavros import mavlink
from pymavlink import mavutil
import rospy
import std_msgs
import genpy
import time

rospy.init_node("test", anonymous=True)

# master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
# master.wait_heartbeat()
# mode = 'LAND'
# mode_id = master.mode_mapping()[mode]
# master.mav.set_mode_send(
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id)

# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     48,
#     1,
#     -35.36102763,  149.16230444, 10, 0, 0, 0, 0)

# while True:
#     # Wait for ACK command
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
#     ack_msg = ack_msg.to_dict()

#     # Check if command in the same in `set_mode`
#     if ack_msg['command'] != 48:
#         continue

#     # Print the ACK result !
#     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
#     break

mavlink_pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=1)

# Sending a HEARTBEAT message:

# msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
msg = mavutil.mavlink.MAVLink_set_gps_global_origin_message(0, -35.36054478, 149.16205658, 10)
msg.pack(mavutil.mavlink.MAVLink(''))

header  = std_msgs.msg.Header(1, genpy.Time(),'earth')
print(str(header))

ros_msg = mavlink.convert_to_rosmsg(msg)
print(str(ros_msg))

# mavlink_pub.publish(ros_msg)

# rate = rospy.Rate(10) # 10hz
# while not rospy.is_shutdown():                 
#     rate.sleep()