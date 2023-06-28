# -*- coding: UTF-8 -*-
import rosbag
import sys
import os
import numpy as np
from sensor_msgs.msg import NavSatFix
import math
import rospy
from geometry_msgs.msg import Vector3

def readGNSS(imu_path):
    timestamps = []
    lats = []
    lons = []
    alts = []
    fin = open(imu_path, 'r')
    fin.readline()
    for i in range(1,23) :
        line = fin.readline().strip()
        i=i+1
    while line:
        parts = line.split( )
        ts  = float(parts[0])+315964800+604800*2258-8*3600
        lat = float(parts[2]) * 180 / math.pi
        lon = float(parts[3]) * 180 / math.pi
        alt = float(parts[4])
        timestamps.append(ts)
        lats.append(lat)
        lons.append(lon)
        alts.append(alt)
        line = fin.readline().strip()
    return timestamps, lats, lons, alts

if __name__ == '__main__':
    gnss_path = sys.argv[1]  # GNSS数据文件路径
    gnss_topic_name = sys.argv[2]    # Topic名称
    bag_path = sys.argv[3]  # 输出Bag路径

    bag_out = rosbag.Bag(bag_path,'a')

    gnss_ts, lats, lons, alts = readGNSS(gnss_path)
    gnss_msg = NavSatFix()
    blh = Vector3()

    for i in range(len(gnss_ts)):
        gnss_ts_ros = rospy.rostime.Time.from_sec(gnss_ts[i])
        gnss_msg.header.stamp = gnss_ts_ros

        gnss_msg.latitude = lats[i]
        gnss_msg.longitude = lons[i]
        gnss_msg.altitude = alts[i]

        bag_out.write(gnss_topic_name, gnss_msg, gnss_ts_ros)
        print('gnss:',i,'/',len(gnss_ts))
    
    bag_out.close()
