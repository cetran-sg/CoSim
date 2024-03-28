# ---Module which encodes and decodes ProtoBuf messages from the simulation data and Baidu Apollo--- #

from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf.internal.decoder import _DecodeVarint32

import time
import datetime
import os
import numpy as np
import random
import utm
from transforms3d.euler import euler2mat, quat2euler, euler2quat
import math
import config as CONFIG

# Import the generated Python ProtoBuf message classes
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles, PerceptionObstacle
from modules.common_msgs.sensor_msgs.gnss_best_pose_pb2 import GnssBestPose
from modules.common_msgs.sensor_msgs.heading_pb2 import Heading
from modules.common_msgs.localization_msgs.gps_pb2 import Gps
from modules.common_msgs.sensor_msgs.ins_pb2 import InsStat
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.localization_msgs.imu_pb2 import CorrectedImu
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.sensor_msgs.imu_pb2 import Imu
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud, PointXYZIT
from modules.common_msgs.sensor_msgs.sensor_image_pb2 import CompressedImage,Image
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStampeds,TransformStamped
# from modules.common_msgs.perception_msgs.traffic_light_detection_pb2 import TransformStampeds,TransformStamped
# from modules.common_msgs.sensor_msgs.radar_pb2 import ContiRadar

GPSepoch = time.mktime(datetime.datetime.strptime(
            "06/01/1980", "%d/%m/%Y").timetuple())
currentTime = time.time()

'''
The Carla and Apollo maps are generated from Mathworks RoadRunner.
Carla coordinate system is the same as the exported map coordinates except with -y
Apollo map has an arbitrary offset which is specified in the config file and used here.

To find this offset, create a 5m long road from (0,0) to (0,5) in RoadRunner. Export the OpenDrive (.xodr) and Apollo map (.txt, .bin) formats.
From the .xodr, find the ID of the road. Find the same road ID in the Apollo .txt file and note the x and y values. These values are the offset. 
'''
APOLLO_MAP_OFFSET_X = CONFIG.APOLLO_MAP_OFFSET_X
APOLLO_MAP_OFFSET_Y = CONFIG.APOLLO_MAP_OFFSET_Y

def encode_POmsgs(objCount, objIDs, objType, objHeadings, objLats, objLons, objutmx, objutmy, objVelXs, objVelYs, objBBxs, objBBys, objBBzs, t, seq):
    """ Package and serialize actor snapshot data from the simulator into Apollo's Perception Obstacle message
        
        Args:
            objCount : Number of objects/obstacles in current simulation snapshot
            objIDs : Unique identifier assigned to each object/obstacle
            objType : Type of object (vehicle, pedestrian, cones etc.)
            objHeadings : Heading of each object
            objLats : Latitude of each object
            objLons : Longitude of each object
            objutmx : UTM X-coordinate of each object
            objutmy : UTM Y-coordinate of each object
            objVelXs : Velocity of each object in the X-axis
            objVelYs : Velocity of each object in the Y-axis
            objBBxs : Bounding box dimension of each object in the X-axis
            objBBys : Bounding box dimension of each object in the Y-axis
            objBBzs : Bounding box dimension of each object in the Z-axis
            t : Timestamp
            seq : Sequence number
        Returns:
            Perception Obstacles message as a bytestring

    """
    msg = PerceptionObstacle()
    MSGS = PerceptionObstacles()

    # header msgs
    MSGS.header.sequence_num = int(seq)
    MSGS.header.timestamp_sec = time.time()
    MSGS.header.module_name = 'perception_obstacle'

    # GPS time in seconds
    msg.timestamp = currentTime+t
    projTimes = []
    initTime = time.time()
    if objCount is not None:
        for i in range(int(objCount)):
            msg.id = int(objIDs[i])
            msg.theta = float(objHeadings[i])
            msg.type = int(objType[i])
            lat = float(objLats[i])
            lon = float(objLons[i])
            tStart = time.time()
            
            tStop = time.time()
            projTimes.append(tStop - tStart)

            msg.position.x = objutmx[i]+APOLLO_MAP_OFFSET_X
            msg.position.y = -objutmy[i]+APOLLO_MAP_OFFSET_Y

            msg.velocity.x = float(objVelXs[i])
            msg.velocity.y = float(objVelYs[i])

            msg.length = float(objBBxs[i])
            msg.width = float(objBBys[i])
            msg.height = float(objBBzs[i])

            MSGS.perception_obstacle.extend([msg])
    binMSGS = MSGS.SerializeToString()
    return binMSGS

def encode_IMUmsgs(avelX,avelY,avelZ, accX, accY, accZ, t, seq):
    """ Package and serialize ego vehicle IMU data from the simulator to Apollo's format
        
        Args:
            avelX : Angular velocity in the X-axis
            avelY : Angular velocity in the Y-axis
            avelZ : Angular velocity in the Z-axis
            accX : Acceleration in the X-axis
            accY : Acceleration in the Y-axis
            accZ : Acceleration in the Z-axis
            t : Timestamp
            seq : Sequence number

        Returns:
            IMU message as a bytestring
        """
    msg = Imu()

    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.measurement_time = t+currentTime
    msg.measurement_span = 0.01

    msg.linear_acceleration.x = accX
    msg.linear_acceleration.y = accY
    msg.linear_acceleration.z = accZ

    msg.angular_velocity.x = avelX
    msg.angular_velocity.y = avelY
    msg.angular_velocity.z = avelZ

    binmsg = msg.SerializeToString()

    return binmsg

def encode_corrIMUmsgs(avelX,avelY,avelZ,eulX,eulY,eulZ,accX,accY,accZ,heading,t):
    """ Package and serialize ego vehicle Corrected IMU data from the simulator to Apollo's format
        
        Args:
            avelX : Angular velocity in the X-axis
            avelY : Angular velocity in the Y-axis
            avelZ : Angular velocity in the Z-axis
            eulX : Euler angle in the X-axis
            eulY : Euler angle in the Y-axis
            eulZ : Euler angle in the Z-axis
            accX : Acceleration in the X-axis
            accY : Acceleration in the Y-axis
            accZ : Acceleration in the Z-axis
            heading : Heading (yaw)
            t : Timestamp

        Returns:
            Corrected IMU message as a bytestring
        """
    msg = CorrectedImu()

    msg.header.timestamp_sec = time.time()

    msg.imu.linear_acceleration.x = accX
    msg.imu.linear_acceleration.y = accY
    msg.imu.linear_acceleration.z = accZ

    msg.imu.angular_velocity.x = avelX
    msg.imu.angular_velocity.y = avelY
    msg.imu.angular_velocity.z = avelZ

    msg.imu.heading = int(heading)

    msg.imu.euler_angles.x = float(eulX)
    msg.imu.euler_angles.y = float(eulY)
    msg.imu.euler_angles.z = float(eulZ)

    binmsg = msg.SerializeToString()

    return binmsg

def encode_BPmsgs(lat,lon,utmx, utmy, t,seq):
    """ Package and serialize ego vehicle Best Pose data from the simulator to Apollo's format
    
    Args:
        lat : Latitude
        lon : Longitude
        utmx : UTM X-coordinate
        utmy : UTM Y-coordinate
        t : Timestamp
        seq : Sequence number

    Returns:
        Best Pose message as a bytestring

    """
    msg = GnssBestPose()

    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'gps'

    msg.measurement_time = t+currentTime
    msg.sol_status = 0
    msg.sol_type = 50
    msg.height_msl = 0.0
    msg.undulation = 0.0
    msg.datum_id = 61

    msg.latitude_std_dev = 0.01
    msg.longitude_std_dev = 0.01
    msg.height_std_dev = 0.01

    msg.differential_age = 2.0
    msg.solution_age = 0.0
    msg.num_sats_tracked = 15
    msg.num_sats_in_solution = 15
    msg.num_sats_l1 = 15
    msg.num_sats_multi = 12
    msg.extended_solution_status = 33
    msg.galileo_beidou_used_mask = 0
    msg.gps_glonass_used_mask = 51

    binmsg = msg.SerializeToString()

    return binmsg

def encode_GPSmsg(lat,lon,utmx,utmy,qw,qx,qy,qz,velx,vely,heading,t,seq):
    """ Package and serialize ego vehicle GPS data from the simulator to Apollo's format
    
    Args:

    Returns:
        GPS message as a bytestring
    """
    msg = Gps()
    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)

    msg.localization.position.x = utmx+APOLLO_MAP_OFFSET_X
    msg.localization.position.y = -utmy+APOLLO_MAP_OFFSET_Y
    msg.localization.orientation.qw = float(qw)
    msg.localization.orientation.qx = float(qx)
    msg.localization.orientation.qy = float(qy)
    msg.localization.orientation.qz = float(qz)

    msg.localization.linear_velocity.x = velx
    msg.localization.linear_velocity.y = vely
    msg.localization.linear_velocity.z = 0


    msg.localization.heading =heading

    binmsg = msg.SerializeToString()

    return binmsg

def encode_INSmsgs(t,seq):
    """ Package and serialize ego vehicle INS data from the simulator to Apollo's format
        
    Args:

    Returns:
        INS message as a bytestring
    """
    msg = InsStat()

    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'gps'

    binmsg = msg.SerializeToString()

    return binmsg

def encode_Chamsgs(t,lat,lon,utmx,utmy,odo,heading,vel,throttle,brake,steering):
    """ Package and serialize ego vehicle chassis data from the simulator to Apollo's format
    
    Args:

    Returns:
        Chassis message as a bytestring
    """
    msg = Chassis()
    now = datetime.datetime.now()
    msg.engine_started = 1

    msg.speed_mps = vel
    msg.odometer_m = float(odo)
    msg.throttle_percentage = throttle
    msg.brake_percentage = brake
    msg.steering_percentage = steering
    msg.driving_mode = 1
    msg.gear_location = 1

    msg.header.module_name = 'chassis'
    msg.chassis_gps.altitude = 0.0
    msg.chassis_gps.heading = float(heading)
    msg.chassis_gps.year = now.year
    msg.chassis_gps.month = now.month
    msg.chassis_gps.day = now.day
    msg.chassis_gps.hours = now.hour
    msg.chassis_gps.minutes = now.minute
    msg.chassis_gps.seconds = now.second
    msg.chassis_gps.compass_direction = float(heading)

    binmsg = msg.SerializeToString()
    return binmsg

def encode_Heamsgs(t,seq,heading):
    """ Package and serialize ego vehicle heading from the simulator to Apollo's format
    
    Args:

    Returns:
        Heading message as a bytestring
    """
    msg = Heading()

    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'gps'
    msg.measurement_time = t+currentTime
    msg.heading = heading
    binmsg = msg.SerializeToString()
    return binmsg

def encode_Posemsgs(lat,lon,utmx,utmy,qw,qx,qy,qz,velx,vely,heading,avelX,avelY,avelZ, accX, accY, accZ,t,seq):
    """ Package and serialize ego vehicle pose from the simulator to Apollo's format
    
    Args:

    Returns:
        Pose message as a bytestring
    """
    msg = LocalizationEstimate()
    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'novatel'
    msg.pose.position.x = utmx+APOLLO_MAP_OFFSET_X
    msg.pose.position.y = -utmy+APOLLO_MAP_OFFSET_Y
    msg.pose.position.z = 0

    msg.pose.orientation.qw = float(qw)
    msg.pose.orientation.qx = float(qx)
    msg.pose.orientation.qy = float(qy)
    msg.pose.orientation.qz = float(qz)

    msg.pose.linear_velocity.x = velx
    msg.pose.linear_velocity.y = vely
    msg.pose.linear_velocity.z = 0

    msg.pose.angular_velocity.x = avelX
    msg.pose.angular_velocity.y = avelY
    msg.pose.angular_velocity.z = avelZ


    msg.pose.linear_acceleration.x = accX
    msg.pose.linear_acceleration.y = accY
    msg.pose.linear_acceleration.z = accZ

    msg.pose.heading = heading

    binmsg = msg.SerializeToString()

    return binmsg 

def encode_Tfmsgs(utmx,utmy,eulx,euly,eulz,t,seq):
    """ Package and serialize ego vehicle transform from the simulator to Apollo's format
    
    Args:

    Returns:
        Transform message as a bytestring
    """
    MSGS = TransformStampeds()
    MSGS.header.timestamp_sec = time.time()
    MSGS.header.sequence_num = int(seq)
    MSGS.header.frame_id = 'world'
    msg = TransformStamped()
    msg.header.timestamp_sec = time.time()
    msg.header.frame_id = 'world'
    msg.child_frame_id = 'localization'
    
    msg.transform.translation.x = utmx+APOLLO_MAP_OFFSET_X
    msg.transform.translation.y = -utmy+APOLLO_MAP_OFFSET_Y
    msg.transform.translation.z = 0

    
    qw, qx, qy, qz  = euler2quat(eulx,euly,eulz-math.radians(90))
    msg.transform.rotation.qx = float(qx)
    msg.transform.rotation.qy = float(qy)
    msg.transform.rotation.qz = float(qz)
    msg.transform.rotation.qw = float(qw)

    MSGS.transforms.extend([msg])

    msg = TransformStamped()
    msg.header.timestamp_sec = time.time()
    msg.header.frame_id = 'world'
    msg.child_frame_id = 'novatel'
    
    msg.transform.translation.x = utmx+APOLLO_MAP_OFFSET_X
    msg.transform.translation.y = -utmy+APOLLO_MAP_OFFSET_Y
    msg.transform.translation.z = 0

    msg.transform.rotation.qx = float(qx)
    msg.transform.rotation.qy = float(qy)
    msg.transform.rotation.qz = float(qz)
    msg.transform.rotation.qw = float(qw)

    MSGS.transforms.extend([msg])
    binMSGS = MSGS.SerializeToString()

    return binMSGS

def ConvertPoints(p,t):
    """ 
    
    Args:

    Returns:

    """
    point =PointXYZIT()
    point.intensity,point.timestamp,point.x,point.y,point.z = [int(p.intensity*100),t,p.point.x,-p.point.y,p.point.z]
    return point

def encode_PC(pcData, seq):
    """ Package and serialize LiDAR pointcloud from the simulator to Apollo's format
        
    Args:
        pcData : Pointcloud data from the simulator
        seq : Sequence number

    Returns:
        Pointcloud message as a bytestring
    """    
    msg = PointCloud()
    msg.header.timestamp_sec = time.time()
    t = int(time.time())
    msg.header.sequence_num = seq
    msg.width = 11520
    msg.height = 1
    msg.measurement_time = time.time()
    msg.frame_id = "velodyne128"
    msg.header.frame_id = "velodyne128"
    msg.is_dense = False

    timeNow = time.time()
    
    lidar_data = np.fromstring(
            bytes(pcData.raw_data), dtype=np.float32)
    lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))

    for lidar_point in lidar_data:
            if (lidar_point[0]**2 + lidar_point[1]**2) >= 2:
                cyber_point = PointXYZIT()
                cyber_point.x = lidar_point[0]
                cyber_point.y = -lidar_point[1]
                cyber_point.z = lidar_point[2]
                msg.point.append(cyber_point)

    timeAfter = time.time()
    binmsg = msg.SerializeToString()
    procTime = timeAfter - timeNow
    return binmsg


def encode_IMG(imgData, seq):
    """ Package and serialize camera images from the simulator to Apollo's format
        
    Args:
        imgData : Image data from the simulator
        seq: Sequence number

    Returns:
        Image data as a bytestring
    """    
    msg = Image()
    msg.header.timestamp_sec = time.time()
    
    msg.header.sequence_num = seq
    msg.width = 1920
    msg.height = 1080
    msg.encoding = "rgb8"
    msg.frame_id = "camera"
    msg.data=imgData

    binmsg = msg.SerializeToString()
    return binmsg


def encode_CompressedIMG(imgData, seq):
    """ Package and serialize compressed camera images from the simulator to Apollo's format
        
    Args:
        imgData : Image data from the simulator
        seq : Sequence number

    Returns:
        Compressed image data as a bytestring
    """    
    msg = CompressedImage()
    msg.header.timestamp_sec = time.time()
    
    msg.header.sequence_num = seq

    msg.format = "jpg"
    msg.frame_id = "camera"
    msg.data=imgData
    binmsg = msg.SerializeToString()
    return binmsg
