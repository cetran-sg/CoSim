# ---Module which encodes ProtoBuf messages from the simulation data obtained from MATLAB--- #

from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf.internal.decoder import _DecodeVarint32

import time
import datetime
import os
import numpy as np
import random
import pyproj
import utm
from pyproj import CRS
from pyproj import Proj
import utm
from transforms3d.euler import euler2mat, quat2euler, euler2quat
import math

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

# from multiprocessing import Process

# FRAME_INTERVAL=1.45
GPSepoch = time.mktime(datetime.datetime.strptime(
            "06/01/1980", "%d/%m/%Y").timetuple())
currentTime = time.time()

'''
The Carla and Apollo maps are generated from Mathworks RoadRunner.
Carla coordinate system is the same as the exported map coordinates except with -y
Apollo map has a very large offset which is mentioned below and is applied to the localization messages
'''
# APOLLO_MAP_OFFSET_X = 833978.5569
APOLLO_MAP_OFFSET_X = 833964.71433961485
APOLLO_MAP_OFFSET_Y = 9999787.0663516354


# crsMercatorProj = CRS.from_proj4("+proj=tmerc +lat_0=1.351823806386649 +lon_0=103.6940212315423 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs")
# crsUTMProj = CRS.from_epsg(32648)

# projTransformer = pyproj.Transformer.from_crs(crsMercatorProj,crsUTMProj)

projConv = Proj('+proj=tmerc +lat_0=1.35445821465432 +lon_0=103.695726920698 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs', preserve_units=True)

def encode_POmsgs(objCount, objIDs, objType, objHeadings, objLats, objLons, objutmx, objutmy, objVelXs, objVelYs, objBBxs, objBBys, objBBzs, t, seq):
    """ 
        
        Args:

        Returns:

    """
    msg = PerceptionObstacle()
    MSGS = PerceptionObstacles()

    # header msgs
    MSGS.header.sequence_num = int(seq)
    MSGS.header.timestamp_sec = time.time()
    # MSGS.header.timestamp_sec = time.time() + t
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
            
            # utmx, utmy,_,_ = utm.from_latlon(lat,lon)
            # utmx, utmy = projConv(lon,lat)
            

            tStop = time.time()
            projTimes.append(tStop - tStart)

            msg.position.x = objutmx[i]+APOLLO_MAP_OFFSET_X
            msg.position.y = -objutmy[i]+APOLLO_MAP_OFFSET_Y

            msg.velocity.x = float(objVelXs[i])
            msg.velocity.y = float(objVelYs[i])

            # bb_w = 2.13
            # bb_h = 1.41
            # bb_l = 4.71

            msg.length = float(objBBxs[i])
            msg.width = float(objBBys[i])
            msg.height = float(objBBzs[i])

            # msg.width = bb_w
            # msg.length = bb_l
            # msg.height = bb_h

            MSGS.perception_obstacle.extend([msg])

    # print(projTimes)

    # print("\nPO encode time elapsed " + str(time.time()-initTime))
    # print(MSGS)
    binMSGS = MSGS.SerializeToString()
    return binMSGS

def encode_IMUmsgs(avelX,avelY,avelZ, accX, accY, accZ, t, seq):
    """ 
        
        Args:

        Returns:

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
    """ 
        
        Args:

        Returns:

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
    """ 
    
    Args:

    Returns:

    """
    msg = GnssBestPose()

    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'gps'

    msg.measurement_time = t+currentTime
    msg.sol_status = 0
    msg.sol_type = 50
    # msg.latitude = lat
    # msg.longitude = lon
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
    """ 
    
    Args:

    Returns:

    """
    msg = Gps()

    # Apollo expects coordinates in UTM, we use PyProj to convert it as such
    # print(lat,lon)

    # utmx, utmy,_,_ = utm.from_latlon(lat,lon)
    # utmx, utmy = projConv(lon,lat)
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
    """ 
        
    Args:

    Returns:

    """
    msg = InsStat()

    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'gps'

    binmsg = msg.SerializeToString()

    return binmsg

def encode_Chamsgs(t,lat,lon,utmx,utmy,odo,heading,vel,throttle,brake,steering):
    """ 
    
    Args:

    Returns:

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

    # msg.chassis_gps.latitude = lat
    # msg.chassis_gps.longitude = lon 
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
    """ 
    
    Args:

    Returns:

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
    """ 
    
    Args:

    Returns:

    """
    msg = LocalizationEstimate()

    # Apollo expects coordinates in UTM, we use PyProj to convert it as such
    # utmx, utmy,_,_ = utm.from_latlon(lat,lon)
    # utmx, utmy = projConv(lon,lat)
    msg.header.timestamp_sec = time.time()
    msg.header.sequence_num = int(seq)
    msg.header.frame_id = 'novatel'
    # print("\n")
    # print("UTMlib: ", utmx, utmy)

    # print("Projlib: ", utmx1, utmy1)
    # print("\n")
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
    """ 
    
    Args:

    Returns:

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
    #print(p)
    point.intensity,point.timestamp,point.x,point.y,point.z = [int(p.intensity*100),t,p.point.x,-p.point.y,p.point.z]
    return point

def encode_PC(sampleData, j):
    """ 
        
    Args:

    Returns:

    """    
    msg = PointCloud()
    msg.header.timestamp_sec = time.time()
    t = int(time.time())
    msg.header.sequence_num = j
    msg.width = 11520
    msg.height = 1
    msg.measurement_time = time.time()
    msg.frame_id = "velodyne128"
    msg.header.frame_id = "velodyne128"
    msg.is_dense = False

    timeNow = time.time()
    
    lidar_data = np.fromstring(
            bytes(sampleData.raw_data), dtype=np.float32)
    lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
    # lidar_data[:, 1] *= -1

    for lidar_point in lidar_data:
            if (lidar_point[0]**2 + lidar_point[1]**2) >= 2:
                cyber_point = PointXYZIT()
                cyber_point.x = lidar_point[0]
                cyber_point.y = -lidar_point[1]
                cyber_point.z = lidar_point[2]
                msg.point.append(cyber_point)


    # pointList = [ConvertPoints(i, t) for i in sampleData if (i.point.x**2 + i.point.y**2)>6]
    # msg.point.extend(pointList)
    #print('number of points in a sample = %d' %(i))
    timeAfter = time.time()

    binmsg = msg.SerializeToString()

    procTime = timeAfter - timeNow
    # print("ProcTime: " + str(procTime))
    return binmsg


def encode_IMG(imgData, j):
    """ 
        
    Args:

    Returns:

    """    
    msg = Image()
    msg.header.timestamp_sec = time.time()
    
    msg.header.sequence_num = j
    msg.width = 1920
    msg.height = 1080
    msg.encoding = "rgb8"
    #msg.format = "jpg"
    msg.frame_id = "camera"
    #pixels = PIL_Image.open(DATAROOT+"/"+sampleData['filename'])
    #msg.data = pixels.tobytes()
    
    #msg.data=sampleData.tobytes()
    msg.data=imgData

    binmsg = msg.SerializeToString()
    return binmsg


def encode_CompressedIMG(imgData, j):
    """ 
        
    Args:

    Returns:

    """    
    msg = CompressedImage()
    msg.header.timestamp_sec = time.time()
    
    msg.header.sequence_num = j

    msg.format = "jpg"
    msg.frame_id = "camera"
    #pixels = PIL_Image.open(DATAROOT+"/"+sampleData['filename'])
    #msg.data = pixels.tobytes()
    
    #msg.data=sampleData.tobytes(encoder_name='jpeg')
    msg.data=imgData
    binmsg = msg.SerializeToString()
    return binmsg
