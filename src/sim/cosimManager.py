import carla
import random
import sys
import numpy as np
from bridgeClient import *
from sensorManager import *
import math
import modules
from time import sleep
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from threading import Thread
from agents.navigation.controller import VehiclePIDController
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from transforms3d.quaternions import quat2mat, mat2quat

import sys
import os
import time

import config as CONFIG

from scenarioManager import *

import time
import pandas as pd

# Delays the start of the scenario (in seconds)
SCENARIO_DELAY = 3.0
class ActorSnapshots():
    def __init__(self):
        self.prevSnaps = list()

        # Number of previous snapshots to store for velocity estimation
        self.snapLimit = 10

    def parseActorSnapshot(self, snap, obsIDList, actorList, carlamap):
        """ Parse the Carla snapshot to extract actor data. This is subsequently used to generate the actor ground truth message for the ADS.
        
        Args:
            snap carla.Snapshot: Carla snapshot to be parsed
            obsIDList : List of actor IDs to be included in the output
            actorList : List of carla.actors used to retrieve static information
            carlamap : Carla world.map object used to get properties such as those for geolocation transformation
        Returns:
            A tuple including all actor properties for each actor
        """
        t = snap.timestamp.elapsed_seconds
        seq = snap.frame
        objCount =0 
        objID =list()
        objType=list()
        objHeading=list()
        objLat =list()
        objLon =list()
        objutmx=list()
        objutmy=list()
        objVelX=list()
        objVelY=list()
        BBx=list()
        BBy=list()
        BBz=list()


        # Estimating velocity of objects based on previous snapshot if instantaneous velocity parameter is not present
        prevSnap = None
        if len(self.prevSnaps) >= self.snapLimit:
            prevSnap = self.prevSnaps.pop(0)
            deltaTime = snap.timestamp.elapsed_seconds - prevSnap.timestamp.elapsed_seconds
        for obsID in obsIDList:
            obs = snap.find(obsID)
            prevObs = None
            if prevSnap is not None:
                prevObs = prevSnap.find(obsID)
            obsTrans = obs.get_transform()
            objutmx.append(obsTrans.location.x)
            objutmy.append(obsTrans.location.y)
            obsVel = obs.get_velocity()

            objCount +=1
            objID.append(obsID)

            objHeading.append(-math.radians(obsTrans.rotation.yaw))
            objLoc = carlamap.transform_to_geolocation(obsTrans.location)
            objLat.append(objLoc.latitude)
            objLon.append(objLoc.longitude)

            if (obsVel.x**2 + obsVel.y**2 < 0.5 ) and prevObs is not None:
                prevTrans = prevObs.get_transform()
                velx =    (obsTrans.location.x - prevTrans.location.x)/deltaTime
                vely =    (obsTrans.location.y - prevTrans.location.y)/deltaTime
                objVelX.append(velx)
                objVelY.append(-vely)
            else:
                objVelX.append(obsVel.x)
                objVelY.append(-obsVel.y)
            
            # Applying bounding box values for other objects from actor properties
            # and defining custom bounding boxes for objects with no bounding box definitions in Carla
            actor = actorList.find(obsID)
            if actor.attributes.get('role_name') == 'traffic_cone':
                objType.extend([1])
                BBx.append(0.4)
                BBy.append(0.4)
                BBz.append(0.8)
            elif actor.attributes.get('role_name') == 'ped':
                objType.extend([3])
                BBx.append(actor.bounding_box.extent.x*2)
                BBy.append(actor.bounding_box.extent.y*2)
                BBz.append(actor.bounding_box.extent.z*2)
            elif actor.attributes.get('role_name') == 'bicycle':
                objType.extend([4])
                BBx.append(2)
                BBy.append(0.5)
                BBz.append(1.8)
            elif actor.attributes.get('role_name') == 'obstacle':
                objType.extend([5])
                BBx.append(actor.bounding_box.extent.x*2)
                BBy.append(actor.bounding_box.extent.y*2)
                BBz.append(actor.bounding_box.extent.z*2)
            elif actor.attributes.get('role_name') == 'sumo_driver':
                objType.extend([5])
                BBx.append(actor.bounding_box.extent.x*2)
                BBy.append(actor.bounding_box.extent.y*2)
                BBz.append(actor.bounding_box.extent.z*2)
        self.prevSnaps.append(snap)
        return (t, seq, objCount, objID, objType, objHeading, objLat, objLon, objutmx, objutmy, objVelX, objVelY, BBx, BBy,BBz)

def parseEgoSnapshot(snap, egoID, carlamap, throttle, brake, steering):
    """ Parse the Carla snapshot to extract ego vehicle data
        
        Args:
            snap carla.Snapshot : Carla snapshot to be parsed
            egoID : ID of the ego vehicle which is a carla.actor 
            carlamap : Carla world.map object used to get properties such as those for geolocation transformation
            throttle : Throttle value taken from the ADS to be fed back in the ego state message
            brake : Brake value taken from the ADS to be fed back in the ego state message
            steering : Steering value taken from the ADS to be fed back in the ego state message
        Returns:
            A tuple including all properties pertaining to the ego vehicle state
        """
    ego = snap.find(egoID)
    t = snap.timestamp.elapsed_seconds
    seq = snap.frame
    
    egoTrans = ego.get_transform()
    
    utmx = egoTrans.location.x
    utmy = egoTrans.location.y
    
    geoLoc = carlamap.transform_to_geolocation(egoTrans.location)
    lat = geoLoc.latitude 
    lon = geoLoc.longitude 
    
    r,p,y = math.radians(egoTrans.rotation.roll),-math.radians(egoTrans.rotation.pitch),-math.radians(egoTrans.rotation.yaw-90)
    
    qw, qx, qy, qz  = euler2quat(r,p,y)
    egoAcc = ego.get_acceleration()
    accX = egoAcc.x
    accY = -egoAcc.y
    accZ = egoAcc.z
    egoAngVel = ego.get_angular_velocity()
    avelX = math.radians(egoAngVel.x)
    avelY = -math.radians(egoAngVel.y)
    avelZ = -math.radians(egoAngVel.z)

    odo = 0
    eulx, euly, eulz = math.radians(egoTrans.rotation.roll),-math.radians(egoTrans.rotation.pitch),-math.radians(egoTrans.rotation.yaw)
    egoVel = ego.get_velocity()
    vel = np.linalg.norm(np.array([egoVel.x,-egoVel.y,egoVel.z]))

    velx = egoVel.x
    vely = -egoVel.y
    heading = math.radians(-egoTrans.rotation.yaw)

    return (t, seq, lat, lon, utmx, utmy, qw, qx, qy, qz, accX,accY,accZ,avelX,avelY,avelZ,odo,eulx,euly,eulz,vel,throttle,brake,steering,velx, vely, heading)

def actorReader(CARLA_HOST, CARLA_PORT):
    """ Manager function to send actor data using the bridge

    Args:
        CARLA_HOST string: Host IP of Carla server
        CARLA_PORT int: Port of Carla server
    """
    print("\n-- Started actorReader process --\n")
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        world = client.get_world()
        carlamap = world.get_map()
        spectator = world.get_spectator()

        client_pub = ApolloBridgeClient_ActorGTPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        client_pub.connectToServer()
        client_pub.register()
        obsIDList = []
        
        acCount = 0
        print("\n--- Carla actor list ---")
        parserSnapshot = ActorSnapshots()
        while True:
            world_snapshot = world.wait_for_tick()

            actorList = world.get_actors()

            nonEgoActorIDs = [
                actor.id for actor in actorList if actor.attributes.get('role_name') in ('obstacle', 'traffic_cone', 'ped', 'bicycle', 'sumo_driver')
            ]
            dataList =  parserSnapshot.parseActorSnapshot(world_snapshot, nonEgoActorIDs, actorList, carlamap)
            client_pub.sendActorGroundTruth(dataList)

    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def camReader(CARLA_HOST, CARLA_PORT):
    """ Manager function to send camera on the bridge

    Args:
        CARLA_HOST string: Host IP of Carla server
        CARLA_PORT int: Port of Carla server
    """
    try:

        # Connect to the client and retrieve the world object
        camera_pub = ApolloBridgeClient_ImgPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        camera_pub.connectToServer()
        camera_pub.register()

        print("\n-- Started camReader process --\n")
        
        # Initialize scenario manager and spawn actors
        camManager = sensorManager(CARLA_HOST, CARLA_PORT)
        
        camera = camManager.spawn_cameras()
        camera.listen(lambda image: camera_pub.sendcImgData(image))

        while True:
            sleep(0.01)
    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def lidarReader(CARLA_HOST, CARLA_PORT):
    """ Manager function to send Lidar data on the bridge

    Args:
        CARLA_HOST string: Host IP of Carla server
        CARLA_PORT int: Port of Carla server
    """
    try:

        # Connect to the client and retrieve the world object

        lidar_pub = ApolloBridgeClient_PCPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        lidar_pub.connectToServer()
        lidar_pub.register()

        print("\n-- Started LiDARReader process --\n")
        
        # Initialize scenario manager and spawn actors
        lidarManager = sensorManager(CARLA_HOST, CARLA_PORT)
        
        lidar = lidarManager.spawn_lidars()
        lidar.listen(lambda point_cloud: lidar_pub.sendPCData(point_cloud))
        print("Lidar loopss")
        while True:
            sleep(0.01)
        
    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def egoReader(CARLA_HOST, CARLA_PORT):
    """ Manager function to send ego data on the bridge + more things TODO
 
    Args:
        CARLA_HOST string: Host IP of Carla server
        CARLA_PORT int: Port of Carla server
    """
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)

        world = client.get_world()

        carlamap = world.get_map()

        spectator = world.get_spectator()

        client_pub = ApolloBridgeClient_EgoPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        client_pub.connectToServer()
        client_pub.register()
  
        ego_vehicle = None
        print('\n --- Ego vehicle ---')

        for actor in world.get_actors():
                # print(actor.attributes.get('role_name'))
                if actor.attributes.get('role_name') == 'hero':
                    egoID = actor.id
                    ego_vehicle = actor
                    print("Ego: " + str(ego_vehicle))
        print("\n-- Started egoReader process --\n")
        while True:

            world_snapshot = world.wait_for_tick()
            controlFeedback = ego_vehicle.get_control()
            throttle=controlFeedback.throttle*100
            brake=controlFeedback.brake*100
            steering=controlFeedback.steer*100


            ego_transform = ego_vehicle.get_transform()
            if ego_vehicle.is_at_traffic_light():
                traffic_light = ego_vehicle.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    traffic_light.set_state(carla.TrafficLightState.Green)

            if CONFIG.egoOn:
                # Third person View
                spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(x=-10 * math.cos(math.radians(ego_transform.rotation.yaw)),
                                                                                                y=-10 * math.sin(math.radians(ego_transform.rotation.yaw)),
                                                                                                z=5),carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw)))
                
            dataList = parseEgoSnapshot(world_snapshot, egoID, carlamap, throttle, brake, -steering)
            client_pub.sendEgoData(dataList)
            (t, seq, lat, lon, utmx, utmy, qw, qx, qy, qz, accX,accY,accZ,avelX,avelY,avelZ,odo,eulx,euly,eulz,vel,throttle,brake,steering,velx, vely, heading) = dataList

    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def egoWriter(CARLA_HOST, CARLA_PORT):
    """ Manager function to receive data from the ADS and control the ego vehicle in Carla accordingly

    Args:
        CARLA_HOST string: Host IP of Carla server
        CARLA_PORT int: Port of Carla server
    """
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)

        world = client.get_world()
        # spectator = world.get_spectator()
        # Connect to the client and retrieve the world object

        client_sub = ApolloBridgeClient_Subscriber(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        client_sub.connectToServer()
        client_sub.register()
        # print('Writer')

        ego_vehicle = None
        for actor in world.get_actors():
            if actor.attributes.get('role_name') == 'hero':
                ego_vehicle = actor
        t = time.time()
        lastTime = t
        print("\n-- Started egoWriter process --\n")
        while True:
            # print("waiting CC")
            # ego_transform = ego_vehicle.get_transform()
            throttle, brake, steeringTarget, steeringRate = client_sub.recvSimData()
            
            t = time.time()
            dt = t-lastTime
            lastTime = t
            controlFeedback=ego_vehicle.get_control()
            steeringAngle=-controlFeedback.steer*100
            sgn = math.copysign(1, steeringTarget - steeringAngle)

            steeringRate = steeringRate * sgn
            steeringAngle += steeringRate * dt
            
            vehicleCC = carla.VehicleControl( throttle = throttle/100 ,brake = brake/100,steer=-steeringTarget/100 )
            ego_vehicle.apply_control(vehicleCC)
            
    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def scenarioPlayer(CARLA_HOST, CARLA_PORT):
    """ Manager function to control carla actors

    Args:
        CARLA_HOST string: Host IP of Carla server
        CARLA_PORT int: Port of Carla server
    """
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        world = client.get_world()
        carlamap = world.get_map()

        # Initialize scenario manager and spawn actors
        scManager = ScenarioManager(CARLA_HOST, CARLA_PORT)

        ego_vehicle = None
        print("\nWaiting %d seconds then starting ScenarioPlayer ...\n" % (SCENARIO_DELAY))
        sleep(SCENARIO_DELAY)
        start_time = time.perf_counter()
        for actor in world.get_actors():
            # print(actor.attributes.get('role_name'))
            if actor.attributes.get('role_name') == 'hero':
                egoID = actor.id
                ego_vehicle = actor
            elif actor.attributes.get('role_name') == 'sumo_driver':
                if actor is not None:
                    actor.destroy()
        count = 0
        print("\n-- Started ScenarioPlayer process --\n")
        while True:
            # print('Elapsed Time: ', elapsed_time)
            world_snapshot = world.wait_for_tick()
            ego_transform = ego_vehicle.get_transform()
            #PID controller for Vehicles
            scManager.executeScenario(ego_transform, start_time)
            count +=1

    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def cosimManager():

    client = carla.Client(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT)
    client.load_world(CONFIG.worldID)
    world = client.get_world()

    carlamap = world.get_map()

    spectator = world.get_spectator()

    ego_spawn = carla.Transform(carla.Location(x=CONFIG.ego_spawn_pos_x, y=CONFIG.ego_spawn_pos_y, z=3), carla.Rotation(yaw=CONFIG.ego_spawn_rot_yaw))

    ego_bp = world.get_blueprint_library().find(CONFIG.carla_ego_name)
    ego_bp.set_attribute('role_name', 'hero')
    ego_vehicle = world.spawn_actor(ego_bp, ego_spawn)
    
    sleep(0.5)
    
    egoID = ego_vehicle.id

    ego_transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(x=-10 * math.cos(math.radians(ego_transform.rotation.yaw)),
                                                                                    y=-10 * math.sin(math.radians(ego_transform.rotation.yaw)),z=5),
                                                                                    carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw)))

    
    """
    Initialization for bridge processes
    """
    if CONFIG.scenarioPlayerFlag:
        scenarioPlayerProcess = multiprocessing.Process(target=scenarioPlayer, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.actorReaderFlag:
        actorReaderProcess = multiprocessing.Process(target=actorReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.egoReaderFlag:
        egoReaderProcess = multiprocessing.Process(target=egoReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.egoWriterFlag:
        egoWriterProcess = multiprocessing.Process(target=egoWriter, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.camReaderFlag :
        camReaderProcess = multiprocessing.Process(target=camReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.lidarReaderFlag:
        lidarReaderProcess = multiprocessing.Process(target=lidarReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))

    """
    Starting bridge processes
    """
    if CONFIG.actorReaderFlag:
        actorReaderProcess.start()
    if CONFIG.egoReaderFlag:
        egoReaderProcess.start()
    if CONFIG.egoWriterFlag:
        egoWriterProcess.start()
    if CONFIG.camReaderFlag :
        camReaderProcess.start()
    if CONFIG.lidarReaderFlag:
        lidarReaderProcess.start()
    
    if CONFIG.scenarioPlayerFlag:
        # Wait 2 seconds before starting scenario player
        sleep(2.0)
        scenarioPlayerProcess.start()
    
if __name__ == '__main__':
    main()