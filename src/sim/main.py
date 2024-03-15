import carla
import random
import sys
import numpy as np
from ApolloBridgeClient import *
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


import conf as CONFIG

if CONFIG.resOn:
    from scenarioManagerRes import *
else:
    from scenarioManager import *


import time
import pandas as pd


SCENARIO_DELAY = 3.0
class ActorSnapshots():
    def __init__(self):
        self.prevSnaps = list()
        self.snapLimit = 10

    def parseActorSnapshot(self, snap, obsIDList, actorList, carlamap):
        """ Parse the Carla snapshot to extract actors data
        
        Args:
            snap carla.Snapshot: carla snapshot to be parsed
            obsIDList : list of actors ID to be included in the output
            actorList : list of Carla.actors, used to retrieve static informations
            carlamap :
        Returns:
            a tuple including all the actor properties
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

        prevSnap = None
        if len(self.prevSnaps) >= self.snapLimit:
            prevSnap = self.prevSnaps.pop(0)
            deltaTime = snap.timestamp.elapsed_seconds - prevSnap.timestamp.elapsed_seconds 
            # print("deltatime:  ",deltaTime)
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
            snap carla.Snapshot: carla snapshot to be parsed
            egoID : ID of the ego vehicle Carla.actor 
            carlamap : 
            throttle : feedback data 
            brake : feedback data
            steering : feedback data
        Returns:
            a tuple including all the ego vehicle properties
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
    
    ###################################################################################################################################


    
    ###################################################################################################################################


def actorReader(CARLA_HOST, CARLA_PORT):
    """ Manager function to send actor data on the bridge

    Args:
        CARLA_HOST string: IP of the simulator
        CARLA_PORT int: port of the simulator
    """
    print("\n-- Started actorReader process --\n")
    import logging
    from datetime import datetime
    timeNow = datetime.now()
    frmtTimeNow = timeNow.strftime("%H:%M:%S.%f on %d/%m/%Y")[:-3]
    tnString = timeNow.strftime("%Y%m%d_%H%M%S")

    # Uncomment to enable logging
    #logging.disable(logging.NOTSET)

    # Uncomment to disable logging
    logging.disable(sys.maxsize)

    # Logging initialization
    logname = "CarlaClient_" + "ActorGroundTruth" + tnString + ".log"
    logging.basicConfig(filename=logname,filemode='a',format='[%(asctime)s.%(msecs)d]-[%(name)s]-[%(levelname)s]- %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)
    fh = logging.FileHandler(logname)
    fh.setLevel(logging.DEBUG)

    logging.info("Process %s started at %s", "ActorGroundTruth", frmtTimeNow)
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        world = client.get_world()
        carlamap = world.get_map()
        spectator = world.get_spectator()
        logger = logging.getLogger("actorReader")
        client_pub = ApolloBridgeClient_ActorGTPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        client_pub.connectToServer()
        client_pub.register()
        # print(world.get_actors())
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
            # count +=1

    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def camReader(CARLA_HOST, CARLA_PORT):
    """ Manager function to send camera on the bridge

    Args:
        CARLA_HOST string: IP of the simulator
        CARLA_PORT int: port of the simulator
    """
    import logging
    from datetime import datetime
    timeNow = datetime.now()
    frmtTimeNow = timeNow.strftime("%H:%M:%S.%f on %d/%m/%Y")[:-3]
    tnString = timeNow.strftime("%Y%m%d_%H%M%S")

    # Uncomment to enable logging
    #logging.disable(logging.NOTSET)

    # Uncomment to disable logging
    logging.disable(sys.maxsize)

    # Logging initialization
    logname = "CarlaClient_" + "CamReader" + tnString + ".log"
    logging.basicConfig(filename=logname,filemode='a',format='[%(asctime)s.%(msecs)d]-[%(name)s]-[%(levelname)s]- %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)
    fh = logging.FileHandler(logname)
    fh.setLevel(logging.DEBUG)

    logging.info("Process %s started at %s", "Cam Reader", frmtTimeNow)
    try:

        # Connect to the client and retrieve the world object

        camera_pub = ApolloBridgeClient_ImgPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        camera_pub.connectToServer()
        camera_pub.register()

        print("\n-- Started camReader process --\n")
        logger = logging.getLogger("camReader")
        
        # Initialize scenario manager and spawn actors
        camManager = sensorManager(CARLA_HOST, CARLA_PORT)
        logger.debug("Carla connected ...")
        
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
        CARLA_HOST string: IP of the simulator
        CARLA_PORT int: port of the simulator
    """
    import logging
    from datetime import datetime
    timeNow = datetime.now()
    frmtTimeNow = timeNow.strftime("%H:%M:%S.%f on %d/%m/%Y")[:-3]
    tnString = timeNow.strftime("%Y%m%d_%H%M%S")

    # Uncomment to enable logging
    #logging.disable(logging.NOTSET)

    # Uncomment to disable logging
    logging.disable(sys.maxsize)

    # Logging initialization
    logname = "CarlaClient_" + "LiDARReader" + tnString + ".log"
    logging.basicConfig(filename=logname,filemode='a',format='[%(asctime)s.%(msecs)d]-[%(name)s]-[%(levelname)s]- %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)
    fh = logging.FileHandler(logname)
    fh.setLevel(logging.DEBUG)

    logging.info("Process %s started at %s", "LiDAR Reader", frmtTimeNow)
    try:

        # Connect to the client and retrieve the world object

        lidar_pub = ApolloBridgeClient_PCPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        lidar_pub.connectToServer()
        lidar_pub.register()

        print("\n-- Started LiDARReader process --\n")
        logger = logging.getLogger("LiDARReader")
        
        # Initialize scenario manager and spawn actors
        lidarManager = sensorManager(CARLA_HOST, CARLA_PORT)
        logger.debug("Carla connected ...")
        
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

def egoReader(CARLA_HOST, CARLA_PORT, ego_sumo_endflag):
    """ Manager function to send ego data on the bridge + more things TODO
 
    Args:
        CARLA_HOST string: IP of the simulator
        CARLA_PORT int: port of the simulator
    """
    import logging
    from datetime import datetime
    timeNow = datetime.now()
    frmtTimeNow = timeNow.strftime("%H:%M:%S.%f on %d/%m/%Y")[:-3]
    tnString = timeNow.strftime("%Y%m%d_%H%M%S")

    # Uncomment to enable logging
    #logging.disable(logging.NOTSET)

    # Uncomment to disable logging
    logging.disable(sys.maxsize)

    # Logging initialization
    logname = "CarlaClient_" + "EgoReader" + tnString + ".log"
    logging.basicConfig(filename=logname,filemode='a',format='[%(asctime)s.%(msecs)d]-[%(name)s]-[%(levelname)s]- %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)
    fh = logging.FileHandler(logname)
    fh.setLevel(logging.DEBUG)

    logging.info("Process %s started at %s", "EgoReader", frmtTimeNow)
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)

        world = client.get_world()

        carlamap = world.get_map()

        spectator = world.get_spectator()

        # if egoCreated.egoCreated is False:
        #     egoCreated.ego_vehicle = ego_vehicle

        # ego_vehicle = None

        # spectator_transform = spectator.get_transform(ego_vehicle)
        logger = logging.getLogger("EgoReader")


        client_pub = ApolloBridgeClient_EgoPublisher(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        client_pub.connectToServer()
        client_pub.register()
        logger.debug("Server connected ...")
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
            # print('Elapsed Time: ', elapsed_time)
            world_snapshot = world.wait_for_tick()
            controlFeedback = ego_vehicle.get_control()
            throttle=controlFeedback.throttle*100
            brake=controlFeedback.brake*100
            steering=controlFeedback.steer*100
            #print(controlFeedback)
            # logger.debug("@ %f",steering)
            ego_transform = ego_vehicle.get_transform()
            if ego_vehicle.is_at_traffic_light():
                traffic_light = ego_vehicle.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                # world.hud.notification("Traffic light changed! Good to go!")
                    traffic_light.set_state(carla.TrafficLightState.Green)

            if CONFIG.egoOn:
                #Back View
                spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(x=-10 * math.cos(math.radians(ego_transform.rotation.yaw)),
                                                                                                y=-10 * math.sin(math.radians(ego_transform.rotation.yaw)),
                                                                                                z=5),carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw)))

                #Onboard View
                # spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(x=0.78 * math.cos(math.radians(ego_transform.rotation.yaw)),
                #                                                                                     y=0.78 * math.sin(math.radians(ego_transform.rotation.yaw)),
                #                                                                                     z=1.32),carla.Rotation(pitch=-8, yaw=ego_transform.rotation.yaw)))
                
            dataList = parseEgoSnapshot(world_snapshot, egoID, carlamap, throttle, brake, -steering)
            client_pub.sendEgoData(dataList)
            (t, seq, lat, lon, utmx, utmy, qw, qx, qy, qz, accX,accY,accZ,avelX,avelY,avelZ,odo,eulx,euly,eulz,vel,throttle,brake,steering,velx, vely, heading) = dataList

            end_sumo_x=utmx-102.5
            end_sumo_y=utmy-692.1
            end_sumo_dist = math.sqrt(end_sumo_x**2 + end_sumo_y**2)
            res_sumo_x=utmx+107.4
            res_sumo_y=utmy-96.3
            res_sumo_dist = math.sqrt(res_sumo_x**2 + res_sumo_y**2)

            if end_sumo_dist <5 or res_sumo_dist <170:
                ego_sumo_endflag.value = True
                # print('Stop Sumo')
            # if ego (utmx/utmy) or (lat/lon) position within 5m of checkpoint


    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def egoWriter(CARLA_HOST, CARLA_PORT):
    """ Manager function to receive data from the ADS and control the ego vehicle in Carla accordingly

    Args:
        CARLA_HOST string: IP of the simulator
        CARLA_PORT int: port of the simulator
    """
    import logging
    from datetime import datetime
    timeNow = datetime.now()
    frmtTimeNow = timeNow.strftime("%H:%M:%S.%f on %d/%m/%Y")[:-3]
    tnString = timeNow.strftime("%Y%m%d_%H%M%S")

    # Uncomment to enable logging
    #logging.disable(logging.NOTSET)

    # Uncomment to disable logging
    logging.disable(sys.maxsize)
    # Logging initialization
    logname = "CarlaClient_" + "EgoWriter" + tnString + ".log"
    logging.basicConfig(filename=logname,filemode='a',format='[%(asctime)s.%(msecs)d]-[%(name)s]-[%(levelname)s]- %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)
    fh = logging.FileHandler(logname)
    fh.setLevel(logging.DEBUG)

    logging.info("Process %s started at %s", "EgoWriter", frmtTimeNow)
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)

        world = client.get_world()
        # spectator = world.get_spectator()
        # Connect to the client and retrieve the world object
        logger = logging.getLogger("EgoWriter")
        client_sub = ApolloBridgeClient_Subscriber(CONFIG.APOLLO_HOST,CONFIG.APOLLO_PORT)
        client_sub.connectToServer()
        client_sub.register()
        # print('Writer')
        logger.debug("Server connected....")
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
            logger.debug("@ %f",steeringTarget)
            
            # self.ccfeed.throttle ,self.ccfeed.brake,self.ccfeed.steeringTarget = throttle,brake,steeringTarget
            
            
            t = time.time()
            dt = t-lastTime
            lastTime = t
            controlFeedback=ego_vehicle.get_control()
            steeringAngle=-controlFeedback.steer*100
            sgn = math.copysign(1, steeringTarget - steeringAngle)

            steeringRate = steeringRate * sgn
            steeringAngle += steeringRate * dt
            
            # vehicleCC = carla.VehicleControl( throttle = throttle/100 ,brake = brake/100,steer=-steeringAngle/100 )
            vehicleCC = carla.VehicleControl( throttle = throttle/100 ,brake = brake/100,steer=-steeringTarget/100 )
            # print(vehicleCC)
            # print("applying", vehicleCC)
            ego_vehicle.apply_control(vehicleCC)
            # spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(x=-10 * math.cos(math.radians(ego_transform.rotation.yaw)),
            #                                                                                 y=-10 * math.sin(math.radians(ego_transform.rotation.yaw)),
            #                                                                                 z=5),carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw)))
            
    except Exception as e:
        print(e)
        print("Problem handling request")
    finally:
        sys.exit()

def scenarioPlayer(CARLA_HOST, CARLA_PORT):
    """ Manager function to control carla actors

    Args:
        CARLA_HOST string: IP of the simulator
        CARLA_PORT int: port of the simulator
    """
    import logging
    from datetime import datetime
    timeNow = datetime.now()
    frmtTimeNow = timeNow.strftime("%H:%M:%S.%f on %d/%m/%Y")[:-3]
    tnString = timeNow.strftime("%Y%m%d_%H%M%S")

    # Uncomment to enable logging
    #logging.disable(logging.NOTSET)

    # Uncomment to disable logging
    logging.disable(sys.maxsize)

    # Logging initialization
    logname = "CarlaClient_" + "ScenarioPlayer" + tnString + ".log"
    logging.basicConfig(filename=logname,filemode='a',format='[%(asctime)s.%(msecs)d]-[%(name)s]-[%(levelname)s]- %(message)s',datefmt='%H:%M:%S',level=logging.DEBUG)
    fh = logging.FileHandler(logname)
    fh.setLevel(logging.DEBUG)

    logging.info("Process %s started at %s", "Scenario Player", frmtTimeNow)
    try:
        # Connect to the client and retrieve the world object
        client = carla.Client(CARLA_HOST, CARLA_PORT)

        world = client.get_world()

        carlamap = world.get_map()


        logger = logging.getLogger("scenarioPlayer")
        
        # Initialize scenario manager and spawn actors
        scManager = ScenarioManager(CARLA_HOST, CARLA_PORT)
        logger.debug("Carla connected ...")
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

def main():


    client = carla.Client(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT)
    client.load_world(CONFIG.worldID)
    world = client.get_world()

    carlamap = world.get_map()

    spectator = world.get_spectator()

    ego_sumo_endflag = multiprocessing.Value('b', False)


    """this need to be fixed, i.e. from file + config"""
    if not CONFIG.resOn:
        if CONFIG.egoSP ==0:
            '''From Start Point'''
            ego_spawn = carla.Transform(carla.Location(x=-524.872, y=-210.508, z=3), carla.Rotation(yaw=67.5))
        if CONFIG.egoSP ==1:    
            '''From Start Point with SUMO'''
            ego_spawn = carla.Transform(carla.Location(x=-529.872, y=-206.508, z=3), carla.Rotation(yaw=67.5))
        if CONFIG.egoSP ==2:
            '''From Check Point 0'''
            ego_spawn = carla.Transform(carla.Location(x=-84, y=400, z=3), carla.Rotation(yaw=52.5))
        if CONFIG.egoSP ==3:
            '''From Check Point 1'''
            ego_spawn = carla.Transform(carla.Location(x=278.473, y=517.110, z=3), carla.Rotation(yaw=240))
        if CONFIG.egoSP ==4:
            '''From Check Point 2'''
            ego_spawn = carla.Transform(carla.Location(x=-67.99, y=439.76, z=3), carla.Rotation(yaw=233))
    if CONFIG.resOn:
        if not CONFIG.egoOn:
            '''From ResSidewalk'''
            ego_spawn = carla.Transform(carla.Location(x=-112.68, y=69.61, z=3), carla.Rotation(yaw=322.5))
        if CONFIG.egoOn:
            '''From Resembler'''
            ego_spawn = carla.Transform(carla.Location(x=-28.022-102*math.sin(51.17), y=33.865+102*math.cos(51.17), z=3), carla.Rotation(yaw=322.5))

    """Traffic light test"""
    # ego_spawn = carla.Transform(carla.Location(x=-76.68, y=72.61, z=3), carla.Rotation(yaw=322.5))
    '''From intersection'''
    # ego_spawn = carla.Transform(carla.Location(x=-72.68, y=69.61, z=3), carla.Rotation(yaw=322.5))
    '''After intersection'''
    # ego_spawn = carla.Transform(carla.Location(x=-40.49, y=43.65, z=3), carla.Rotation(yaw=322.5))
    '''After SC2'''
    # ego_spawn = carla.Transform(carla.Location(x=65.477, y=-34.510, z=3), carla.Rotation(yaw=322.5)) 
    '''Beside Ambulance'''
    # ego_spawn = carla.Transform(carla.Location(x=-7.022+118.04-7.5,y=17.865-92.56+10, z=3), carla.Rotation(yaw=322.5)) 


    ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2017')
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
        egoReaderProcess = multiprocessing.Process(target=egoReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT, ego_sumo_endflag))
    if CONFIG.egoWriterFlag:
        egoWriterProcess = multiprocessing.Process(target=egoWriter, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.camReaderFlag :
        camReaderProcess = multiprocessing.Process(target=camReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))
    if CONFIG.lidarReaderFlag:
        lidarReaderProcess = multiprocessing.Process(target=lidarReader, args=(CONFIG.CARLA_HOST, CONFIG.CARLA_PORT))


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

    # Apollo docker: cyber_launch start modules/drivers/tools/image_decompress/launch/image_decompress.launch
    
    # Comment below if you don't need actor ground truth
    # actorReaderProcess.start()
    #
    

    sleep(10.0)
    
    if CONFIG.scenarioPlayerFlag:
        scenarioPlayerProcess.start()
    

    
if __name__ == '__main__':
    main()

