import carla
import random
import sys
import numpy as np
from ApolloBridgeClient import *
import math
# from haversine import haversine, Unit
import modules
from time import sleep
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from threading import Thread
from agents.navigation.controller import VehiclePIDController
from agents.navigation.controller_wpLoc import VehiclePIDController as VehiclePIDControllerLoc
import time
import pandas as pd

class scenarioManager():
    def __init__(self, CARLA_HOST, CARLA_PORT):
        self.ScenarioNum = 1
        self.EgoOn = False
        self.SingleNpc = False
        self.showWP = False

        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = self.client.get_world()
        self.carlamap = self.world.get_map()
        self.spectator = self.world.get_spectator()
        self.recordCamera = None
        # Read Resembler dataframe
        self.vutR_df = pd.read_csv(f'ResemblerData{self.ScenarioNum}/VUT_status.csv', sep=';')
        self.npcR_df = pd.read_csv(f'ResemblerData{self.ScenarioNum}/Environment_actors.csv', sep=';')
        

        # print('actors',self.npcR_df)

        self.groupedActors = self.npcR_df.groupby(['Actor_Id', 'Actor_type'])
        # print('group',actor_id)
        self.actor_names = []
        # Create separate dataframes for each Actor based on type. 
        for (actor_id, actor_type), group in self.groupedActors: 
            # print('group',actor_type)
            if self.SingleNpc:
                if actor_type.lower() == 'car' and actor_id == 'B':
                    globals()[f'npc{actor_id}_df'] = group.copy()
                    self.actor_names.append(f'{actor_id}')
                elif actor_type.lower() == 'ped':
                    globals()[f'ped{actor_id}_df'] = group.copy()
                    self.actor_names.append(f'{actor_id}')

            if not self.SingleNpc:
                if actor_type.lower() == 'car':
                    globals()[f'npc{actor_id}_df'] = group.copy()
                    self.actor_names.append(f'{actor_id}')
                if actor_type.lower() == 'motorcycle':
                    globals()[f'npc{actor_id}_df'] = group.copy()
                    self.actor_names.append(f'{actor_id}')
                if actor_type.lower() == 'bus':
                    globals()[f'npc{actor_id}_df'] = group.copy()
                    self.actor_names.append(f'{actor_id}')
                elif actor_type.lower() == 'ped':
                    globals()[f'ped{actor_id}_df'] = group.copy()
                    self.actor_names.append(f'{actor_id}')

        # print(globals()[f'npcB_df'].iloc[0])
        self.waypoints = self.client.get_world().get_map().generate_waypoints(distance=1.0)
        self.npc_timer_start = dict()
        self.scenarioStartFlag = False

        self.vut_timer_start = time.time()
        self.vutR_wp=[]
        self.vut_wpCount = 0
        self.npc_wpCount = dict()

        

        if self.ScenarioNum == 1:
            self.x_ref=-28.022
            self.y_ref=33.865
            self.z_ref=3
            self.yaw_ref=322.5
            self.ScRange = 100 
            self.carlaAngle = math.atan(65.658/51.745)
            self.init_df = pd.read_csv(f'ResemblerData{self.ScenarioNum}/Environment_metadata.csv')

        if self.ScenarioNum == 2:
            self.x_ref=-66.99
            self.y_ref=440.76
            self.z_ref=3
            self.yaw_ref=232.5 
            self.ScRange = 0
            # self.carlaAngle = math.atan(65.658/51.745)+math.radians(90)
            self.carlaAngle = math.atan(65.658/51.745)
            self.init_df = pd.read_csv(f'ResemblerData{self.ScenarioNum}/Environment_metadata.csv', sep=';')
        
        self.transformAngle = 0
        
        self.x_ego=self.x_ref-self.ScRange*math.sin(self.carlaAngle)
        self.y_ego=self.y_ref+self.ScRange*math.cos(self.carlaAngle)
        self.actorList = self.spawnActors()

        self.duration = self.init_df['Total_duration'].iloc[0]
        print(self.duration)
        self.totalCount = int(self.duration/0.1)
        print('Total count:',self.totalCount)
	    #### Wait before executing scenario ####
        # sleep(1.0)

        (obs_vutR) = self.actorList

        #initialise scenario

        xNorth =  5/7
        yNorth = -5/7
        pedDataAngle = 0
        pedAngle = math.radians(pedDataAngle) #rads
        self.xTarg = xNorth*math.cos(pedAngle) - yNorth*math.sin(pedAngle)
        self.yTarg = xNorth*math.sin(pedAngle) + yNorth*math.cos(pedAngle)

        

        #-----------------------------------Resembler--------------------------------------------------------#
        

        self.vut_wp = []
        self.vutR_wpCount = 0
        self.vutR_spd = 0.0
        self.vutDT = self.vutR_df['VUT_travelled'].iloc[-1]
        # print(self.vutDT)
        # self.vutSPx = 100*math.sin(51.756778)
        # self.vutSPy = 100*math.cos(51.3256)
        
        self.getWaypoints()
        # self.drawWaypoints(road_id, life_time)

    def haversine(self, lat1, lon1, lat2, lon2):
        # Radius of the earth in meters
        R = 6371000.0

        # Convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

        # Differences in coordinates
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 

        # Haversine formula 
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a)) 
        r = R * c

        # Calculate the bearing
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)

        # Convert the bearing from radians to degrees
        bearing = math.degrees(bearing)

        # Adjust the bearing to be between 0 and 360 degrees
        bearing = (bearing + 360) % 360

        return (r , bearing)

    def spawnActors(self):

        ##### SPAWN ACTORS #####
        #-----------------------------------Resembler Vehicles --------------------------------------------------------#
        # Reference Vehicle
        if self.EgoOn:
            Spawn_vutR = carla.Transform(carla.Location(x=self.x_ref+100 ,y=self.y_ref, z=1), carla.Rotation(yaw=self.yaw_ref))
        if not self.EgoOn:
            Spawn_vutR = carla.Transform(carla.Location(x=self.x_ego, y=self.y_ego, z=self.z_ref), carla.Rotation(yaw=self.yaw_ref))

        vutR_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        vutR_bp.set_attribute('role_name', 'obstacle')
        obs_vutR = self.world.spawn_actor(vutR_bp, Spawn_vutR)
        
        """Assume Last point as reference for road direction"""
        ego_lat = self.vutR_df['VUT_pos_lat'].iloc[0]
        ego_lng = self.vutR_df['VUT_pos_lng'].iloc[0]
        ego_lat2 = self.vutR_df['VUT_pos_lat'].iloc[-1]
        ego_lng2 = self.vutR_df['VUT_pos_lng'].iloc[-1]
        self.egodist, self.egobearing = self.haversine(ego_lat, ego_lng, ego_lat2, ego_lng2)
        ego_yaw = self.vutR_df['VUT_yaw'].iloc[0]

        #Spawn Actors
        obs_npc_dict = {}
        obs_ped_dict = {}   
        for actor_name in self.actor_names:

            lat_data = globals()[f'npc{actor_name}_df']['Actor_pos_lat'].iloc[0]
            lng_data = globals()[f'npc{actor_name}_df']['Actor_pos_lng'].iloc[0]
            dist, bearing = self.haversine(ego_lat, ego_lng, lat_data, lng_data)
            bearing_diff=bearing-self.egobearing
            # print(bearing_diff)

            actor_type = globals()[f'npc{actor_name}_df']['Actor_type'].iloc[0].lower()
            print(f'Actor {actor_name} is {dist} and {bearing} bearing from ego')
            x_ref=dist * math.sin(self.carlaAngle)
            y_ref=dist * math.cos(self.carlaAngle)

            x_data=-globals()[f'npc{actor_name}_df']['Actor_pos_x'].iloc[0]
            y_data=globals()[f'npc{actor_name}_df']['Actor_pos_y'].iloc[0]

            yaw_data = globals()[f'npc{actor_name}_df']['Actor_yaw'].iloc[0]
            yaw_diff = math.degrees(ego_yaw-yaw_data)

            Theta=math.atan(x_data/y_data)
            if self.ScenarioNum == 1:
                self.transformAngle=self.carlaAngle+Theta+math.radians(bearing_diff)
            if self.ScenarioNum == 2:
                self.transformAngle=self.carlaAngle+Theta+math.radians(bearing_diff)+math.radians(90)
            print('Transform angle',self.transformAngle)

            self.npc_timer_start[f"{actor_name}"] = time.time()
            self.npc_wpCount[f"{actor_name}"] = 0
            
            x_offset = x_data * math.cos(self.transformAngle) - y_data * math.sin(self.transformAngle)
            y_offset = x_data * math.sin(self.transformAngle) + y_data * math.cos(self.transformAngle)

            if actor_type == 'car':
                Spawn_npc = carla.Transform(carla.Location(x=self.x_ego+x_offset,y=self.y_ego+y_offset, z=1), carla.Rotation(yaw=self.yaw_ref-yaw_diff))
                npc_bp = self.world.get_blueprint_library().find('vehicle.audi.etron')
                npc_bp.set_attribute('role_name', 'obstacle')
                obs_npc = self.world.spawn_actor(npc_bp, Spawn_npc)
                obs_npc_dict[f'obs_npc{actor_name}'] = obs_npc
                print(f"Vehicle Spawned: {actor_name}, {actor_type}")

            elif actor_type == 'motorcycle':
                Spawn_npc = carla.Transform(carla.Location(x=self.x_ego+x_offset,y=self.y_ego+y_offset, z=1), carla.Rotation(yaw=self.yaw_ref-yaw_diff))
                npc_bp = self.world.get_blueprint_library().find('vehicle.yamaha.yzf')
                npc_bp.set_attribute('role_name', 'obstacle')
                obs_npc = self.world.spawn_actor(npc_bp, Spawn_npc)
                obs_npc_dict[f'obs_npc{actor_name}'] = obs_npc
                print(f"Vehicle Spawned: {actor_name}, {actor_type}")

            elif actor_type == 'bus':
                Spawn_npc = carla.Transform(carla.Location(x=self.x_ego+x_offset,y=self.y_ego+y_offset, z=1), carla.Rotation(yaw=self.yaw_ref-yaw_diff))
                npc_bp = self.world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
                npc_bp.set_attribute('role_name', 'obstacle')
                obs_npc = self.world.spawn_actor(npc_bp, Spawn_npc)
                obs_npc_dict[f'obs_npc{actor_name}'] = obs_npc
                print(f"Vehicle Spawned: {actor_name}, {actor_type}")

            elif actor_type == 'ped':
                Spawn_ped = carla.Transform(carla.Location(x=self.x_ego+x_offset,y=self.y_ego+y_offset, z=1), carla.Rotation(yaw=self.yaw_ref-yaw_diff))
                ped_bp = self.world.get_blueprint_library().find('walker.pedestrian.0025')
                ped_bp.set_attribute('role_name', 'ped')
                obs_ped = self.world.spawn_actor(ped_bp, Spawn_ped)
                obs_ped_dict[f'obs_ped{actor_name}'] = obs_ped
                print(f"Pedestrian Spawned: {actor_name}")

        # cam1bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # # Modify the attributes of the blueprint to set image resolution and field of view.
        # cam1bp.set_attribute('image_size_x', str(1920))
        # cam1bp.set_attribute('image_size_y', str(1080))
        # cam1bp.set_attribute('fov', str(70))
        # cam1bp.set_attribute('sensor_tick', str(1/30))
        
        
        # camera_location = carla.Location(x=0.48, z=1.22)
        # camera_transform = carla.Transform(camera_location)
        # self.recordCamera = self.world.spawn_actor(cam1bp, camera_transform, attach_to=obs_vutR)
        # self.recordCamera.listen(lambda image: image.save_to_disk('output/%06d.png'%image.frame_number))
        

        return (obs_vutR, obs_npc_dict, obs_ped_dict)
    

    
    def getWaypoints(self):
        #-----------------------------------Resembler Vehicles --------------------------------------------------------#
        # npc_wp_dict={}
        # transformAngle=math.radians(26.727764092774528)

        #Transform resdata for SC2
        if self.ScenarioNum ==2:
            self.transformAngle=self.transformAngle+math.radians(68)

        for step in self.vutR_df['Step_number']:
            
            vut_x= -self.vutR_df['VUT_pos_x'].iloc[step]
            vut_y= self.vutR_df['VUT_pos_y'].iloc[step]
            vut_yaw= self.vutR_df['VUT_yaw'].iloc[step]
            vut_x_offset = vut_x * math.cos(self.transformAngle) - vut_y * math.sin(self.transformAngle)
            vut_y_offset = vut_x * math.sin(self.transformAngle) + vut_y * math.cos(self.transformAngle)

            # if self.ScenarioNum ==2:
            #     vut_x_offset=vut_x
            #     vut_y_offset=vut_y

            vut_wp = carla.Transform(carla.Location(x=self.x_ego+vut_x_offset, y=self.y_ego+vut_y_offset,z=0.54),carla.Rotation(yaw=self.yaw_ref))
            self.vutR_wp.append(vut_wp)

        #Transform resdata for SC2
        if self.ScenarioNum ==2:
            self.transformAngle=self.transformAngle-math.radians(28)

        for actor_name in self.actor_names:
            # print(actor_name)
            globals()[f'npc{actor_name}_wp']=[]

            actor_type = self.npcR_df['Actor_type'].iloc[0].lower()

            if actor_type == 'car' or actor_type == 'motorcycle' or actor_type == 'bus' :
                for step in globals()[f'npc{actor_name}_df']['Step_number']:
                    x_data = -globals()[f'npc{actor_name}_df']['Actor_pos_x'].iloc[step]
                    y_data = globals()[f'npc{actor_name}_df']['Actor_pos_y'].iloc[step]
                    yaw_data = globals()[f'npc{actor_name}_df']['Actor_yaw'].iloc[step]

                    x_offset = x_data * math.cos(self.transformAngle) - y_data * math.sin(self.transformAngle)
                    y_offset = x_data * math.sin(self.transformAngle) + y_data * math.cos(self.transformAngle)
                    yaw_diff = math.degrees(vut_yaw-yaw_data)

                    wp = carla.Transform(carla.Location(x=self.x_ego+x_offset, y=self.y_ego+y_offset,z=0.54),carla.Rotation(yaw=self.yaw_ref-yaw_diff))
                    globals()[f'npc{actor_name}_wp'].append(wp)

        """Show Waypoints"""
        if self.showWP:
            for actor_name in self.actor_names:
                for wp in globals()[f'npc{actor_name}_wp']:
                    self.client.get_world().debug.draw_string(wp.location, f'{actor_name}', draw_shadow=False,
                                        color=carla.Color(r=255, g=0, b=0), life_time=50,
                                        persistent_lines=True)

            for wp in self.vutR_wp:
                self.client.get_world().debug.draw_string(wp.location, 'V', draw_shadow=False,
                                    color=carla.Color(r=255, g=0, b=0), life_time=50,
                                    persistent_lines=True)

    def drawWaypoints(self, road_id, life_time):
        
        for waypoint in self.waypoints:
            if(waypoint.road_id == road_id):
                self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                    persistent_lines=True)

    def executeScenario(self, ego_transform, start_time):
        # (obs_npc1, obs_npc2, obs_ped1, obs_npc3, obs_ped2, obs_cones, obs_npc4, obs_npc5, obs_npc6, obs_ped3, obs_ped4, obs_npcA, obs_npcB, obs_npcC, obs_npcD, obs_vutR, obs_npc_dict, obs_ped_dict) = self.actorList
        (obs_vutR, obs_npc_dict, obs_ped_dict) = self.actorList

        elapsed_time = time.perf_counter() - start_time
        
        if self.ScenarioNum ==1:
            if self.EgoOn:
                trigDist = 32
            else:
                trigDist = 100

        #-----------------------------------Resembler Vehicles --------------------------------------------------------#

        vut_controller = VehiclePIDControllerLoc(obs_vutR, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        for actor_name in self.actor_names:
            actor_type = self.npcR_df['Actor_type'].iloc[0].lower()

            if actor_type == 'car' or actor_type == 'motorcycle' or actor_type == 'bus':
                globals()[f'npc{actor_name}_controller'] = VehiclePIDControllerLoc(obs_npc_dict["obs_npc"+actor_name], args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
                
                if not self.scenarioStartFlag:
                    #print(obs_npc_dict.keys(),actor_name)
            
                    transf= obs_npc_dict["obs_npc"+actor_name].get_transform()
                    #for step in globals()[f'npc{actor_name}_df']['Step_number']:
                    
                    #NPC Distance Finder
                    #print (ego_transform.location.x, transf.location.x,ego_transform.location.x - transf.location.x)
                    xdif = ego_transform.location.x - transf.location.x
                    #print(globals()[f'x_diff_npc{actor_name}'],globals()[f'y_diff_npc{actor_name}'])
                    ydif = ego_transform.location.y - transf.location.y
                    dist = math.sqrt(xdif**2 + ydif**2)
                    """dist = trigger distance -> automise"""
                    if dist<=trigDist:
                        self.scenarioStartFlag = True



                if self.scenarioStartFlag:
            
                    # ped_timer = 0.0
                    # print(time.time()-ped_timer_start)
                    if time.time()-self.vut_timer_start >0.1 and self.vut_wpCount <self.totalCount:
                        self.vut_wpCount = self.vut_wpCount+1
                        print(self.vut_wpCount)
                        self.vut_timer_start = time.time()

                    if time.time()-self.npc_timer_start[actor_name] >0.1 and self.npc_wpCount[actor_name]<self.totalCount:
                        self.npc_wpCount[actor_name] = self.npc_wpCount[actor_name]+1  
                        # print(self.npc_wpCount[actor_name])
                        
                        self.npc_timer_start[actor_name] = time.time()

                    if self.npc_wpCount[actor_name] < self.totalCount:
                        # print(f'npc{actor_name} moving')
                        spd = globals()[f'npc{actor_name}_df']['Actor_vel_abs'].iloc[self.npc_wpCount[actor_name]]
                        nextwp =    globals()[f'npc{actor_name}_wp'][self.npc_wpCount[actor_name]]

                    else:
                        spd = 0.0
                        nextwp =    globals()[f'npc{actor_name}_wp'][-1]

                    if self.vut_wpCount < len(self.vutR_wp):

                        vut_spd = self.vutR_df['VUT_vel_abs'].iloc[self.vut_wpCount]
                        vut_nextwp =    self.vutR_wp[self.vut_wpCount]

                    else:
                        vut_spd = 0.0
                        vut_nextwp =    self.vutR_wp[len(self.vutR_wp)-1] 
                             
                 
                else:
                    spd= 0.0
                    nextwp =    globals()[f'npc{actor_name}_wp'][0]

                    vut_spd =    0.0
                    vut_nextwp =    self.vutR_wp[0]


                globals()[f'npc{actor_name}_control_signal'] = globals()[f'npc{actor_name}_controller'].run_step(spd*3.6,nextwp)
                obs_npc_dict["obs_npc"+actor_name].apply_control(carla.VehicleControl(steer = globals()[f'npc{actor_name}_control_signal'].steer/1.5, throttle = globals()[f'npc{actor_name}_control_signal'].throttle, brake = globals()[f'npc{actor_name}_control_signal'].brake))    
                # print(globals()[f'npc{actor_name}_control_signal'].throttle)

                #VUT controller
                if not self.EgoOn:
                    vut_control_signal = vut_controller.run_step(vut_spd*3.6,vut_nextwp)
                    obs_vutR.apply_control(carla.VehicleControl(steer = vut_control_signal.steer/1.5, throttle = vut_control_signal.throttle, brake = vut_control_signal.brake))    
                    vut_transform=obs_vutR.get_transform()

                    # self.spectator.set_transform(carla.Transform(vut_transform.location + carla.Location(x=0.48 * math.cos(math.radians(vut_transform.rotation.yaw)),
                    #                                                                                      y=0.48 * math.sin(math.radians(vut_transform.rotation.yaw)),
                    #                                                                                      z=1.22),carla.Rotation(pitch=-10, yaw=vut_transform.rotation.yaw)))
                    
                    self.spectator.set_transform(carla.Transform(vut_transform.location + carla.Location(x=-10 * math.cos(math.radians(vut_transform.rotation.yaw)),
                                                                                                        y=-10 * math.sin(math.radians(vut_transform.rotation.yaw)),
                                                                                                        z=5),carla.Rotation(pitch=-15, yaw=vut_transform.rotation.yaw)))

        #print(globals()[f'relDist_npc{actor_name}'])
               
                #Ego Speed

                # #NPC trigger
                # if self.npcA_wpCount < len(self.npcA_wp)-1:
                #     # npcA_nextWP_dist = math.sqrt((npcA_transform.location.x-self.npcA_wp[self.npcA_wpCount].transform.location.x)**2 + (npcA_transform.location.y-self.npcA_wp[self.npcA_wpCount].transform.location.y)**2)
                #     npcA_nextWP_dist = math.sqrt((npcA_transform.location.x-self.npcA_wp[self.npcA_wpCount].location.x)**2 + (npcA_transform.location.y-self.npcA_wp[self.npcA_wpCount].location.y)**2)
                #     if npcA_nextWP_dist < 1:
                #         self.npcA_wpCount = self.npcA_wpCount+1

                #     if relDist_npcD<=1000:
                #         self.npcA_spd=8.3333 #30km/h
                        
                # if self.npcA_wpCount < len(self.npcA_wp)-1:        
                #     npcA_control_signal = npcA_controller.run_step(self.npcA_spd*3.6, self.npcA_wp[self.npcA_wpCount])

                # else:
                #     self.npcA_spd = 0.0
                #     npcA_control_signal = npcA_controller.run_step(self.npcA_spd*3.6, self.npcA_wp[len(self.npcA_wp)-1])   


        # #-----------------------------------Scenario 1--------------------------------------------------------#
        # npc2_controller = VehiclePIDControllerLoc(obs_npc2, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        # npc2_transform = obs_npc2.get_transform()
        # ped1_transform = obs_ped1.get_transform()

        # #NPC2 Distance Finder
        # x_diff_npc2 = ego_transform.location.x - npc2_transform.location.x
        # y_diff_npc2 = ego_transform.location.y - npc2_transform.location.y
        # relDist_npc2 = math.sqrt(x_diff_npc2**2 + y_diff_npc2**2)

        # # npc2_targetDist = math.sqrt((npc2_transform.location.x-npc2_wp.transform.location.x)**2 + (npc2_transform.location.y-npc2_wp.transform.location.y)**2)
        # # print(npc2_travelDist)

        # #PED1 Distance Finder
        # x_diff_ped1 = ego_transform.location.x - ped1_transform.location.x
        # y_diff_ped1 = ego_transform.location.y - ped1_transform.location.y
        # relDist_ped1 = math.sqrt(x_diff_ped1**2 + y_diff_ped1**2)

        # ped1_travelDist = math.sqrt((ped1_transform.location.x-self.ped1_init.location.x)**2 + (ped1_transform.location.y-self.ped1_init.location.y)**2)

        # #Ped1 new trigger
        # if relDist_ped1<=100:
            
        #     # ped_timer = 0.0
        #     # print(time.time()-ped_timer_start)
        #     if time.time()-self.ped_timer_start >1 :
        #         self.ped1_wpCount = self.ped1_wpCount+1                            
        #         # print(self.ped1_wpCount)
        #         self.ped_timer_start = time.time()

        #     if self.ped1_wpCount < 35:
        #         # print('SC1: Ped moving')
                
        #         self.ped1_spd = self.ped1_df['Speedx2'][self.ped1_wpCount]
        #         pedDataAngle = math.degrees(self.ped1_df['Heading'][self.ped1_wpCount])
            
        #         # print(pedDataAngle)
        #         # ped1_spd = WP2pedspd
        #         # pedDataAngle = WP2pedheading # << Negative , >> Positive   
        #     else:
        #         self.ped1_spd = 0.0
        #         pedDataAngle = 0 # << Negative , >> Positive  
            
        # else:
        #     self.ped1_spd = 0.0
        #     pedDataAngle = 0 # << Negative , >> Positive

        # #Ped1 Controller
        # obs_ped1.apply_control(carla.WalkerControl( 
        #         speed=self.ped1_spd,
        #         direction=carla.Vector3D(self.xTarg, self.yTarg, 0.0),
        #     ))

        # #NPC2 trigger
        # if self.npc2_wpCount < len(self.npc2_wp)-1:
        #     npc2_nextWP_dist = math.sqrt((npc2_transform.location.x-self.npc2_wp[self.npc2_wpCount].location.x)**2 + (npc2_transform.location.y-self.npc2_wp[self.npc2_wpCount].location.y)**2)

        #     if npc2_nextWP_dist < 1:
        #         self.npc2_wpCount = self.npc2_wpCount+1

        #     if relDist_npc2<=40:
        #         self.npc2_spd=8.0
        #         # print('SC1: NPC moving')
        # if self.npc2_wpCount < len(self.npc2_wp)-1:        
        #     npc2_control_signal = npc2_controller.run_step(self.npc2_spd*3.6, self.npc2_wp[self.npc2_wpCount])
        # else:
        #     self.npc2_spd = 0.0
        #     npc2_control_signal = npc2_controller.run_step(self.npc2_spd*3.6, self.npc2_wp[len(self.npc2_wp)-1])    
        
        # #NPC2 Controller                        
        # obs_npc2.apply_control(carla.VehicleControl(steer = npc2_control_signal.steer/1.5, throttle = npc2_control_signal.throttle, brake = npc2_control_signal.brake))

        # #Traffic Light Controller
        # # sc1_trafficlight= self.world.get_traffic_light(carla.Location(x=-71.19, y=65.06, z=2))
        # # # sc1_trafficlight= self.world.get_traffic_light()
        # # sc1_trafficlight(carla.TrafficLight.set_state.Green)


        # #-----------------------------------Scenario 2--------------------------------------------------------#
        # npc3_controller = VehiclePIDControllerLoc(obs_npc3, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        # npc3_transform = obs_npc3.get_transform()
        # ped2_transform = obs_ped2.get_transform()

        # #NPC3 Distance Finder
        # x_diff_npc3 = ego_transform.location.x - npc3_transform.location.x
        # y_diff_npc3 = ego_transform.location.y - npc3_transform.location.y
        # relDist_npc3 = math.sqrt(x_diff_npc3**2 + y_diff_npc3**2)

        # #PED2 Distance Finder
        # x_diff_ped2 = ego_transform.location.x - ped2_transform.location.x
        # y_diff_ped2 = ego_transform.location.y - ped2_transform.location.y
        # relDist_ped2 = math.sqrt(x_diff_ped2**2 + y_diff_ped2**2)

        # ped2_travelDist = math.sqrt((ped2_transform.location.x-self.ped2_init.location.x)**2 + (ped2_transform.location.y-self.ped2_init.location.y)**2)

        # #NPC3 trigger
        # if self.npc3_wpCount < len(self.npc3_wp)-1:
        #     npc3_nextWP_dist = math.sqrt((npc3_transform.location.x-self.npc3_wp[self.npc3_wpCount].location.x)**2 + (npc3_transform.location.y-self.npc3_wp[self.npc3_wpCount].location.y)**2)

        #     if npc3_nextWP_dist < 1:
        #         self.npc3_wpCount = self.npc3_wpCount+1

        #     if relDist_npc3<=40:
        #         self.npc3_spd=6.9 #25km/h
        #         # print('SC2: NPC moving')
        # if self.npc3_wpCount < len(self.npc3_wp)-1:        
        #     npc3_control_signal = npc3_controller.run_step(self.npc3_spd*3.6, self.npc3_wp[self.npc3_wpCount])
        # else:
        #     self.npc3_spd = 0
        #     npc3_control_signal = npc3_controller.run_step(self.npc3_spd*3.6, self.npc3_wp[len(self.npc3_wp)-1])    
                                
        # obs_npc3.apply_control(carla.VehicleControl(steer = npc3_control_signal.steer, throttle = npc3_control_signal.throttle, brake = npc3_control_signal.brake))

        # #Ped2 trigger
        # if relDist_ped2<=50 and ped2_travelDist <=20:
        #     # print('SC2: Ped moving')
        #     self.ped2_spd=1.0
            
        # else:
        #     self.ped2_spd=0.0

        # #Ped2 Controller
        # obs_ped2.apply_control(carla.WalkerControl(
        #         speed=self.ped2_spd,
        #         direction=carla.Vector3D(1.0, 1.0, 0.0),
        #     ))

        # #-----------------------------------Scenario 3--------------------------------------------------------#
        # npc4_controller = VehiclePIDControllerLoc(obs_npc4, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})        
        # npc5_controller = VehiclePIDControllerLoc(obs_npc5, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        # npc4_transform = obs_npc4.get_transform()
        # npc5_transform = obs_npc5.get_transform()

        # #NPC4 Distance Finder
        # x_diff_npc4 = ego_transform.location.x - npc4_transform.location.x
        # y_diff_npc4 = ego_transform.location.y - npc4_transform.location.y
        # relDist_npc4 = math.sqrt(x_diff_npc4**2 + y_diff_npc4**2)

        # #NPC5 Distance Finder
        # x_diff_npc5 = ego_transform.location.x - npc5_transform.location.x
        # y_diff_npc5 = ego_transform.location.y - npc5_transform.location.y
        # relDist_npc5 = math.sqrt(x_diff_npc5**2 + y_diff_npc5**2)

        # #NPC4 trigger
        # if self.npc4_wpCount < len(self.npc4_wp)-1:
        #     npc4_nextWP_dist = math.sqrt((npc4_transform.location.x-self.npc4_wp[self.npc4_wpCount].location.x)**2 + (npc4_transform.location.y-self.npc4_wp[self.npc4_wpCount].location.y)**2)

        #     if npc4_nextWP_dist < 1:
        #         self.npc4_wpCount = self.npc4_wpCount+1

        #     if relDist_npc4<=13:
        #         self.npc4_spd=13.9 #50km/h
        #         # print('SC3: NPC ambulance moving')
        # if self.npc4_wpCount < len(self.npc4_wp)-1:        
        #     npc4_control_signal = npc4_controller.run_step(self.npc4_spd*3.6, self.npc4_wp[self.npc4_wpCount])
        # else:
        #     self.npc4_spd = 0
        #     npc4_control_signal = npc4_controller.run_step(self.npc4_spd*3.6, self.npc4_wp[len(self.npc4_wp)-1])    
                                
        # obs_npc4.apply_control(carla.VehicleControl(steer = npc4_control_signal.steer, throttle = npc4_control_signal.throttle, brake = npc4_control_signal.brake))

        # #NPC5 trigger
        # if self.npc5_wpCount < len(self.npc5_wp)-1:
        #     npc5_nextWP_dist = math.sqrt((npc5_transform.location.x-self.npc5_wp[self.npc5_wpCount].location.x)**2 + (npc5_transform.location.y-self.npc5_wp[self.npc5_wpCount].location.y)**2)

        #     if npc5_nextWP_dist < 1:
        #         self.npc5_wpCount = self.npc5_wpCount+1
        #         # self.npc5_spd=10 #20km/h

        #     if relDist_npc5<=30:
        #         self.npc5_spd=5.5 #20km/h
        #         # print('SC3: NPC bike moving')
        # if self.npc5_wpCount < len(self.npc5_wp)-1:        
        #     npc5_control_signal = npc5_controller.run_step(self.npc5_spd*3.6, self.npc5_wp[self.npc5_wpCount])
        # else:
        #     self.npc5_spd = 0
        #     npc5_control_signal = npc5_controller.run_step(self.npc5_spd*3.6, self.npc5_wp[len(self.npc5_wp)-1])    
                                
        # obs_npc5.apply_control(carla.VehicleControl(steer = npc5_control_signal.steer/1.5, throttle = npc5_control_signal.throttle, brake = npc5_control_signal.brake))


        # #-----------------------------------Scenario 4--------------------------------------------------------#
        # ped3_transform = obs_ped3.get_transform()
        # ped4_transform = obs_ped4.get_transform()

        # #PED3 Distance Finder
        # x_diff_ped3 = ego_transform.location.x - ped3_transform.location.x
        # y_diff_ped3 = ego_transform.location.y - ped3_transform.location.y
        # relDist_ped3 = math.sqrt(x_diff_ped3**2 + y_diff_ped3**2)

        # ped3_travelDist = math.sqrt((ped3_transform.location.x-self.ped3_init.location.x)**2 + (ped3_transform.location.y-self.ped3_init.location.y)**2)
        
        # #PED4 Distance Finder
        # x_diff_ped4 = ego_transform.location.x - ped4_transform.location.x
        # y_diff_ped4 = ego_transform.location.y - ped4_transform.location.y
        # relDist_ped4 = math.sqrt(x_diff_ped4**2 + y_diff_ped4**2)

        # ped4_travelDist = math.sqrt((ped4_transform.location.x-self.ped4_init.location.x)**2 + (ped4_transform.location.y-self.ped4_init.location.y)**2)

        # #Ped3 and Ped4 trigger
        # if relDist_ped3<=30 and ped3_travelDist <=18:
        #     # print('SC2: Ped moving')
        #     self.ped3_spd=1.38889
        #     self.ped4_spd=1.66667
            
        # else:
        #     self.ped3_spd=0.0
        #     self.ped4_spd=0.0

        # #Ped3 Controller
        # obs_ped3.apply_control(carla.WalkerControl(
        #         speed=self.ped3_spd,
        #         direction=carla.Vector3D(1.0, 1.0, 0.0),
        #     ))

        # #Ped4 Controller
        # obs_ped4.apply_control(carla.WalkerControl(
        #         speed=self.ped4_spd,
        #         direction=carla.Vector3D(1.0, 1.0, 0.0),
        #     ))