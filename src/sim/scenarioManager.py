import carla
import random
import sys
import numpy as np
from ApolloBridgeClient import *
import math
import modules
from time import sleep
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from threading import Thread
from agents.navigation.controller import VehiclePIDController
from agents.navigation.controller_wpLoc import VehiclePIDController as VehiclePIDControllerLoc
import time
import pandas as pd

class ScenarioManager():
    def __init__(self, CARLA_HOST, CARLA_PORT):
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = self.client.get_world()
        self.carlamap = self.world.get_map()
        self.spectator = self.world.get_spectator()

        # Read WP2 pedestrian dataframe
        self.ped1_df = pd.read_csv('WP2_TrafficCrossingIntersectionData.csv', usecols = ['Heading','Speedx2'])
        
        self.waypoints = self.client.get_world().get_map().generate_waypoints(distance=1.0)

        self.actorList = self.spawnActors()
	    #### Wait before executing scenario ####
        # sleep(1.0)

        (obs_npc1, obs_npc2, obs_ped1, obs_npc3, obs_ped2, obs_cones, obs_npc4, obs_npc5, obs_npc6, obs_ped3, obs_ped4, obs_npcA, obs_npcB, obs_npcC, obs_npcD) = self.actorList

        #initialise scenario

        self.xNorth =  5/7
        self.yNorth = -5/7
        pedDataAngle = 0
        pedAngle = math.radians(pedDataAngle) #rads
        self.xTarg = self.xNorth*math.cos(pedAngle) - self.yNorth*math.sin(pedAngle)
        self.yTarg = self.xNorth*math.sin(pedAngle) + self.yNorth*math.cos(pedAngle)

        #-----------------------------------Oncoming --------------------------------------------------------#
        
        # NPC waypoint declarations
        self.npcOC_wp = []
        self.npcA_wp = []
        self.npcB_wp = []
        self.npcC_wp = []
        self.npcD_wp = []

        self.npcA_wpCount = 0
        self.npcB_wpCount = 0
        self.npcC_wpCount = 0
        self.npcD_wpCount = 0
        
        self.npcA_spd=0.0
        self.npcB_spd=0.0
        self.npcC_spd=0.0
        self.npcD_spd=0.0
        
        #-----------------------------------Scenario 1--------------------------------------------------------#
        self.ped1_wpCount = 0
        self.ped_timer_start=time.time()
        
        # NPC waypoint declarations
        self.npc2_wp = []
        self.npc2_wpCount = 0
        
        self.npc2_spd=0.0
        self.ped1_spd=0.0
        self.pedDataAngle =0.0
        self.ped1_init = obs_ped1.get_transform()
        
        #-----------------------------------Scenario 2--------------------------------------------------------#
        self.npc3_spd=0.0
        self.ped2_spd=0.0
        
        # NPC waypoint declarations
        self.npc3_wp = []
        self.npc3_wpCount = 0

        self.ped2_init = obs_ped2.get_transform()
        
        #-----------------------------------Scenario 3--------------------------------------------------------#
        self.npc4_spd=0.0
        self.npc5_spd=0.0

        # NPC waypoint declarations
        self.npc4_wp = []
        self.npc5_wp = []

        self.npc4_wpCount = 0
        self.npc5_wpCount = 0

        #-----------------------------------Scenario 4--------------------------------------------------------#
        self.ped3_init = obs_ped3.get_transform()
        self.ped4_init = obs_ped4.get_transform()

        # NPC waypoint declarations

        self.getWaypoints()
        # self.drawWaypoints(road_id, life_time)

    def spawnActors(self):

        ##### SPAWN ACTORS #####
        #-----------------------------------Oncoming Vehicles --------------------------------------------------------#
        Spawn_npcA = carla.Transform(carla.Location(x=300.00+0*4.712, y=-214.70-0*3.714, z=2), carla.Rotation(yaw=142.5))
        Spawn_npcB = carla.Transform(carla.Location(x=300.00+1*4.712, y=-214.70-1*3.714, z=2), carla.Rotation(yaw=142.5))
        Spawn_npcC = carla.Transform(carla.Location(x=540.0-2*4.712, y=-404.0+2*3.714, z=2), carla.Rotation(yaw=142.5))
        Spawn_npcD = carla.Transform(carla.Location(x=-226.0, y=198.0, z=1), carla.Rotation(yaw=142.5))

        npcA_bp = self.world.get_blueprint_library().find('vehicle.tesla.cybertruck')
        npcB_bp = self.world.get_blueprint_library().find('vehicle.jeep.wrangler_rubicon')
        npcC_bp = self.world.get_blueprint_library().find('vehicle.mercedes.coupe_2020')
        npcD_bp = self.world.get_blueprint_library().find('vehicle.volkswagen.t2_2021')
        
        npcA_bp.set_attribute('role_name', 'obstacle')
        npcB_bp.set_attribute('role_name', 'obstacle')
        npcC_bp.set_attribute('role_name', 'obstacle')
        npcD_bp.set_attribute('role_name', 'obstacle')
        
        obs_npcA = self.world.spawn_actor(npcA_bp, Spawn_npcA)
        obs_npcB = self.world.spawn_actor(npcB_bp, Spawn_npcB)
        obs_npcC = self.world.spawn_actor(npcC_bp, Spawn_npcC)
        obs_npcD = self.world.spawn_actor(npcD_bp, Spawn_npcD)

        #-----------------------------------Scenario 1--------------------------------------------------------#
        Spawn_npc1 = carla.Transform(carla.Location(x=-65.338, y=69.467, z=2), carla.Rotation(yaw=328.5))
        Spawn_npc2 = carla.Transform(carla.Location(x=-47.540, y=58.278, z=2), carla.Rotation(yaw=148.5))
        Spawn_ped1 = carla.Transform(carla.Location(x=-74.742, y=57.240, z=2), carla.Rotation(yaw=328.5))
        Spawn_stped1 = carla.Transform(carla.Location(x=-72.742, y=57.240, z=2), carla.Rotation(yaw=328.5))
        Spawn_stped2 = carla.Transform(carla.Location(x=-73.742, y=57.240, z=2), carla.Rotation(yaw=328.5))
        
        npc1_bp = self.world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
        npc2_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        ped1_bp = self.world.get_blueprint_library().find('walker.pedestrian.0029')
        stped1_bp = self.world.get_blueprint_library().find('walker.pedestrian.0010')
        stped2_bp = self.world.get_blueprint_library().find('walker.pedestrian.0046')
        
        
        npc1_bp.set_attribute('role_name', 'obstacle')
        npc2_bp.set_attribute('role_name', 'obstacle')
        ped1_bp.set_attribute('role_name', 'ped')
        stped1_bp.set_attribute('role_name', 'ped')
        stped2_bp.set_attribute('role_name', 'ped')

        
        obs_npc1 = self.world.spawn_actor(npc1_bp, Spawn_npc1)
        obs_npc2 = self.world.spawn_actor(npc2_bp, Spawn_npc2)
        obs_ped1 = self.world.spawn_actor(ped1_bp, Spawn_ped1)
        obs_stped1 = self.world.spawn_actor(stped1_bp, Spawn_stped1)
        obs_stped2 = self.world.spawn_actor(stped2_bp, Spawn_stped2)

        #-----------------------------------Scenario 2--------------------------------------------------------#
        Spawn_npc3 = carla.Transform(carla.Location(x=-7.022 ,y=17.865, z=2), carla.Rotation(yaw=322.5))
        Spawn_ped2 = carla.Transform(carla.Location(x=93.660, y=-65.238, z=2), carla.Rotation(yaw=52.5))

        npc3_bp = self.world.get_blueprint_library().find('vehicle.carlamotors.carlacola')
        ped2_bp = self.world.get_blueprint_library().find('walker.pedestrian.0025')

        npc3_bp.set_attribute('role_name', 'obstacle')
        ped2_bp.set_attribute('role_name', 'ped')

        obs_npc3 = self.world.spawn_actor(npc3_bp, Spawn_npc3)
        obs_ped2 = self.world.spawn_actor(ped2_bp, Spawn_ped2)

        #Cones
        Spawn_cone1 = carla.Transform(carla.Location(x=31.714, y=-13.781, z=0.57), carla.Rotation(yaw=232.5))
        Spawn_cone2 = carla.Transform(carla.Location(x=34.950, y=-14.965, z=0.57), carla.Rotation(yaw=232.5))
        Spawn_cone3 = carla.Transform(carla.Location(x=37.936, y=-16.265, z=0.57), carla.Rotation(yaw=232.5))
        Spawn_cone4 = carla.Transform(carla.Location(x=43.542, y=-20.606, z=0.54), carla.Rotation(yaw=232.5))
        Spawn_cone5 = carla.Transform(carla.Location(x=47.141, y=-24.575, z=0.49), carla.Rotation(yaw=232.5))
        Spawn_cone6 = carla.Transform(carla.Location(x=49.977, y=-28.340, z=0.44), carla.Rotation(yaw=232.5))
        
        cone1_bp = self.world.get_blueprint_library().find('static.prop.constructioncone')
        cone2_bp = self.world.get_blueprint_library().find('static.prop.constructioncone')
        cone3_bp = self.world.get_blueprint_library().find('static.prop.constructioncone')
        cone4_bp = self.world.get_blueprint_library().find('static.prop.constructioncone')
        cone5_bp = self.world.get_blueprint_library().find('static.prop.constructioncone')
        cone6_bp = self.world.get_blueprint_library().find('static.prop.constructioncone')
        
        cone1_bp.set_attribute('role_name', 'traffic_cone')
        cone2_bp.set_attribute('role_name', 'traffic_cone')
        cone3_bp.set_attribute('role_name', 'traffic_cone')
        cone4_bp.set_attribute('role_name', 'traffic_cone')
        cone5_bp.set_attribute('role_name', 'traffic_cone')
        cone6_bp.set_attribute('role_name', 'traffic_cone')
        # print("Spawn cones")

        obs_cones = []

        obs_cones.append(self.world.spawn_actor(cone1_bp, Spawn_cone1))
        obs_cones.append(self.world.spawn_actor(cone2_bp, Spawn_cone2))
        obs_cones.append(self.world.spawn_actor(cone3_bp, Spawn_cone3))
        obs_cones.append(self.world.spawn_actor(cone4_bp, Spawn_cone4))
        obs_cones.append(self.world.spawn_actor(cone5_bp, Spawn_cone5))
        obs_cones.append(self.world.spawn_actor(cone6_bp, Spawn_cone6))
        

        #-----------------------------------Scenario 3--------------------------------------------------------#
        Spawn_npc4 = carla.Transform(carla.Location(x=-15.022+118.04,y=17.865-92.56, z=2), carla.Rotation(yaw=322.5))
        Spawn_npc5 = carla.Transform(carla.Location(x=-11.022+118.04+78.69,y=17.865-92.56-61.70, z=2), carla.Rotation(yaw=322.5))
        # Spawn_npc5 = carla.Transform(carla.Location(x=-9.022+118.04+51.15,y=17.865-92.56-40.11, z=2), carla.Rotation(yaw=322.5))

        npc4_bp = self.world.get_blueprint_library().find('vehicle.ford.ambulance')
        # npc5_bp = self.world.get_blueprint_library().find('vehicle.micro.microlino')
        npc5_bp = self.world.get_blueprint_library().find('vehicle.diamondback.century')

        npc4_bp.set_attribute('role_name', 'obstacle')
        npc5_bp.set_attribute('role_name', 'bicycle')
        
        obs_npc4 = self.world.spawn_actor(npc4_bp, Spawn_npc4)
        obs_npc5 = self.world.spawn_actor(npc5_bp, Spawn_npc5)
        obs_npc4.set_light_state(carla.VehicleLightState.Special1)

        #-----------------------------------Scenario 4--------------------------------------------------------#
        Spawn_npc6 = carla.Transform(carla.Location(x=390.30+27.488-4.7, y=-290.15-21.665, z=2), carla.Rotation(yaw=322.5))
        Spawn_ped3 = carla.Transform(carla.Location(x=390.30+27.488-3.8, y=-290.15-21.665-11.3, z=2), carla.Rotation(yaw=52.5))
        Spawn_ped4 = carla.Transform(carla.Location(x=390.30+27.488-2.5, y=-290.15-21.665-9.5, z=2), carla.Rotation(yaw=52.5))

        npc6_bp = self.world.get_blueprint_library().find('vehicle.nissan.patrol_2021')
        ped3_bp = self.world.get_blueprint_library().find('walker.pedestrian.0037')
        ped4_bp = self.world.get_blueprint_library().find('walker.pedestrian.0009')

        npc6_bp.set_attribute('role_name', 'obstacle')
        ped3_bp.set_attribute('role_name', 'ped')
        ped4_bp.set_attribute('role_name', 'ped')

        obs_npc6 = self.world.spawn_actor(npc6_bp, Spawn_npc6)
        obs_ped3 = self.world.spawn_actor(ped3_bp, Spawn_ped3)
        obs_ped4 = self.world.spawn_actor(ped4_bp, Spawn_ped4)

        return (obs_npc1, obs_npc2, obs_ped1, obs_npc3, obs_ped2, obs_cones, obs_npc4, obs_npc5, obs_npc6, obs_ped3, obs_ped4, obs_npcA, obs_npcB, obs_npcC, obs_npcD)
    
    def getWaypoints(self):
        #-----------------------------------Oncoming Vehicles --------------------------------------------------------#
        # npcOC_waypoints = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 50):
        #         npcOC_waypoints.append(waypoint)

        npcA_wp1 = carla.Transform(carla.Location(x=73.43, y=-32.24, z=0.540000), carla.Rotation(yaw=-38.248478))
        npcA_wp2 = carla.Transform(carla.Location(x=9.54, y=10.52, z=0.540000), carla.Rotation(yaw=-38.248478))
        npcA_wp3 = carla.Transform(carla.Location(x=-36.973751, y=54.583321, z=0.540000), carla.Rotation(yaw=-38.248478))
        npcA_wp4 = carla.Transform(carla.Location(x=-38.973751, y=52.583321, z=0.540000), carla.Rotation(yaw=-38.248478))
        
        # npcA_wp31 = npcOC_waypoints[11]
        # npcA_wp2 = npcOC_waypoints[21]
        # npcA_wp1 = npcOC_waypoints[52]
        self.npcA_wp = [npcA_wp1,npcA_wp2,npcA_wp3,npcA_wp4]

        npcB_wp1 = carla.Transform(carla.Location(x=92.795494, y=-52.169624, z=0.170165), carla.Rotation(yaw=141.751526))
        npcB_wp2 = carla.Transform(carla.Location(x=65.119583, y=-25.896183, z=0.320277), carla.Rotation(yaw=141.751526))
        npcB_wp3 = carla.Transform(carla.Location(x=50.387497, y=-18.739677, z=0.398604), carla.Rotation(yaw=141.751526))
        npcB_wp4 = carla.Transform(carla.Location(x=37.822159, y=-8.834510, z=0.476566), carla.Rotation(yaw=141.751526))
        npcB_wp5 = carla.Transform(carla.Location(x=-39.140507, y=51.834652, z=0.540000), carla.Rotation(yaw=141.751526))

        # npcB_wp51 = npcOC_waypoints[12]
        # npcB_wp41 = npcOC_waypoints[502]
        # npcB_wp31 = npcOC_waypoints[582]
        # npcB_wp21 = npcOC_waypoints[661]
        # npcB_wp11 = npcOC_waypoints[852]

        self.npcB_wp = [npcB_wp1,npcB_wp2,npcB_wp3,npcB_wp4,npcB_wp5]

        
        npcC_wp1 = carla.Transform(carla.Location(x=149.935593, y=-92.756073, z=0.170000), carla.Rotation(yaw=141.751526))
        npcC_wp2 = carla.Transform(carla.Location(x=116.355492, y=-70.741814, z=0.170000), carla.Rotation(yaw=141.751526))       
        npcC_wp3 = carla.Transform(carla.Location(x=102.815582, y=-55.611691, z=0.170000), carla.Rotation(yaw=141.751526))      
        npcC_wp4 = carla.Transform(carla.Location(x=-32.857841, y=46.882069, z=0.540000), carla.Rotation(yaw=141.751526))
        npcC_wp5 = carla.Transform(carla.Location(x=-35.999172, y=49.358360, z=0.540000), carla.Rotation(yaw=141.751526))
  
        self.npcC_wp = [npcC_wp1,npcC_wp2,npcC_wp3,npcC_wp4,npcC_wp5]

        # self.npcD_wp.append(npcOC_waypoints[32])
        # self.npcD_wp.append(npcOC_waypoints[52])

        # for wp in self.npcA_wp:
        #     self.client.get_world().debug.draw_string(wp.location, 'O', draw_shadow=False,
        #                         color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                         persistent_lines=True)
        
        #-----------------------------------Scenario 1--------------------------------------------------------#
        # npc2_waypoints = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 64):
        #         npc2_waypoints.append(waypoint)


        # self.npc2_wp.append(npc2_waypoints[3])
        # self.npc2_wp.append(npc2_waypoints[123])
        # self.npc2_wp.append(npc2_waypoints[173])

        # print('1:',npc2_waypoints[3])
        # print('2:',npc2_waypoints[123])
        # print('3:',npc2_waypoints[173])

        npc2_wp1 = carla.Transform(carla.Location(x=-71.396385, y=45.022663, z=0.170000), carla.Rotation(yaw=51.998535))
        npc2_wp2 = carla.Transform(carla.Location(x=-86.172745, y=26.110786, z=0.170000), carla.Rotation(yaw=51.998535))
        npc2_wp3 = carla.Transform(carla.Location(x=-92.329567, y=18.230835, z=0.170000), carla.Rotation(yaw=51.998535))
        self.npc2_wp = [npc2_wp1,npc2_wp2,npc2_wp3]

        # for wp in self.npc2_wp:
        #     self.client.get_world().debug.draw_string(wp.transform.location, 'O', draw_shadow=False,
        #                         color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                         persistent_lines=True)

        #-----------------------------------Scenario 2--------------------------------------------------------#
        # npc3_waypoints = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 50):
        #         npc3_waypoints.append(waypoint)
        
        # npc3_wp1 = npc3_waypoints[454]
        # npc3_wp2 = npc3_waypoints[503]
        # npc3_wp3 = npc3_waypoints[603]
        # npc3_wp4 = npc3_waypoints[654]
        # npc3_wp5 = npc3_waypoints[804]
        # npc3_wp6 = npc3_waypoints[904]

        # print('1:',npc3_waypoints[454])
        # print('2:',npc3_waypoints[503])
        # print('3:',npc3_waypoints[603])
        # print('4:',npc3_waypoints[654])
        # print('5:',npc3_waypoints[804])
        # print('6:',npc3_waypoints[904])

        npc3_wp1 = carla.Transform(carla.Location(x=25.635315, y=-8.141112, z=0.513907), carla.Rotation(yaw=141.751526))
        npc3_wp2 = carla.Transform(carla.Location(x=35.655403, y=-11.583177, z=0.476566), carla.Rotation(yaw=141.751526))
        npc3_wp3 = carla.Transform(carla.Location(x=51.362076, y=-23.964638, z=0.379022), carla.Rotation(yaw=141.751526))
        npc3_wp4 = carla.Transform(carla.Location(x=57.048653, y=-32.904034, z=0.330068), carla.Rotation(yaw=141.751526))
        npc3_wp5 = carla.Transform(carla.Location(x=80.608643, y=-51.476231-2.5, z=0.187904), carla.Rotation(yaw=141.751526))
        npc3_wp6 = carla.Transform(carla.Location(x=96.315308, y=-63.857693-2.5, z=0.170000), carla.Rotation(yaw=141.751526))

        self.npc3_wp = [npc3_wp1, npc3_wp2, npc3_wp3, npc3_wp4, npc3_wp5, npc3_wp6]

        # for wp in self.npc3_wp:
        #     self.client.get_world().debug.draw_string(wp.transform.location, 'O', draw_shadow=False,
        #                         color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                         persistent_lines=True)
            
        #-----------------------------------Scenario 3--------------------------------------------------------#
        # npc4_waypoints1 = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 50):
        #         npc4_waypoints1.append(waypoint)
        
        # # # npc4_wp1 = npc4_waypoints[1004]
        # npc4_wp1 = npc4_waypoints1[1004]
        # npc4_wp2 = npc4_waypoints1[1104]
        # npc4_wp3 = npc4_waypoints1[1353]
        # # npc4_wp4 = npc4_waypoints[2003]
        # # npc4_wp5 = npc4_waypoints[2903]
        # # npc4_wp6 = npc4_waypoints[3003]
        # npc4_wp4 = npc4_waypoints1[1503]
        # npc4_wp5 = npc4_waypoints1[2163]
        # # npc4_wp6 = npc4_waypoints[2233]

        # npc4_waypoints2 = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 80):
        #         npc4_waypoints2.append(waypoint)
        # npc4_wp6 = npc4_waypoints2[180]
        # npc4_wp7 = npc4_waypoints2[48]
        # npc4_wp8 = npc4_waypoints2[8]

        # print('1:',npc4_wp1)
        # print('2:',npc4_wp2)
        # print('3:',npc4_wp3)
        # print('4:',npc4_wp4)
        # print('5:',npc4_wp5)
        # print('6:',npc4_wp6)
        # print('7:',npc4_wp7)
        # print('8:',npc4_wp8)

        npc4_wp1 = carla.Transform(carla.Location(x=112.021973, y=-76.239151, z=0.170000), carla.Rotation(yaw=141.751526))
        npc4_wp2 = carla.Transform(carla.Location(x=127.728653, y=-88.620605, z=0.170000), carla.Rotation(yaw=141.751526))
        npc4_wp3 = carla.Transform(carla.Location(x=169.162079, y=-116.825600, z=0.170000), carla.Rotation(yaw=141.751526))
        npc4_wp4 = carla.Transform(carla.Location(x=192.722076, y=-135.397781, z=0.170000), carla.Rotation(yaw=141.751526))
        npc4_wp5 = carla.Transform(carla.Location(x=296.386078, y=-217.115433, z=0.170000), carla.Rotation(yaw=141.751526))
        npc4_wp6 = carla.Transform(carla.Location(x=298.195496, y=-236.457703, z=0.170000), carla.Rotation(yaw=49.791031))
        npc4_wp7 = carla.Transform(carla.Location(x=255.587387, y=-286.861572, z=0.170000), carla.Rotation(yaw=49.791031))
        npc4_wp8 = carla.Transform(carla.Location(x=242.675827, y=-302.135468, z=0.170000), carla.Rotation(yaw=49.791031))
        

        # self.npc4_wp = [npc4_wp1, npc4_wp2, npc4_wp3, npc4_wp4]
        self.npc4_wp = [npc4_wp1, npc4_wp2, npc4_wp3, npc4_wp4, npc4_wp5, npc4_wp6, npc4_wp7, npc4_wp8]

        # for wp in self.npc4_wp:
        #     self.client.get_world().debug.draw_string(wp.location, '4', draw_shadow=False,
        #                         color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                         persistent_lines=True)
            
        # npc5_waypoints1 = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 50):
        #         npc5_waypoints1.append(waypoint)
        # npc5_wp1 = self.client.get_world().get_map().get_waypoint(carla.Location(x=191.113754, y=-137.460129, z=0), project_to_road=True, lane_type=(carla.LaneType.Shoulder))
        # npc5_wp2 = self.client.get_world().get_map().get_waypoint(carla.Location(x=238.339310, y=-174.470215, z=0), project_to_road=True, lane_type=(carla.LaneType.Shoulder))
        # npc5_wp3 = self.client.get_world().get_map().get_waypoint(carla.Location(x=301.306702, y=-223.817032, z=0), project_to_road=True, lane_type=(carla.LaneType.Shoulder))
       
        # npc5_waypoints2 = []
        # for waypoint in self.waypoints:
        #     if(waypoint.road_id == 80):
        #         npc5_waypoints2.append(waypoint)
        # npc5_wp5 = self.client.get_world().get_map().get_waypoint(carla.Location(x=297.4209236, y=-235.220518021, z=0), project_to_road=True, lane_type=(carla.LaneType.Shoulder))
        # npc5_wp4 = self.client.get_world().get_map().get_waypoint(carla.Location(x=269.2502923, y=-266.890518021, z=0), project_to_road=True, lane_type=(carla.LaneType.Shoulder))

        # print('1:',npc5_wp1)
        # print('2:',npc5_wp2)
        # print('3:',npc5_wp3)
        # print('4:',npc5_wp4)
        # print('5:',npc5_wp5)

        npc5_wp1 = carla.Transform(carla.Location(x=189.389832, y=-139.647064, z=0.170000), carla.Rotation(yaw=141.751526))
        npc5_wp2 = carla.Transform(carla.Location(x=236.509598, y=-176.791275, z=0.170000), carla.Rotation(yaw=141.751526))
        npc5_wp3 = carla.Transform(carla.Location(x=299.008942, y=-226.650360, z=0.170000), carla.Rotation(yaw=140.047653))
        npc5_wp4 = carla.Transform(carla.Location(x=269.167114, y=-266.820160, z=0.170000), carla.Rotation(yaw=49.791031))
        npc5_wp5 = carla.Transform(carla.Location(x=296.955231, y=-234.826859, z=0.170000), carla.Rotation(yaw=49.791031))

        # self.npc5_wp = [npc5_wp1, npc5_wp2, npc5_wp3]
        self.npc5_wp = [npc5_wp1, npc5_wp2, npc5_wp3, npc5_wp4, npc5_wp5]

        # for wp in self.npc5_wp:
        #     self.client.get_world().debug.draw_string(wp.location, '5', draw_shadow=False,
        #                         color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                         persistent_lines=True)
            
        #-----------------------------------Scenario 4--------------------------------------------------------#

    def drawWaypoints(self, road_id, life_time):
        
        for waypoint in self.waypoints:
            if(waypoint.road_id == road_id):
                self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                    persistent_lines=True)

    def executeScenario(self, ego_transform, start_time):
        
        (obs_npc1, obs_npc2, obs_ped1, obs_npc3, obs_ped2, obs_cones, obs_npc4, obs_npc5, obs_npc6, obs_ped3, obs_ped4, obs_npcA, obs_npcB, obs_npcC, obs_npcD) = self.actorList
        
        elapsed_time = time.perf_counter() - start_time
        
        #-----------------------------------Oncoming Vehicles --------------------------------------------------------#
        npcA_controller = VehiclePIDControllerLoc(obs_npcA, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        npcB_controller = VehiclePIDControllerLoc(obs_npcB, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        npcC_controller = VehiclePIDControllerLoc(obs_npcC, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        npcD_controller = VehiclePIDControllerLoc(obs_npcD, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

        npcA_transform = obs_npcA.get_transform()
        npcB_transform = obs_npcB.get_transform()
        npcC_transform = obs_npcC.get_transform()
        npcD_transform = obs_npcD.get_transform()

        trigDist = 10
        #NPC_OC Distance Finder
        x_diff_npcD = ego_transform.location.x - npcD_transform.location.x
        y_diff_npcD = ego_transform.location.y - npcD_transform.location.y
        relDist_npcD = math.sqrt(x_diff_npcD**2 + y_diff_npcD**2)

        #NPC_A trigger
        if self.npcA_wpCount < len(self.npcA_wp)-1:
            # npcA_nextWP_dist = math.sqrt((npcA_transform.location.x-self.npcA_wp[self.npcA_wpCount].transform.location.x)**2 + (npcA_transform.location.y-self.npcA_wp[self.npcA_wpCount].transform.location.y)**2)
            npcA_nextWP_dist = math.sqrt((npcA_transform.location.x-self.npcA_wp[self.npcA_wpCount].location.x)**2 + (npcA_transform.location.y-self.npcA_wp[self.npcA_wpCount].location.y)**2)
            if npcA_nextWP_dist < 1:
                self.npcA_wpCount = self.npcA_wpCount+1

            if relDist_npcD<=trigDist:
                self.npcA_spd=8.3333 #30km/h
                
        if self.npcA_wpCount < len(self.npcA_wp)-1:        
            npcA_control_signal = npcA_controller.run_step(self.npcA_spd*3.6, self.npcA_wp[self.npcA_wpCount])

        else:
            self.npcA_spd = 0.0
            npcA_control_signal = npcA_controller.run_step(self.npcA_spd*3.6, self.npcA_wp[len(self.npcA_wp)-1])    
        
        #NPC_A Controller                        
        obs_npcA.apply_control(carla.VehicleControl(steer = npcA_control_signal.steer/1.5, throttle = npcA_control_signal.throttle, brake = npcA_control_signal.brake))

        #NPC_B trigger
        if self.npcB_wpCount < len(self.npcB_wp)-1:
            npcB_nextWP_dist = math.sqrt((npcB_transform.location.x-self.npcB_wp[self.npcB_wpCount].location.x)**2 + (npcB_transform.location.y-self.npcB_wp[self.npcB_wpCount].location.y)**2)
            # npcB_nextWP_dist = math.sqrt((npcB_transform.location.x-self.npcB_wp[self.npcB_wpCount].location.x)**2 + (npcB_transform.location.y-self.npcB_wp[self.npcB_wpCount].location.y)**2)

            if npcB_nextWP_dist < 1:
                self.npcB_wpCount = self.npcB_wpCount+1

            if relDist_npcD<=trigDist:
                self.npcB_spd=6.2556 #20km/h
                
        if self.npcB_wpCount < len(self.npcB_wp)-1:        
            npcB_control_signal = npcB_controller.run_step(self.npcB_spd*3.6, self.npcB_wp[self.npcB_wpCount])

        else:
            self.npcB_spd = 0.0
            npcB_control_signal = npcB_controller.run_step(self.npcB_spd*3.6, self.npcB_wp[len(self.npcB_wp)-1])    
        
        #NPC_B Controller                        
        obs_npcB.apply_control(carla.VehicleControl(steer = npcB_control_signal.steer/1.5, throttle = npcB_control_signal.throttle, brake = npcB_control_signal.brake))

        #NPC_C trigger
        if self.npcC_wpCount < len(self.npcC_wp)-1:
            npcC_nextWP_dist = math.sqrt((npcC_transform.location.x-self.npcC_wp[self.npcC_wpCount].location.x)**2 + (npcC_transform.location.y-self.npcC_wp[self.npcC_wpCount].location.y)**2)

            if npcC_nextWP_dist < 1:
                self.npcC_wpCount = self.npcC_wpCount+1

            if relDist_npcD<=trigDist:
                self.npcC_spd=6.94444 #25km/h
                
        if self.npcC_wpCount < len(self.npcC_wp)-1:        
            npcC_control_signal = npcC_controller.run_step(self.npcC_spd*3.6, self.npcC_wp[self.npcC_wpCount])

        else:
            self.npcC_spd = 0.0
            npcC_control_signal = npcC_controller.run_step(self.npcC_spd*3.6, self.npcC_wp[len(self.npcC_wp)-1])    
        
        #NPC_B Controller                        
        obs_npcC.apply_control(carla.VehicleControl(steer = npcC_control_signal.steer/1.5, throttle = npcC_control_signal.throttle, brake = npcC_control_signal.brake))

       

        #-----------------------------------Scenario 1--------------------------------------------------------#
        npc2_controller = VehiclePIDControllerLoc(obs_npc2, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        npc2_transform = obs_npc2.get_transform()
        ped1_transform = obs_ped1.get_transform()

        #NPC2 Distance Finder
        x_diff_npc2 = ego_transform.location.x - npc2_transform.location.x
        y_diff_npc2 = ego_transform.location.y - npc2_transform.location.y
        relDist_npc2 = math.sqrt(x_diff_npc2**2 + y_diff_npc2**2)

        # npc2_targetDist = math.sqrt((npc2_transform.location.x-npc2_wp.transform.location.x)**2 + (npc2_transform.location.y-npc2_wp.transform.location.y)**2)
        # print(npc2_travelDist)

        #PED1 Distance Finder
        x_diff_ped1 = ego_transform.location.x - ped1_transform.location.x
        y_diff_ped1 = ego_transform.location.y - ped1_transform.location.y
        relDist_ped1 = math.sqrt(x_diff_ped1**2 + y_diff_ped1**2)

        ped1_travelDist = math.sqrt((ped1_transform.location.x-self.ped1_init.location.x)**2 + (ped1_transform.location.y-self.ped1_init.location.y)**2)

        #Ped1 new trigger
        if relDist_ped1<=100:
            
            # ped_timer = 0.0
            # print(time.time()-ped_timer_start)
            if time.time()-self.ped_timer_start >1 :
                self.ped1_wpCount = self.ped1_wpCount+1                            
                # print(self.ped1_wpCount)
                self.ped_timer_start = time.time()

            if self.ped1_wpCount < 35:
                # print('SC1: Ped moving')
                
                self.ped1_spd = self.ped1_df['Speedx2'][self.ped1_wpCount]
                pedDataAngle = math.degrees(self.ped1_df['Heading'][self.ped1_wpCount])
                pedAngle = math.radians(pedDataAngle) #rads
                self.xTarg = self.xNorth*math.cos(pedAngle) - self.yNorth*math.sin(pedAngle)
                self.yTarg = self.xNorth*math.sin(pedAngle) + self.yNorth*math.cos(pedAngle)
                # print(pedDataAngle)
                # ped1_spd = WP2pedspd
                # pedDataAngle = WP2pedheading # << Negative , >> Positive   
            else:
                self.ped1_spd = 0.0
                pedDataAngle = 0 # << Negative , >> Positive  
            
        else:
            self.ped1_spd = 0.0
            pedDataAngle = 0 # << Negative , >> Positive

        #Ped1 Controller
        obs_ped1.apply_control(carla.WalkerControl( 
                speed=self.ped1_spd,
                direction=carla.Vector3D(self.xTarg, self.yTarg, 0.0),
            ))

        #NPC2 trigger
        if self.npc2_wpCount < len(self.npc2_wp)-1:
            npc2_nextWP_dist = math.sqrt((npc2_transform.location.x-self.npc2_wp[self.npc2_wpCount].location.x)**2 + (npc2_transform.location.y-self.npc2_wp[self.npc2_wpCount].location.y)**2)

            if npc2_nextWP_dist < 1:
                self.npc2_wpCount = self.npc2_wpCount+1

            if relDist_npc2<=40:
                self.npc2_spd=8.0
                # print('SC1: NPC moving')
        if self.npc2_wpCount < len(self.npc2_wp)-1:        
            npc2_control_signal = npc2_controller.run_step(self.npc2_spd*3.6, self.npc2_wp[self.npc2_wpCount])
        else:
            self.npc2_spd = 0.0
            npc2_control_signal = npc2_controller.run_step(self.npc2_spd*3.6, self.npc2_wp[len(self.npc2_wp)-1])    
        
        #NPC2 Controller                        
        obs_npc2.apply_control(carla.VehicleControl(steer = npc2_control_signal.steer/1.5, throttle = npc2_control_signal.throttle, brake = npc2_control_signal.brake))

        #Traffic Light Controller
        # sc1_trafficlight= self.world.get_traffic_light(carla.Location(x=-71.19, y=65.06, z=2))
        # # sc1_trafficlight= self.world.get_traffic_light()
        # sc1_trafficlight(carla.TrafficLight.set_state.Green)


        #-----------------------------------Scenario 2--------------------------------------------------------#
        npc3_controller = VehiclePIDControllerLoc(obs_npc3, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        npc3_transform = obs_npc3.get_transform()
        ped2_transform = obs_ped2.get_transform()

        #NPC3 Distance Finder
        x_diff_npc3 = ego_transform.location.x - npc3_transform.location.x
        y_diff_npc3 = ego_transform.location.y - npc3_transform.location.y
        relDist_npc3 = math.sqrt(x_diff_npc3**2 + y_diff_npc3**2)

        #PED2 Distance Finder
        x_diff_ped2 = ego_transform.location.x - ped2_transform.location.x
        y_diff_ped2 = ego_transform.location.y - ped2_transform.location.y
        relDist_ped2 = math.sqrt(x_diff_ped2**2 + y_diff_ped2**2)

        ped2_travelDist = math.sqrt((ped2_transform.location.x-self.ped2_init.location.x)**2 + (ped2_transform.location.y-self.ped2_init.location.y)**2)

        #NPC3 trigger
        if self.npc3_wpCount < len(self.npc3_wp)-1:
            npc3_nextWP_dist = math.sqrt((npc3_transform.location.x-self.npc3_wp[self.npc3_wpCount].location.x)**2 + (npc3_transform.location.y-self.npc3_wp[self.npc3_wpCount].location.y)**2)

            if npc3_nextWP_dist < 1:
                self.npc3_wpCount = self.npc3_wpCount+1

            if relDist_npc3<=40:
                # self.npc3_spd=6.9 #25km/h
                self.npc3_spd=5.9 #25km/h
                # print('SC2: NPC moving')
        if self.npc3_wpCount < len(self.npc3_wp)-1:        
            npc3_control_signal = npc3_controller.run_step(self.npc3_spd*3.6, self.npc3_wp[self.npc3_wpCount])
        else:
            self.npc3_spd = 0
            npc3_control_signal = npc3_controller.run_step(self.npc3_spd*3.6, self.npc3_wp[len(self.npc3_wp)-1])    
                                
        obs_npc3.apply_control(carla.VehicleControl(steer = npc3_control_signal.steer, throttle = npc3_control_signal.throttle, brake = npc3_control_signal.brake))

        #Ped2 trigger
        if relDist_ped2<=50 and ped2_travelDist <=20:
            # print('SC2: Ped moving')
            self.ped2_spd=1.0
            
        else:
            self.ped2_spd=0.0

        #Ped2 Controller
        obs_ped2.apply_control(carla.WalkerControl(
                speed=self.ped2_spd,
                direction=carla.Vector3D(1.0, 1.0, 0.0),
            ))

        #-----------------------------------Scenario 3--------------------------------------------------------#
        npc4_controller = VehiclePIDControllerLoc(obs_npc4, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})        
        npc5_controller = VehiclePIDControllerLoc(obs_npc5, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
        npc4_transform = obs_npc4.get_transform()
        npc5_transform = obs_npc5.get_transform()

        #NPC4 Distance Finder
        x_diff_npc4 = ego_transform.location.x - npc4_transform.location.x
        y_diff_npc4 = ego_transform.location.y - npc4_transform.location.y
        relDist_npc4 = math.sqrt(x_diff_npc4**2 + y_diff_npc4**2)

        #NPC5 Distance Finder
        x_diff_npc5 = ego_transform.location.x - npc5_transform.location.x
        y_diff_npc5 = ego_transform.location.y - npc5_transform.location.y
        relDist_npc5 = math.sqrt(x_diff_npc5**2 + y_diff_npc5**2)

        #NPC4 trigger
        if self.npc4_wpCount < len(self.npc4_wp)-1:
            npc4_nextWP_dist = math.sqrt((npc4_transform.location.x-self.npc4_wp[self.npc4_wpCount].location.x)**2 + (npc4_transform.location.y-self.npc4_wp[self.npc4_wpCount].location.y)**2)

            if npc4_nextWP_dist < 1:
                self.npc4_wpCount = self.npc4_wpCount+1

            if relDist_npc4<=13:
                self.npc4_spd=13.9 #50km/h
                # print('SC3: NPC ambulance moving')
        if self.npc4_wpCount < len(self.npc4_wp)-1:        
            npc4_control_signal = npc4_controller.run_step(self.npc4_spd*3.6, self.npc4_wp[self.npc4_wpCount])
        else:
            self.npc4_spd = 0
            npc4_control_signal = npc4_controller.run_step(self.npc4_spd*3.6, self.npc4_wp[len(self.npc4_wp)-1])    
                                
        obs_npc4.apply_control(carla.VehicleControl(steer = npc4_control_signal.steer, throttle = npc4_control_signal.throttle, brake = npc4_control_signal.brake))

        #NPC5 trigger
        if self.npc5_wpCount < len(self.npc5_wp)-1:
            npc5_nextWP_dist = math.sqrt((npc5_transform.location.x-self.npc5_wp[self.npc5_wpCount].location.x)**2 + (npc5_transform.location.y-self.npc5_wp[self.npc5_wpCount].location.y)**2)

            if npc5_nextWP_dist < 1:
                self.npc5_wpCount = self.npc5_wpCount+1
                # self.npc5_spd=10 #20km/h

            if relDist_npc5<=30:
                self.npc5_spd=5.5 #20km/h
                # print('SC3: NPC bike moving')
        if self.npc5_wpCount < len(self.npc5_wp)-1:        
            npc5_control_signal = npc5_controller.run_step(self.npc5_spd*3.6, self.npc5_wp[self.npc5_wpCount])
        else:
            self.npc5_spd = 0
            npc5_control_signal = npc5_controller.run_step(self.npc5_spd*3.6, self.npc5_wp[len(self.npc5_wp)-1])    
                                
        obs_npc5.apply_control(carla.VehicleControl(steer = npc5_control_signal.steer/1.5, throttle = npc5_control_signal.throttle, brake = npc5_control_signal.brake))


        #-----------------------------------Scenario 4--------------------------------------------------------#
        ped3_transform = obs_ped3.get_transform()
        ped4_transform = obs_ped4.get_transform()

        #PED3 Distance Finder
        x_diff_ped3 = ego_transform.location.x - ped3_transform.location.x
        y_diff_ped3 = ego_transform.location.y - ped3_transform.location.y
        relDist_ped3 = math.sqrt(x_diff_ped3**2 + y_diff_ped3**2)

        ped3_travelDist = math.sqrt((ped3_transform.location.x-self.ped3_init.location.x)**2 + (ped3_transform.location.y-self.ped3_init.location.y)**2)
        
        #PED4 Distance Finder
        x_diff_ped4 = ego_transform.location.x - ped4_transform.location.x
        y_diff_ped4 = ego_transform.location.y - ped4_transform.location.y
        relDist_ped4 = math.sqrt(x_diff_ped4**2 + y_diff_ped4**2)

        ped4_travelDist = math.sqrt((ped4_transform.location.x-self.ped4_init.location.x)**2 + (ped4_transform.location.y-self.ped4_init.location.y)**2)

        #Ped3 and Ped4 trigger
        if relDist_ped3<=30 and ped3_travelDist <=18:
            # print('SC2: Ped moving')
            self.ped3_spd=1.38889
            self.ped4_spd=1.66667
            
        else:
            self.ped3_spd=0.0
            self.ped4_spd=0.0

        #Ped3 Controller
        obs_ped3.apply_control(carla.WalkerControl(
                speed=self.ped3_spd,
                direction=carla.Vector3D(1.0, 1.0, 0.0),
            ))

        #Ped4 Controller
        obs_ped4.apply_control(carla.WalkerControl(
                speed=self.ped4_spd,
                direction=carla.Vector3D(1.0, 1.0, 0.0),
            ))