import carla
from bridgeClient import *
import math
from time import sleep
from agents.navigation.controller_wpLoc import VehiclePIDController as VehiclePIDControllerLoc
import time

'''
This is an example Scenario Manager module which shows how actors
can be spawned, waypoints drawn and actors moved based on these
waypoints.

The end-user may replace the code under spawnActors(), getWaypoints() and executeScenario()
to create their own scenarios based on the Carla Python API framework.

See https://carla.readthedocs.io/en/latest/python_api/#carlaactor and associated documentation.

'''
class ScenarioManager():
    def __init__(self, CARLA_HOST, CARLA_PORT):
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = self.client.get_world()
        self.carlamap = self.world.get_map()
        self.spectator = self.world.get_spectator()
        
        self.waypoints = self.client.get_world().get_map().generate_waypoints(distance=1.0)

        self.actorList = self.spawnActors()
        
    def spawnActors(self):

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
        self.getWaypoints()

        return (obs_npcA, obs_npcB, obs_npcC, obs_npcD)
    
    def getWaypoints(self):
        #-----------------------------------Oncoming Vehicles --------------------------------------------------------#

        npcA_wp1 = carla.Transform(carla.Location(x=73.43, y=-32.24, z=0.540000), carla.Rotation(yaw=-38.248478))
        npcA_wp2 = carla.Transform(carla.Location(x=9.54, y=10.52, z=0.540000), carla.Rotation(yaw=-38.248478))
        npcA_wp3 = carla.Transform(carla.Location(x=-36.973751, y=54.583321, z=0.540000), carla.Rotation(yaw=-38.248478))
        npcA_wp4 = carla.Transform(carla.Location(x=-38.973751, y=52.583321, z=0.540000), carla.Rotation(yaw=-38.248478))
        
        self.npcA_wp = [npcA_wp1,npcA_wp2,npcA_wp3,npcA_wp4]
        self.npcA_wpCount = len(self.npcA_wp)

        npcB_wp1 = carla.Transform(carla.Location(x=92.795494, y=-52.169624, z=0.170165), carla.Rotation(yaw=141.751526))
        npcB_wp2 = carla.Transform(carla.Location(x=65.119583, y=-25.896183, z=0.320277), carla.Rotation(yaw=141.751526))
        npcB_wp3 = carla.Transform(carla.Location(x=50.387497, y=-18.739677, z=0.398604), carla.Rotation(yaw=141.751526))
        npcB_wp4 = carla.Transform(carla.Location(x=37.822159, y=-8.834510, z=0.476566), carla.Rotation(yaw=141.751526))
        npcB_wp5 = carla.Transform(carla.Location(x=-39.140507, y=51.834652, z=0.540000), carla.Rotation(yaw=141.751526))

        self.npcB_wp = [npcB_wp1,npcB_wp2,npcB_wp3,npcB_wp4,npcB_wp5]
        self.npcB_wpCount = len(self.npcB_wp)

        npcC_wp1 = carla.Transform(carla.Location(x=149.935593, y=-92.756073, z=0.170000), carla.Rotation(yaw=141.751526))
        npcC_wp2 = carla.Transform(carla.Location(x=116.355492, y=-70.741814, z=0.170000), carla.Rotation(yaw=141.751526))       
        npcC_wp3 = carla.Transform(carla.Location(x=102.815582, y=-55.611691, z=0.170000), carla.Rotation(yaw=141.751526))      
        npcC_wp4 = carla.Transform(carla.Location(x=-32.857841, y=46.882069, z=0.540000), carla.Rotation(yaw=141.751526))
        npcC_wp5 = carla.Transform(carla.Location(x=-35.999172, y=49.358360, z=0.540000), carla.Rotation(yaw=141.751526))
  
        self.npcC_wp = [npcC_wp1,npcC_wp2,npcC_wp3,npcC_wp4,npcC_wp5]
        self.npcC_wpCount = len(self.npcC_wp)

        # for wp in self.npcA_wp:
        #     self.client.get_world().debug.draw_string(wp.location, 'O', draw_shadow=False,
        #                         color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                         persistent_lines=True)
        

    def drawWaypoints(self, road_id, life_time):
        
        for waypoint in self.waypoints:
            if(waypoint.road_id == road_id):
                self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                    persistent_lines=True)

    def executeScenario(self, ego_transform, start_time):
        
        (obs_npcA, obs_npcB, obs_npcC, obs_npcD) = self.actorList
        
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
        # NPC_OC Distance Finder
        x_diff_npcD = ego_transform.location.x - npcD_transform.location.x
        y_diff_npcD = ego_transform.location.y - npcD_transform.location.y
        relDist_npcD = math.sqrt(x_diff_npcD**2 + y_diff_npcD**2)

        # NPC_A trigger
        if self.npcA_wpCount < len(self.npcA_wp)-1:
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
        
        # NPC_A Controller                        
        obs_npcA.apply_control(carla.VehicleControl(steer = npcA_control_signal.steer/1.5, throttle = npcA_control_signal.throttle, brake = npcA_control_signal.brake))

        # NPC_B trigger
        if self.npcB_wpCount < len(self.npcB_wp)-1:
            npcB_nextWP_dist = math.sqrt((npcB_transform.location.x-self.npcB_wp[self.npcB_wpCount].location.x)**2 + (npcB_transform.location.y-self.npcB_wp[self.npcB_wpCount].location.y)**2)

            if npcB_nextWP_dist < 1:
                self.npcB_wpCount = self.npcB_wpCount+1

            if relDist_npcD<=trigDist:
                self.npcB_spd=6.2556 #20km/h
                
        if self.npcB_wpCount < len(self.npcB_wp)-1:        
            npcB_control_signal = npcB_controller.run_step(self.npcB_spd*3.6, self.npcB_wp[self.npcB_wpCount])

        else:
            self.npcB_spd = 0.0
            npcB_control_signal = npcB_controller.run_step(self.npcB_spd*3.6, self.npcB_wp[len(self.npcB_wp)-1])    
        
        # NPC_B Controller                        
        obs_npcB.apply_control(carla.VehicleControl(steer = npcB_control_signal.steer/1.5, throttle = npcB_control_signal.throttle, brake = npcB_control_signal.brake))

        # NPC_C trigger
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
        
        # NPC_B Controller                        
        obs_npcC.apply_control(carla.VehicleControl(steer = npcC_control_signal.steer/1.5, throttle = npcC_control_signal.throttle, brake = npcC_control_signal.brake))