#!/usr/bin/env python3

import asyncio

from mavsdk.mission import (MissionItem, MissionPlan) # Mission planning for control
from mavsdk.offboard import (Attitude, OffboardError) # Altitude Control
from mavsdk.offboard import (OffboardError, PositionNedYaw) # GPS Control
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed) # Velocity Control
from mavsdk.offboard import (OffboardError, VelocityNedYaw) # GPS & Velocity Control
    # Asyncio allows for near concurrent processing 
    # Mavsdk interacts with the drone through serial port USB0/1 or ACM0/1 
    # Mavsdk.offboard allows manual altitude/thrust/tilt/GPS

class flights():
    def __init__(self):
        pass

    # Arm the drone if status check passed, generate path
    # Lift up and verify surroundings clear
    async def takeoff(self, drone, util):

        print("-- Taking off")
        # await drone.action.set_takeoff_altitude(6)
        # await drone.action.takeoff()

        await asyncio.sleep(5)
        await util.arm_drone(drone)

        print("-- Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, util.heading))

        await util.start_offboard(drone)  

        print("-- Go 0m North, 0m East, -5m Down within local coordinate system, facing current heading")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.5, util.heading))
        await asyncio.sleep(5)

        print("-- Go 0m North, 0m East, -5m Down within local coordinate system, turn to face 0 degree")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.5, 0))
        await asyncio.sleep(5)

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)

        await asyncio.sleep(5)


    async def test_LIDAR(self, drone, util, lidar_read):

        print("-- Testing object avoidance using LIDAR to see obsticle")

        await asyncio.sleep(5)
        await util.arm_drone(drone)

        print("-- Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, util.heading))

        await util.start_offboard(drone)  

        print("-- Go 0m North, 0m East, -2m Down within local coordinate system, facing current heading")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, util.heading))
        await asyncio.sleep(5)

        i = 0.0;    d = await lidar_read.readline()
        while i < 5:        # This is loop throws away the fist 5 responses from pi pico (settings & info)
            _ = await lidar_read.readline()
            i += 1

        i = 0.0;    d = await lidar_read.readline();    line = str(d[:-2], 'utf8')
        print("< Go 0.01m  North every 0.01 seconds until LIDAR detects object >")
        while ( (int(line) > 900) and ((i < 4)) ):   # Move until gone 4m or detect object 3ft away
            i += 0.015                         # Move 0.015m/0.01s = 1.5m/s
            d = await lidar_read.readline()
            line = str(d[:-2], 'utf8')
            await drone.offboard.set_position_ned(PositionNedYaw(i, 0.0, -1.0, util.heading))
            print("-- North by {0}m : \tLidar {1}cm".format('%s' % float('%.4g' % i),line))
            await asyncio.sleep(0.01)      

        # Now that loop has been escaped, move east 1.5m, north 1.5m, and move back 1.5m west (Should avoid obj in square movement)
        print('-- LIDAR detected object and stopped moving the drone')
        await asyncio.sleep(4)

        print("-- Go 'i'm North, 1.5m East, -1m Down within local coordinate system, facing current heading")
        await drone.offboard.set_position_ned(PositionNedYaw(i, 1.5, -1.0, util.heading))
        await asyncio.sleep(4)

        i += 1.5
        print("-- Go {0}m North, 1.5m East, -1m Down within local coordinate system, facing current heading".format('%s' % float('%.4g' % i)))
        await drone.offboard.set_position_ned(PositionNedYaw(i, 1.5, -1.0, util.heading))
        await asyncio.sleep(4)

        print("-- Go {0}m North, 0m East, -1m Down within local coordinate system, facing current heading".format('%s' % float('%.4g' % i)))
        await drone.offboard.set_position_ned(PositionNedYaw(i, 0, -1.0, util.heading))
        await asyncio.sleep(4)

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)
        await asyncio.sleep(5)


    # Arm the drone if status check passed, generate path
    # Lift up and verify surroundings clear
    async def demo_flight(self, drone, util, lidar_read):

        print("-- Taking off")
        # await drone.action.set_takeoff_altitude(6)
        # await drone.action.takeoff()

        await asyncio.sleep(5)
        await util.arm_drone(drone)

        print("-- Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, util.heading))

        await util.start_offboard(drone)  

        print("-- Go 0m North, 0m East, -1m Down within local coordinate system, facing current heading")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, util.heading))
        await asyncio.sleep(5)

        i = 0.0;    d = await lidar_read.readline()
        while i < 5:        # This is loop throws away the fist 5 responses from pi pico (settings & info)
            _ = await lidar_read.readline()
            i += 1

        i = 0.0;    d = await lidar_read.readline();    line = str(d[:-2], 'utf8')
        print("< Go 0.015m  North every 0.01 seconds until LIDAR detects object >")
        while ( i < 4 ):   # Move until gone 4m or detect object 3ft away
            while (int(line) > 900):
                i += 0.015                         # Move 0.015m/0.01s = 1.5m/s
                d = await lidar_read.readline()
                line = str(d[:-2], 'utf8')
                await drone.offboard.set_position_ned(PositionNedYaw(i, 0.0, -1.0, util.heading))
                print("-- North by {0}m : \tLidar {1}cm".format('%s' % float('%.4g' % i),line))
                await asyncio.sleep(0.01)  
            break       # Remove break to continue reading lidar and moving drone
        
            print('-- LIDAR detected object and stopped moving the drone')
            await asyncio.sleep(4)
            while (int(line) > 900):    # Begin further object avoidance here
                pass
        
        # Now that loop has been escaped, move east 1.5m, north 1.5m, and move back 1.5m west (Should avoid obj in square movement)
        print('-- LIDAR detected object and stopped moving the drone')
        await asyncio.sleep(4)

        print("-- Go 0m North, 0m East, -1m Down within local coordinate system, facing current heading")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, util.heading))
        await asyncio.sleep(5)          # Return drone back to original position

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)
        await asyncio.sleep(5)


    async def altitude_control(self, drone, util):

        print("-- Setting initial setpoint")
        await drone.action.set_takeoff_altitude(4)
        await drone.action.takeoff()
        await asyncio.sleep(8)

        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
        await util.start_offboard(drone)

        # await drone.offboard.PositionGlobalYaw(lat_deg, lon_deg, alt_m, yaw_deg, altitude_type)

        # print("-- Go up at 70% thrust")
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.7))
        await asyncio.sleep(4)

        print("-- Roll 30 at 60% thrust")
        await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
        await asyncio.sleep(2)

        print("-- Roll -30 at 60% thrust")
        await drone.offboard.set_attitude(Attitude(-30.0, 0.0, 0.0, 0.6))
        await asyncio.sleep(2)

        print("-- Hover at 60% thrust")
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
        await asyncio.sleep(2)

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)
        
        await asyncio.sleep(10)


    async def GPS_control(self, drone, util):

        print("-- Setting initial setpoint")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        await util.start_offboard(drone)

        print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await asyncio.sleep(10)

        print("-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East")
        await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -5.0, 90.0))
        await asyncio.sleep(10)

        print("-- Go 5m North, 10m East, -5m Down within local coordinate system")
        await drone.offboard.set_position_ned(PositionNedYaw(5.0, 10.0, -5.0, 90.0))
        await asyncio.sleep(15)

        print("-- Go 0m North, 10m East, 0m Down within local coordinate system, turn to face South")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, 0.0, 180.0))
        await asyncio.sleep(10)

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)

        await asyncio.sleep(10)


    # Moves the drone in a circle -> then circle sideways
    # Can move the drone at different speeds
    async def velocity_control(self, drone, util):

        print("-- Setting initial setpoint")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        await util.start_offboard(drone)

        print("-- Turn clock-wise and climb")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -1.0, 60.0))
        await asyncio.sleep(5)

        print("-- Turn back anti-clockwise")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, -60.0))
        await asyncio.sleep(5)

        print("-- Wait for a bit")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(2)

        print("-- Fly a circle")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(5.0, 0.0, 0.0, 30.0))
        await asyncio.sleep(15)

        print("-- Wait for a bit")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(5)

        print("-- Fly a circle sideways")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, -5.0, 0.0, 30.0))
        await asyncio.sleep(15)

        print("-- Wait for a bit")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(8)

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)

        await asyncio.sleep(10)


    async def GPS_velocity_control(self, drone, util):

        await util.start_offboard(drone)

        print("-- Setting initial setpoint")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Go up 2 m/s")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(4)

        print("-- Go North 2 m/s, turn to face East")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(2.0, 0.0, 0.0, 90.0))
        await asyncio.sleep(4)

        print("-- Go South 2 m/s, turn to face West")
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(-2.0, 0.0, 0.0, 270.0))
        await asyncio.sleep(4)

        print("-- Go West 2 m/s, turn to face East")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -2.0, 0.0, 90.0))
        await asyncio.sleep(4)

        print("-- Go East 2 m/s")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 2.0, 0.0, 90.0))
        await asyncio.sleep(4)

        print("-- Turn to face South")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 180.0))
        await asyncio.sleep(2)

        print("-- Go down 1 m/s, turn to face North")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 1.0, 0.0))
        await asyncio.sleep(4)

        await util.stop_offboard(drone)
        await util.land_drone(drone, util)

        await asyncio.sleep(10)

    # Will be used to schedule how the drone can move using coordinates and camera actions as well
    async def schedule_mission(self, drone, util):
        print("-- Testing mission planning for flight path of drone (To Test Obj. Avoidance)")

        mission_items = [];         latd = util.latitude;        long = util.longitude

        mission_items.append(MissionItem(latitude_deg           = latd,
                                        longitude_deg           = long,
                                        relative_altitude_m     = 3,
                                        speed_m_s               = 1,
                                        is_fly_through          = True,
                                        gimbal_pitch_deg        = float('nan'),
                                        gimbal_yaw_deg          = float('nan'),
                                        camera_action           = MissionItem.CameraAction.NONE,
                                        loiter_time_s           = float('nan'),
                                        camera_photo_interval_s = float('nan'),
                                        acceptance_radius_m     = float('nan'),
                                        yaw_deg                 = float('nan'),
                                        camera_photo_distance_m = float('nan')))
        
        mission_items.append(MissionItem(latitude_deg           = latd,
                                        longitude_deg           = long + 0.00001,
                                        relative_altitude_m     = 3,
                                        speed_m_s               = 1,
                                        is_fly_through          = True,
                                        gimbal_pitch_deg        = float('nan'),
                                        gimbal_yaw_deg          = float('nan'),
                                        camera_action           = MissionItem.CameraAction.NONE,
                                        loiter_time_s           = float('nan'),
                                        camera_photo_interval_s = float('nan'),
                                        acceptance_radius_m     = float('nan'),
                                        yaw_deg                 = float('nan'),
                                        camera_photo_distance_m = float('nan')))

        mission_items.append(MissionItem(latitude_deg           = latd,
                                        longitude_deg           = long + 0.00001,
                                        relative_altitude_m     = 2.5,    # Lower hover to 1 meter
                                        speed_m_s               = 1,
                                        is_fly_through          = True,
                                        gimbal_pitch_deg        = float('nan'),
                                        gimbal_yaw_deg          = float('nan'),
                                        camera_action           = MissionItem.CameraAction.NONE,
                                        loiter_time_s           = float('nan'),
                                        camera_photo_interval_s = float('nan'),
                                        acceptance_radius_m     = float('nan'),
                                        yaw_deg                 = float('nan'),
                                        camera_photo_distance_m = float('nan')))

        mission_plan = MissionPlan(mission_items)
        await drone.mission.set_return_to_launch_after_mission(True)

        print("-- Uploading mission")
        p = drone.mission.upload_mission_with_progress(mission_plan)
        print(p)

        await asyncio.sleep(5)
        await util.arm_drone(drone)

        print("-- Starting mission")
        await drone.mission.start_mission()

        while await drone.mission.is_mission_finished() == False:
            await asyncio.sleep(1)

        await util.land_drone(drone, util)
        await asyncio.sleep(10)


    async def qgroundcontrol_mission(self, drone, util):
        mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission("example-mission.plan")
        print(f"{len(mission_import_data.mission_items)} mission items imported")
        await drone.mission_raw.upload_mission(mission_import_data.mission_items)
        print("Mission uploaded")