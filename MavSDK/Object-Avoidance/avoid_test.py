#!/usr/bin/env python3

import argparse         # Lets us specify if we want to simulate or move drone
import subprocess       # Run concurrent terminals for Shell to PX4 and mavsdk_server
import asyncio          # Asyncio allows for near concurrent processing 
import time
import drone_util       # Utilities to use on drone
import flight_tests     # Flight paths and mission planning
import tune_example     # Plays tune to show connection
import read_pi_pico     # Lidar connection using rpi pico GPIO pinout

from mavsdk import System   # Mavsdk system talks to the drone using serial port USB0/1 or ACM0/1
     
# Basic setup before flight can happen, connect drone, check telemetry & gps, play tune etc
async def init_drone(sim):
    await util.start_connection(drone, sim)
    await util.check_status(drone)
    await util.setup_info(drone)
    await tunes.valk(drone)    # Plays the flight of the valkryies tune
    # await util.define_geofence(drone)
    # await util.calibrate_drone(drone)


# ALL CODE TESTING IS DONE IN MAIN #
    # To make new functions, duplicate a function in drone_util.py, then copy and modify the code using this website ->
    # Mavsdk-python API library using 'System()' as drone.function.example()   |   http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/index.html#
async def main(sim, LIDAR):
    ''' Initializes the drone, print coordinates & heading, then start a flight path '''

    lidar_read, lidar_write = await LIDAR.run(port='/dev/ttyACM1')     # Instantiate serial connection to pico for lidar data

    # while True:       # Continues to print lidar read distance
    #     await asyncio.sleep(0.01);      d = await lidar_read.readline()
    #     line = str(d[:-2], 'utf8');     print(line)

    await init_drone(sim)
    print("\n\t:: Drone Initialized Ready for Flight ::\n")
    await util.abs_altitude(drone)
    print("GPS Coordinates \n\tAltitude: "+str(util.absolute_altitude)+"\n\tLatitude: "
          +str(util.latitude)+"\n\tLongitude: "+str(util.longitude)+"\n")
    await util.get_heading(drone)

    # await util.connect_shell(drone)       # - Used to talk directly with pixhawk 4 autopilot 
    # await asyncio.sleep(5000)             # - Time delay used to keep talking with px4
    # await util.print_all_params(drone)
    # await util.use_camera(drone)

    # await flights.takeoff(drone, util)    # - Default will fly 3 meters high, spin, and land
    await flights.test_LIDAR(drone, util, lidar_read) # - Testing LIDAR, flys 1 meter up, moves until detects obj
    print('past lidar testflight')
    # await flights.altitude_control(drone, util)
    # await flights.GPS_control(drone, util)
    # await flights.velocity_control(drone, util)
    # await flights.schedule_mission(drone, util)
    # await flights.qgroundcontrol_mission(drone, util)

    await asyncio.sleep(2)
    print("\n\n\t< Finished Program -- Exiting Loop >\n\n")
    util.status_text_task.cancel()
    loop = asyncio.get_running_loop()   # End Script
    loop.stop()


# Launches ./mavsdk_server on port 50051 across the serial port, typically ttyACM0 on ubuntu
def launch_mavsdk(connection):
    print("-- Shell terminal to PX4 | Start mavsdk_server by 'serial:///dev/ttyACM0:921600'\n")
    time.sleep(1)
    try:
        shell_CMD = ['gnome-terminal', '--', 'python3', 'mavlink_shell.py', '--baudrate', '921600']
        mavlink_CMD = ['gnome-terminal', '--', './rpi_server/mavsdk_server', '-p', '50051']
        # print(mavlink_CMD); print(shell_CMD);

        shell_CMD.append(connection)
        mavlink_CMD.append("serial://" + connection + ":921600")
        # print(mavlink_CMD); print(shell_CMD)

        PX4_shell = subprocess.run(args=shell_CMD, shell=False, text=True, universal_newlines=True)
        time.sleep(2)
        mavlink_srvr = subprocess.run(args=mavlink_CMD, shell=False, text=True, universal_newlines=True)

        if ( not(PX4_shell.stderr == None) or not(mavlink_srvr.stderr == None) ):
            print(str(PX4_shell.stdout) + "\n" + str(PX4_shell.stderr) + "\n\n\t:: MAVLINK ::\n" +
                str(mavlink_srvr.stdout) + "\n" + str(mavlink_srvr.stderr))

    except:
        print("\n\tServer can't launch, check the /dev/tty___ port being used as connection!\n")


if __name__=="__main__":
    # Take arguments passed to program to see if simulation is run ('-s 1' runs sim)
    parser = argparse.ArgumentParser(description='Drone object avoidance in simulation or over pixhawk4.')
    parser.add_argument('-s', choices=['1'], help='Set flag as 1 to run simulation, default is 0.')
    args = parser.parse_args()
    sim = (args.s)

    # Initilizedss drone to look for the mav_sdk server on 'localhost':port 50051 
    #   If "--sim" is added when running program initiates simulator 
    if ( (sim == None) or (sim == '0') ):
        drone = System(mavsdk_server_address='localhost', port=50051)
        launch_mavsdk('/dev/ttyACM0')
    else:
        drone = System()

    # Create the drone, the util file, tunes, & LIDAR as Objects for use in the scripts
    util = drone_util.util()
    flights = flight_tests.flights()
    tunes = tune_example.songs()
    LIDAR = read_pi_pico.OutputProtocol()
    
    # Run the asyncio loop, if you need to quit the program press 'ctrl + c'
    asyncio.ensure_future(main(sim,LIDAR))
    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        print("\n\tEnded from keyboard interrupt: press 'ctrl + c' again to close script.\n")
        asyncio.run(util.land_drone(drone, util))
        exit(1)
    finally:
        exit(0)