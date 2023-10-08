#!/usr/bin/env python3

import asyncio

from mavsdk.camera import (CameraError, Mode)
from mavsdk import System

# Define an asynchronous function 'run' to control the drone
async def run():

    # Create a System object to interact with the drone
    drone = System()

    # Connect to the drone using a specified system address (UDP)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    # Wait for the drone to establish a connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Create asynchronous tasks to print camera mode and status
    print_mode_task = asyncio.ensure_future(print_mode(drone))
    print_status_task = asyncio.ensure_future(print_status(drone))
    running_tasks = [print_mode_task, print_status_task]

    print("Setting mode to 'PHOTO'")
    try:
         # Set the camera mode to 'PHOTO'
        await drone.camera.set_mode(Mode.PHOTO)
    except CameraError as error:
        print(f"Setting mode failed with error code: {error._result.result}")

    await asyncio.sleep(2)

    print("Taking a photo")
    try:
        # Trigger the camera to take a photo
        await drone.camera.take_photo()
    except CameraError as error:
        print(f"Couldn't take photo: {error._result.result}")

    # Shut down the running coroutines (here 'print_mode()' and
    # 'print_status()')
    for task in running_tasks:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    await asyncio.get_event_loop().shutdown_asyncgens()

# Define an asynchronous function 'print_mode' to print the camera mode
async def print_mode(drone):
    async for mode in drone.camera.mode():
        print(f"Camera mode: {mode}")

# Define an asynchronous function 'print_status' to print the camera status
async def print_status(drone):
    async for status in drone.camera.status():
        print(status)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
