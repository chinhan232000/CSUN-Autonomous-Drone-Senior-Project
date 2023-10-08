# Import necessary libraries
from rplidar import RPLidar     # Library for RPLidar communication
import multiprocessing as mp    # Library for managing multiple processes
import numpy as np              # Library for numerical operations
import time                     # Library for timing and delays


# Define the lidar_node function for Lidar data collection
def lidar_node(return_quality, return_angle, return_distance, return_request):
    print("Starting lidar_node...")

    # Initialize RPLidar, reset it, and get health and info information
    lidar = RPLidar('/dev/ttyUSB0')
    lidar.reset()
    health = lidar.get_health()
    info = lidar.get_info()
    print(f"Health: {health}")
    print(f"Info: {info}")

    # Continuously iterate through Lidar scans
    for scan in lidar.iter_scans():
        unpacked_data = list(zip(*scan))
        quality = unpacked_data[0]
        angle = unpacked_data[1]
        distance = unpacked_data[2]

        # If a request for data is received, copy the Lidar data into shared memory
        if return_request.value == True:
            for index, _ in enumerate(shared_quality):
                    return_quality[index] = np.nan
                    return_angle[index] = np.nan
                    return_distance[index] = np.nan

            for index, (qual, ang, dist) in enumerate(zip(quality, angle, distance)):
                return_quality[index] = qual
                return_angle[index] = ang
                return_distance[index] = dist
            return_request.value = False

# Define the animation_node function for data visualization
def animation_node(return_angle, return_distance, return_request):
    print("Starting animation_node...")

    # Import Matplotlib for data visualization
    import matplotlib.pyplot as plt

    # Function to update the plot
    def plotter(x, y, duration = 0.001):
        plt.scatter(x, y)
        plt.pause(duration)
        plt.cla()
        plt.draw()

    while True:
        return_request.value = True
        while return_request.value == True:
            time.sleep(0.001)

        # Convert Lidar data to radians and distances for plotting
        radians = np.deg2rad(return_angle[:])
        distances = np.array(return_distance[:])
        x = np.multiply(distances, np.cos(radians))
        y = np.multiply(distances, np.sin(radians))
        plotter(x, y)
 
# Define the recv_data function for receiving and displaying shared data
def recv_data(return_quality, return_angle, return_distance, return_request):
    while True:
        return_request.value = True
        while return_request.value == True:
            print("waiting for response...")
            time.sleep(0.01)
        print("received new data")

        # Print the shared Lidar data
        print(f"shared_quality: {return_quality[:]}")
        print(f"shared_angle: {return_angle[:]}")
        print(f"shared_distance: {return_distance[:]}")

# Main block of code
if __name__ == '__main__':
    print("Starting clean.py...")

    # Define the length of shared arrays and create shared memory objects
    ARRAY_LENGTH = 750
    shared_quality = mp.Array('f', np.nan*np.ones(ARRAY_LENGTH))
    shared_angle = mp.Array('f', np.nan*np.ones(ARRAY_LENGTH))
    shared_distance = mp.Array('f', np.nan*np.ones(ARRAY_LENGTH))
    shared_request = mp.Value('I', False)

    # Create separate processes for Lidar data collection and data visualization
    lidar_process = mp.Process(target = lidar_node, args = (shared_quality, shared_angle, shared_distance, shared_request))
    animation_process = mp.Process(target = animation_node, args = (shared_angle, shared_distance, shared_request))
    
    # Start the processes
    lidar_process.start()
    animation_process.start()
