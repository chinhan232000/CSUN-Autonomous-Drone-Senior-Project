from rplidar import RPLidar
import multiprocessing as mp
import numpy as np
import time


def lidar_node(return_quality, return_angle, return_distance, return_request):
    print("Starting lidar_node...")
    lidar = RPLidar('/dev/ttyUSB0')
    lidar.reset()
    health = lidar.get_health()
    info = lidar.get_info()
    print(f"Health: {health}")
    print(f"Info: {info}")

    for scan in lidar.iter_scans():
        unpacked_data = list(zip(*scan))
        quality = unpacked_data[0]
        angle = unpacked_data[1]
        distance = unpacked_data[2]

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


def animation_node(return_angle, return_distance, return_request):
    print("Starting animation_node...")
    import matplotlib.pyplot as plt

    def plotter(x, y, duration = 0.001):
        plt.scatter(x, y)
        plt.pause(duration)
        plt.cla()
        plt.draw()

    while True:
        return_request.value = True
        while return_request.value == True:
            time.sleep(0.001)
        radians = np.deg2rad(return_angle[:])
        distances = np.array(return_distance[:])
        x = np.multiply(distances, np.cos(radians))
        y = np.multiply(distances, np.sin(radians))
        plotter(x, y)
 

def recv_data(return_quality, return_angle, return_distance, return_request):
    while True:
        return_request.value = True
        while return_request.value == True:
            print("waiting for response...")
            time.sleep(0.01)
        print("received new data")

        print(f"shared_quality: {return_quality[:]}")
        print(f"shared_angle: {return_angle[:]}")
        print(f"shared_distance: {return_distance[:]}")


if __name__ == '__main__':
    print("Starting clean.py...")
    ARRAY_LENGTH = 750
    shared_quality = mp.Array('f', np.nan*np.ones(ARRAY_LENGTH))
    shared_angle = mp.Array('f', np.nan*np.ones(ARRAY_LENGTH))
    shared_distance = mp.Array('f', np.nan*np.ones(ARRAY_LENGTH))
    shared_request = mp.Value('I', False)

    lidar_process = mp.Process(target = lidar_node, args = (shared_quality, shared_angle, shared_distance, shared_request))
    animation_process = mp.Process(target = animation_node, args = (shared_angle, shared_distance, shared_request))
    
    lidar_process.start()
    animation_process.start()
