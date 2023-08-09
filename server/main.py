import serial
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import concurrent.futures

# Initialize distances and angles arrays
num_anchors = 3
distances = [0] * num_anchors
anchor_positions = np.array([[0, 0, 0], [0, 10, 0], [0, 20, 0]])


# Configure serial ports for each anchor
serialPort = "/dev/ttyUSB1"
serial = serial.Serial(serialPort, 115200, timeout=1)

def read_distance():
    try:
        data = serial.readline().decode()
        print(data)
        id, mac, rssi = data.split(',')
        return (id, int(rssi.split(":")[1])*-1)
    except Exception as e:
        return (0, 0)

def calculate_distance(rssi):
    # Implement your RSSI-to-distance conversion logic here
    # Return the calculated distance
    calculated_distance = pow(10, ((-56-np.double(rssi))/(10*2)))*3.2808
    
    return calculated_distance

def update_distances(anchor):
    rssi = read_distance(anchor)
    return rssi

def multilateration(beacon_positions, distances):
    num_beacons = len(beacon_positions)
    A = np.zeros((num_beacons - 1, 3))
    b = np.zeros((num_beacons - 1, 1))

    # Set up the system of equations
    for i in range(num_beacons - 1):
        beacon1 = beacon_positions[i]
        beacon2 = beacon_positions[i + 1]
        d1 = calculate_distance(distances[i])
        d2 = calculate_distance(distances[i+1])

        A[i] = 2 * (beacon2 - beacon1)
        b[i] = np.linalg.norm(beacon2)**2 - np.linalg.norm(beacon1)**2 - d2**2 + d1**2

    # Solve the system of equations
    estimated_position = np.linalg.lstsq(A, b, rcond=None)
    print(estimated_position)

    return estimated_position


distances = [1,1,1]

time_steps = []
angle_data = [[] for _ in range(num_anchors)]
while True:
    Id, dist = read_distance()
    if dist == 0:
        continue
    distances[int(Id)] = calculate_distance(dist*-1)
    print(distances)

    plt.clf()

    # Create a Circle object for each anchor, with radius set to the corresponding distance value
    circles = [Circle(anchor_positions[i][:2], radius=distances[i], fill=False) for i in range(num_anchors)]

    for i, circle in enumerate(circles):
        plt.gca().add_patch(circle)
        plt.text(anchor_positions[i][0], anchor_positions[i][1], f"Anchor {i+1}", ha='center', va='center')

    plt.axis('equal')
    plt.xlim(-100, 100)
    plt.ylim(-100, 100)
    plt.pause(1)
