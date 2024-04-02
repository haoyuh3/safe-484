import numpy as np
import matplotlib.pyplot as plt
from waypoint_list import WayPoints
# Load CSV data into a NumPy array
data = np.genfromtxt('acceleration_log.csv', delimiter=',', skip_header=0)

# Assuming time interval is 0.01, create time values
time_values = np.arange(len(data)) * 0.01

# Plot acceleration over time
plt.plot(time_values, data, label='Acceleration')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration')
plt.title('Acceleration over Time')
plt.legend()
plt.grid(True)
plt.savefig('acc.png')
plt.show()

x = np.genfromtxt('x_coor.csv', delimiter=',', skip_header=0)
y = np.genfromtxt('y_coor.csv', delimiter=',', skip_header=0)
wp = WayPoints().pos_list
plt.figure()
plt.plot(x, y, color = 'red')

x, y = zip(*wp)
plt.scatter(x, y , color = 'green', label='Way Points')
plt.title('Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.savefig('traj.png')
plt.show()