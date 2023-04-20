from scipy.interpolate import InterpolatedUnivariateSpline
import numpy as np
import matplotlib.pyplot as plt
import csv

# Read in the waypoints
x = []
y = []
yaw = []
speed = []
path = "/home/rithwik/UPenn/ESE615/sim_ws/src/lab-5-slam-and-pure-pursuit-hot-wheels/pure_pursuit/waypoints/"
with open(path+'just_to_interpolate.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        x.append(float(row[0]))
        y.append(float(row[1]))
        yaw.append(float(row[2]))
        speed.append(float(row[3]))

axes = np.linspace(1, len(x), len(x))

# Interpolate the waypoints
x_spline = InterpolatedUnivariateSpline(axes, x)
y_spline = InterpolatedUnivariateSpline(axes, y)
yaw_spline = InterpolatedUnivariateSpline(axes, yaw)
speed_spline = InterpolatedUnivariateSpline(axes, speed)

# Sample the interpolated waypoints
number_of_samples = 100
axes_sampled = np.linspace(1, len(x), number_of_samples)

x_sampled = x_spline(axes_sampled)
y_sampled = y_spline(axes_sampled)
yaw_sampled = yaw_spline(axes_sampled)
speed_sampled = speed_spline(axes_sampled)

# Save the interpolated waypoints
with open(path+'dense_straight_only.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')

    for i in range(len(x_sampled)):
        writer.writerow([x_sampled[i], y_sampled[i], yaw_sampled[i], speed_sampled[i]])

# Plot the interpolated waypoints
plt.plot(x_sampled, y_sampled, 'r')
plt.plot(x_sampled, y_sampled, 'rx')
plt.plot(x, y, 'b--')
plt.show()



