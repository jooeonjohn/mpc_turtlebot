import numpy as np
import matplotlib.pyplot as plt
import csv

def generate_straight_line(start, end, interval):
    start = np.array(start)
    end = np.array(end)
    distance = np.linalg.norm(end - start)
    num_points = int(distance / interval) + 1
    x_vals = np.linspace(start[0], end[0], num_points)
    y_vals = np.linspace(start[1], end[1], num_points)
    return np.column_stack((x_vals, y_vals))

def generate_circular_arc(center, radius, start_angle, end_angle, interval):
    while end_angle < start_angle:
        end_angle += 2*np.pi
    center = np.array(center)
    arc_length = abs(end_angle - start_angle) * radius
    num_points = int(arc_length / interval) + 1
    angles = np.linspace(start_angle, end_angle, num_points)
    x_vals = center[0] + radius * np.cos(angles)
    y_vals = center[1] + radius * np.sin(angles)
    return np.column_stack((x_vals, y_vals))

def generate_counter_circular_arc(center, radius, start_angle, end_angle, interval):
    while end_angle > start_angle:
        end_angle -= 2*np.pi
    center = np.array(center)
    arc_length = abs(start_angle - end_angle) * radius
    num_points = int(arc_length / interval) + 1
    angles = np.linspace(start_angle, end_angle, num_points)
    x_vals = center[0] + radius * np.cos(angles)
    y_vals = center[1] + radius * np.sin(angles)
    return np.column_stack((x_vals, y_vals))

# Parameters
interval = 0.1

# Path generation
path = []

p1 = (1-0.25*np.sqrt(2), 0.5-0.5/np.sqrt(2))
p2 = (1+0.25*np.sqrt(2), 0.5+0.5/np.sqrt(2))
p3 = (1+0.25*np.sqrt(2), 0.5-0.5/np.sqrt(2))
p4 = (1-0.25*np.sqrt(2), 0.5+0.5/np.sqrt(2))

# 1. Straight line between (0.5, 0) and (2.0, 0)
path.append(generate_straight_line(p1, p2, interval))

# 2. Circular arc from (2.0, 0) to (2.0, 1.0), center at (1.5, 0)
path.append(generate_counter_circular_arc((1+0.5*np.sqrt(2), 0.5), 0.5, np.pi*3/4, -np.pi*3/4, interval))

# 3. Straight line between (2.0, 1.0) and (0.5, 1.0)
path.append(generate_straight_line(p3, p4, interval))

# 4. Circular arc from (0.5, 1.0) to (0.5, 0), center at (1.0, 1.0)
path.append(generate_circular_arc((1-0.5*np.sqrt(2), 0.5), 0.5, np.pi*1/4, -np.pi*1/4, interval))

# p1 = (0.5,0)
# p2 = (1.5,0)
# p3 = (1.5,1.0)
# p4 = (0.5,1.0)

# path.append(generate_straight_line(p1,p2,interval))
# path.append(generate_circular_arc((1.5,0.5),0.5,-np.pi/2,np.pi/2,interval))
# path.append(generate_straight_line(p3,p4,interval))
# path.append(generate_circular_arc((0.5,0.5),0.5,np.pi/2,-np.pi/2,interval))

# Combine paths
path = np.vstack(path)

# Remove repeated points
filtered_path = [path[0]]
for i in range(1, len(path)):
    if not np.array_equal(path[i], path[i - 1]):
        filtered_path.append(path[i])
path = np.array(filtered_path)

# Plotting
plt.figure(figsize=(8, 8))
plt.plot(path[:, 0], path[:, 1], marker='o', markersize=2, linestyle='-', label='Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.title('Generated Path')
plt.legend()
plt.grid()
plt.show()

with open('/home/jooeon/colcon_ws/src/mpc/path/lab_path5.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerows(path)
