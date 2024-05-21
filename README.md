# obs_avoidance_ros_ideas

## Multi-Sensor Integration:
### Nodes:
front_sensor_sub: Subscribes to the front sensor topic.
rear_sensor_sub: Subscribes to the rear sensor topic.
cmd_publisher: Publishes aggregated velocity commands.
### Functionality:
Merge data from both sensors.
Compute a safe path and publish commands to avoid obstacles detected by any of the sensors.

## Basic Obstacle Detection and Avoidance:
### Nodes:
sensor_subscriber: Subscribes to sensor data (e.g., /scan for LiDAR).
cmd_publisher: Publishes velocity commands to avoid obstacles.
### Functionality:
Read sensor data and detect obstacles within a certain range.
Publish velocity commands to steer the robot away from obstacles.

## Simple Wall Follower:
### Nodes:
sensor_subscriber: Subscribes to the sensor topic to get distance measurements.
cmd_publisher: Publishes velocity commands to follow the wall.
### Functionality:
Detect the distance to the wall.
Adjust the robot's path to maintain a constant distance from the wall.

## Dynamic Obstacle Avoidance:
### Nodes:
sensor_subscriber: Subscribes to the sensor topic.
cmd_publisher: Publishes velocity commands to avoid both static and dynamic obstacles.
### Functionality:
Track moving obstacles using consecutive sensor readings.
Adjust the robot's path dynamically to avoid collisions with moving obstacles.

## Grid-based Navigation:
### Nodes:
grid_subscriber: Subscribes to an occupancy grid topic (e.g., /map).
cmd_publisher: Publishes navigation commands based on the grid.
### Functionality:
Read the occupancy grid and identify free and occupied cells.
Plan a path through the free cells and publish commands to follow the path while avoiding obstacles.
