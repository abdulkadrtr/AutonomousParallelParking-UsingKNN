# AutonomousParallelParking-UsingKNN

An autonomous vehicle that searches for parking spots and performs parallel parking all on its own! We utilized the KNN algorithm, sensor data, ROS2, and the Webots Tesla model to create this innovative and cutting-edge technology.
The KNN algorithm is powered by data collected from 110 manual parking maneuvers.

Our autonomous vehicle is capable of navigating through any environment and detecting available parking spots with ease. Once a spot has been identified, the vehicle uses the KNN algorithm to determine the optimal parking trajectory.

With the help of ROS2, our autonomous vehicle is able to seamlessly communicate with various sensors and systems to ensure a safe and efficient parking experience. And with the Webots Tesla model.

![foto](https://user-images.githubusercontent.com/87595266/222945324-a38971a5-57c4-4d4b-8f34-1ff51360d5c0.png)


# Youtube Project Introduction & Demo

https://youtu.be/Rp8lRX7t5Qg

# How to Create a Project Environment ? 

To run the project, you will need the ROS2 Humble and webots_ros2 packages, as well as the webots simulation environment. 
Please follow the instructions below. These instructions assume that you have installed ROS2 Humble and the Webots simulation environment.

- Create a ros2_ws folder and create an src folder inside the directory. Then, follow the instructions under the 'Install webots_ros2 from sources' section in the link below.

    [Install webots_ros2 from sources](https://github.com/cyberbotics/webots_ros2/wiki/Linux-Installation-Guide#install-webots_ros2-from-sources)
    
- Next, replace the 2 files located in the editedFiles folder of the repository with the files specified in the address below. 
This modification will allow for the addition of new elements in the simulation environment.

    robot_launch.py -> `/ros2_ws/src/webots_ros2/webots_ros2_tesla/launch`
    
    
    tesla_world.wbt -> `/ros2_ws/src/webots_ros2/webots_ros2_tesla/worlds`
    
- Afterwards, copy the autonomous_park folder to the /src directory and compile the project using `colcon build`.

- The autonomous parking simulation environment is now ready! You can test it by running the following commands.

    `ros2 launch webots_ros2_tesla robot_launch.py`

    `ros2 run autonomous_park parking`
    
 # Autonomous Parking Dataset
 
You can find the dataset used by the KNN algorithm under the data folder. Each file in this folder contains sensor data recorded during a manual parking process. There are a total of 110 manual parking records. The format of each line in these files is as follows: the first 32 values represent the front lidar, the next 64 values represent the right lidar, the next 3 values represent the GPS location, and the last 4 values represent the Ackermann controls (steering angle, speed, acceleration, jerk). You can use this data to implement your own artificial intelligence project. 
    
    

