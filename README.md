# Cloud and Fog Robotics: A Hands-on Tutorial with ROS2 and FogROS2 (ICRA 2024)

At the workshop, we will provide pre-provisioned SSH-able machines for convenience. However, to get the most out of the tutorial, we recommend to install with your local computer. Please refer to [Preparation Doc](./Tutorial_Prep.md) for more information. All of our work is done in the docker, so it will not leave a trace to your computer. 

## PART 0: Verify Running Environment

If you successfully followed the [Preparation Doc](./Tutorial_Prep.md) or get a SSH machine from us, you may run the following command to verify your running environment: 

MacOS / Linux
```
cd ~/CloudRobotics_tutorial
./docker-run.sh
```

Windows
```
docker-run.cmd
```

You should get the following output similar to the following:
```
$ ./docker-run.sh 
Starting with USER: jeffi, UID: 501, GID: 20
jeffi@docker-desktop:~/CloudRobotics_tutorial/fog_ws$ 
```
If you got that, congrats!  Everything is working.  At this point, type `CTRL-D` to exit.


## PART 1: Talker and Listener with FogROS2  (15 Min)

The `talker.py`  and `listener.py` files are provided in the `tutorial_workspace/fogros2_tutorial` folder in the repository. If we wanted, we could run both the talker and listener nodes individually and see the nodes communicating. Instead, we are going to run these nodes using a launch file.

The `talker.local.launch.py` file is provided in the `tutorial_workspace/launch` in the repository.

6. Start the container again
   
```
cd ~/CloudRobotics_tutorial
./docker-run.sh
```

7.  Run local launch file
```
ros2 launch fogros2_tutorial talker.local.launch.py
```
Here you can see both the talker node publishing and the listener node subscribing. 

8. Run cloud launch file (using FogROS2 and AWS)

Now, we take the same local launch file and run the talker node on a provisioned AWS cloud instance. FogROS2 handles the provisioning and setup of the cloud instance for us. 

The `talker.aws.launch.py` file is provided in the `tutorial_workspace/launch` folder in the repository.

```
ros2 launch fogros2_tutorial talker.aws.launch.py
```
This process will take a few minutes, and you'll see a lot of information scroll by as FogROS2 provisions the cloud instance and installs all required software and dependencies. Finally, you would see both the cloud node and the local node communicating. 

CTRL-C kills the local instance (e.g., listener) the first time and then the cloud instance the second time. 


## (Bonus) SAM with FogROS2 (5 Min)
This is a bonus part to run a cloud instance with [Segment Anything Model](https://github.com/facebookresearch/segment-anything) (SAM). Do it only if you are assigned with a physical cloudgripper.

Like in **Part 1**, we have created  `sam_server.py`  and `sam_client.py` which you can look at in the `tutorial_workspace/fogros2_tutorial` folder. We will be running these nodes using two launch files: `sam.aws.launch.py`, which are provided in the `tutorial_workspace/launch` folder in the repository.

run:
```
ros2 launch fogros2_tutorial sam.aws.launch.py
```
In this terminal, we launch a server node on the cloud which subscribes to images of the physical robot, loads a SAM model (in this case, we are using the smallest one: "vit_b"), generates masks from the image and then publishes the generated masks. 
While waiting for the machine to launch, please **open up a new terminal**, and proceed with CloudGripper.

## PART 2: CloudGripper (10 Min)
We integrate FogROS2 with [CloudGripper](https://cloudgripper.org/). To interact with the robot, this part assumes that you have already signed up for the robot. You need the assigned robot ID and a robot access API token. 

9. Start launch files

In the new terminal, run 
```
cd ~/CloudRobotics_tutorial
./docker-run.sh
```

9.1 **Run only if you ran SAM** 
```
export CYCLONEDDS_URI=file:///fog_ws/src/FogROS2/fogros2/configs/cyclonedds.ubuntu.2204.xml
```
This propagate all the messages from CloudGripper to FogROS2-enabled Cloud.


9.2 In the container, run
```
cd /fog_ws/src/tutorial_workspace/launch
ros2 launch cloudgripper.launch.py -- robot_name:=robotX
```
where `robotX` is the robot ID that you are assigned. In this terminal, we launch a client node which receives images of the robot workspace from CloudGripper, publishes the image, and subscribes to generated masks from the cloud. We both the original image from cloudgripper and then generated masks.



#### Keyboard Control
In the new terminal, run 
```
cd ~/CloudRobotics_tutorial
./docker-run.sh
```

10.1 **Run only if you ran SAM** 
```
export CYCLONEDDS_URI=file:///fog_ws/src/FogROS2/fogros2/configs/cyclonedds.ubuntu.2204.xml
```
This propagate all the messages to CloudGripper with the same DDS setup. 

and in the container run 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
This will enable real-time keyboard control for the Cloudgripper.
- **Key Bindings**:
  - **Movement**:
    - `i`: Move forward
    - `k`: Move backward
    - `j`: Move left
    - `l`: Move right
  - **Z-Axis Control**:
    - `t`: Move up
    - `b`: Move down
  - **Rotation Control**:
    - `u`: Rotate left
    - `i`: Rotate right
  - **Gripper Control**:
    - `m`: Close gripper
    - `,`: Open gripper


#### Viewing SAM's results 
If you have ran SAM and CloudGrippper and both are up and running, you can look in ~/CloudRobotics_tutorial/tutorial_workspace/launch/saved_images for both the original image and the saved mask image. 


## PART 4: FOG-RTX Data Collection and Management (5 Min)
All the data is automatically collected through [fog-rt-x](https://github.com/BerkeleyAutomation/fog_x), a cloud based data collection and management. 
[fog_rtx_recorder.py](./tutorial_workspace/fogros2_tutorial/fog_rtx_recorder.py) shows an example of collecting data from various topics and store them to the cloud. 
The website will be statically generated and posted at the end of the workshop at link: https://berkeleyautomation.github.io/CloudRobotics_tutorial/  

