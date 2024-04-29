# Cloud and Fog Robotics: A Hands-on Tutorial with ROS2 and FogROS2 (ICRA 2024)

## Before tutorial instructions

Install Docker Desktop (if not already installed).  Select the appropriate link below and follow the instructions.  For Mac and Windows, this amounts to following the download link, and installing the software.

* [Docker for Mac OS](https://docs.docker.com/desktop/install/mac-install/)
* [Docker for Windows](https://docs.docker.com/desktop/install/windows-install/)
* [Docker for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)




## PART 1 : SETUP AND BASIC ROS

For this bootcamp, we recommend following the instructions below from your home directory.  It should work from other places, but you'll have to do all the path conversions on your own.

1. Clone Github Repository below your home directory
```
cd ~
git clone https://github.com/BerkeleyAutomation/CloudRobotics_tutorial.git
```

If you do not have `git`, you can go to https://github.com/BerkeleyAutomation/CloudRobotics_tutorial.git, then hit the green "code" button, and then "download zip".  Once you have the zip downloaded, extract the files so that `CloudRobotics_tutorial` is in your home directory.
```
cd ~
unzip ~/Downloads/CloudRobotics_tutorial-main.zip
mv CloudRobotics_tutorial-main CloudRobotics_tutorial
```


2. Build docker image and start the docker container


First set the  CLOUDGRIPPER_API_KEY in your environment for authentication with the CloudGripper API. We will need this for a later part. 

```
export CLOUDGRIPPER_API_KEY="your_api_key_here"
```

From the checked out directory, run:

MacOS
```
cd ~/CloudRobotics_tutorial
./docker-build.sh
```

Windows
```
cd CloudRobotics_tutorial
./docker-build.cmd
```

This process may take a few minutes, and you'll see a lot of information scroll by.  If there is no error message, move on to the next step.  If there was an error message, it can usually be resolved by waiting a minute and running `./docker-build.sh` again until it works--most common problems are related to internet connections, either on your computer/wifi or on the server from which docker is downloading software.


To test if the docker build worked, try running:

MacOS
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


If you get:
```
docker: Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?.
```
Then it means you need to start Docker Desktop and wait until the Docker Deskop window shows that it has started.


3. Configure AWS. This requires credentials and will be provided to you during the tutorial. 


**Note: You do not have to run Steps 4 and 5 as we have already set them up. They are included below for completeness as they are required when using ROS2**


4. (Optional) Build ROS2 Workspace

We have premade the workspace folder for you. First start the  container again. 

You should be in the _/fog_ws_ directory which is the workspace folder. 

Run
```
colcon build --symlink-install
```

If you only get
```
--- stderr: xxxxxxx                                                                
/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn(
---
```
Then you are fine. 


5. (Optional) Source the ROS2 Environment
```
. install/setup.bash
```


## PART 2: BASIC TALKER AND LISTENER EXAMPLES IN FOGROS2

The `talker.py`  and `listener.py` files are provided in the `tutorial_workspace/fogros2_tutorial` folder in the repository. If we wanted, we could run both the talker and listener nodes individually and see the nodes communicating. Instead, we are going to run these nodes using a launch file.

The `talker.local.launch.py` file is provided in the `tutorial_workspace/launch` in the repository.

6. Start the container again
   
```
cd ~/CloudRobotics_tutorial
./docker-run.sh
```

7.  (TODO: don't run this yet, or run it after step 8) Run local launch file
```
cd /fog_ws/src/tutorial_workspace/launch/
ros2 launch talker.local.launch.py
```
Here you can see both the talker node publishing and the listener node subscribing. 

8. Run cloud launch file (using FogROS2 and AWS)

Now, we take the same local launch file and run the talker node on a provisioned AWS cloud instance. FogROS2 handles the provisioning and setup of the cloud instance for us. 

The `talker.aws.launch.py` file is provided in the `tutorial_workspace/launch` folder in the repository.

```
cd /fog_ws/src/tutorial_workspace/launch

ros2 launch talker.aws.launch.py
```
This process will take a few minutes, and you'll see a lot of information scroll by as FogROS2 provisions the cloud instance and installs all required software and dependencies. Finally, you would see both the cloud node and the local node communicating. 

CTRL-C kills the local instance (e.g., listener) the first time and then the cloud instance the second time. 


## PART 3: SAM AND CLOUDGRIPPER
Next we will show FogROS2 used to run a cloud instance with Segment Anything Model (SAM). We will be using this with images received from CloudGripper.

Like in **Part 2**, we have created  `sam_server.py`  and `sam_client.py` which you can look at in the `tutorial_workspace/fogros2_tutorial` folder. We will be running these nodes using two launch files: `sam.aws.launch.py`  and `cloudgripper.launch.py` which are provided in the `tutorial_workspace/launch` folder in the repository.

9. Start launch files

Open two terminals and start a container in each terminal.

In terminal 1, run:
```
cd /fog_ws/src/tutorial_workspace/launch

ros2 launch sam.aws.launch.py
```
In this terminal, we launch a server node on the cloud which subscribes to images of the physical robot, loads a SAM model (in this case, we are using the smallest one: "vit_b"), generates masks from the image and then publishes the generated masks. 

In terminal 2, run:
```
cd /fog_ws/src/tutorial_workspace/launch

ros2 launch cloudgripper.launch.py
```
In this terminal, we launch a client node which receives images of the robot workspace from CloudGripper, publishes the image, and subscribes to generated masks from the cloud. We both the original image from cloudgripper and then generated masks.

You can look in /fog_ws/src/tutorial_workspace/launch/saved_images for both the original image and the saved mask image. 


## PART 4: FOG-RTX DATA COLLECTION AND VISUALIZATION
All the data is automatically collected through [fog-rt-x](https://github.com/BerkeleyAutomation/fog_x), a cloud based data collection and management. 
[fog_rtx_recorder.py](./tutorial_workspace/fogros2_tutorial/fog_rtx_recorder.py) shows an example of collecting data from various topics and store them to the cloud. 
The website will be statically generated at the end of the workshop at link: https://berkeleyautomation.github.io/CloudRobotics_tutorial/  