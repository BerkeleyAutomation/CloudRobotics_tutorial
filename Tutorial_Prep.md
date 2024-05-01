
# Prepare for Tutorial 

At the workshop, we will provide pre-provisioned SSH-able machines for convenience. However, to get the most out of the tutorial, we recommend to install with your local computer.

## Note to Mac M-Chip Users
If you plan to use laptop with Apple M-chip or any specialized hardware, we recommend to use our pre-provisioned Cloud VM or using Ubuntu Virtual machine with VMware Fusion or Parallel Desktop. 


## Steps 
Install Docker Desktop (if not already installed).  Select the appropriate link below and follow the instructions.  For Mac and Windows, this amounts to following the download link, and installing the software.

* [Docker for Mac OS](https://docs.docker.com/desktop/install/mac-install/)
* [Docker for Windows](https://docs.docker.com/desktop/install/windows-install/)
* [Docker for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)


For this bootcamp, we recommend following the instructions below from your home directory.  It should work from other places, but you'll have to do all the path conversions on your own.

### Step 1: Clone Github Repository below your home directory
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


### Step 2: Build docker image and start the docker container


First set the  CLOUDGRIPPER_API_KEY in your environment for authentication with the CloudGripper API. We will need this for a later part. 

```
export CLOUDGRIPPER_API_KEY="your_api_key_here"
```

From the checked out directory, run:

MacOS / Linux
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


### Step 3: Configure AWS
In the workshop, we will use AWS cloud machines, but FogROS2 and CloudGripper are not limited by AWS. We will provide AWS credentials at the tutorial. If you wish to register your own account, please refer to [AWS.md](./AWS.md).