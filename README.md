# On Board

## Table of Contents

1. [General description](#general-description)
2. [Installations](#installations)
3. [Operation modes](#operation-modes)
4. [Docker](#docker)
5. [Docker documentation](#docker-documentation)
6. [Deploying the containers](#deploying-the-containers)

## General description
The modules in this block run in the on board computer (Raspberry Pi) to control different devices in the drone platform (autopilot, camera, leds, etc.) and to save information associated to its execution (air backend). All on board modules are developed in Python. 
Each of the modules in this block has a GitHub repo where you can find the code together with a description, installation instructions and demos. These are the repos:
* *Autopilot service*:
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-AutopilotService-brightgreen.svg)](https://github.com/dronsEETAC/DroneAutopilotDEE) an on-board module that controls the autopilot to execute the commands coming from other modules (arm, takeoff, go to position, etc.).    

* *Camera service*:
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-CameraService-brightgreen.svg)](https://github.com/dronsEETAC/CameraControllerDEE) an on-board module that controls the on-board camera to execute the commands coming from other modules (take a picture, get the video stream, etc.)       
   
* *LEDs service*:
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-LEDsService-brightgreen.svg)](https://github.com/dronsEETAC/LEDsControllerDEE) an on-board module that controls the LEDs of the drone platform to inform of the status of the drone platform, or a servo installed in the platform, as required by other modules.  
    
* *Monitor*:
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-Monitor-brightgreen.svg)](https://github.com/dronsEETAC/MonitorDEE) records on board data for future analysis (for instance, all the messages send through the brokers.

     
* *Air API REST*:
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-Air_API_REST-brightgreen.svg)](https://github.com/dronsEETAC/AirAPIREST) a server that provides data storage and retrieval on board through HTTP basic operations (GET, POST, PUT, DELETE).      


## Installations

In order to run and contribute you must install Python 3.7. We recommend PyCharm as IDE for development.    
Contributions must follow the contribution protocol that you will find in the main repo of the Drone Engineering Ecosystem.
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-MainRepo-brightgreen.svg)](https://github.com/dronsEETAC/DroneEngineeringEcosystemDEE)

To be able to create Docker images and upload them to Docker Hub, Docker Desktop needs to be downloaded, and an account in Docker Hub needs to be created:
- [Install Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [Create account in Docker Hub](https://hub.docker.com/)

## Operation modes
All services in this block can be run in simulation mode and also in production mode. To use the service in simulation mode, clone the repo in your computer and install de requirements. Be also sure that you have running the internal broker at "localhost:1884". When running the service you must specify the communication and operation mode and also which broker must be used as external broker. To do that you must edit the run/debug configuration in PyCharm, as shown in the image, in order to pass the required arguments to the script implementing the service. At least two parameters are required: connection_mode (global or local) and operation_mode (simulation or production). In case of global communication mode, a third argument is requiered indicating the external broker to be used. The different options for ths third argument are shown in this table:

options for external broker (third argument) | Comments    
--- | --- 
hivemq | broker.hivemq.com:8000 with websockets 
hivemq_cert | broker.hivemq.com:8884 with secure websockets  
classpip-cred |classpip.upc.edu:8000 with websockets and credentials (in the fourth and fifth arguments)   
classpip_cert | classpip.upc.edu:8883 with secure websockets and credentials (in the fourth and fifth arguments)   
localhost | localhost:8000 with websockets   
localhost_cert | localhost:8883 with secure websockets

In case the external broker requieres credentials, two additional parameters must be includes (username and password). The figure shows and example where the external broker does not requires credentials.   
![runConfig](https://github.com/dronsEETAC/DEE_OnBoard/assets/100842082/09c20edf-552f-436a-87bd-90192d75a299)

## Docker

The usage of Docker in the case of the administration of onboard services can be seen as complex at the first time, but the structure which is followed is the easiest one at the time of executing the different containers and communicating with all of them.

Among all the onboard services, and the air backend, three different images are going to be created:
1. The first one, involves Autopilot Service, Camera Service, LEDs Service and boot.py
2. The second one, takes the Monitor Service
3. The last one, is the image which involves the newest implementation of the onboard structure, the REST API from the air backend

![dockerStructure](https://github.com/JordiLlaveria/OnBoardServicesDEE/blob/manager/EstructuraDocker.PNG)

In order to create both images from the second and the third case, they can both be created directly using the GitHub repository and all the information it contains, as the structure based on the Dockerfile, and the .py files associated are already made following the Docker structure.

The case where several changes need to be made is in the first image, the one that involves most of the services, and it is needed to collect in a single directory the following files:
1. requierements.txt and Dockerfile files from this repository
2. AutopilotService.py file from the Autopilot Service repository
3. CameraService.py file from the Camera Service repository
4. LEDsService.py file from the LEDs Service repository
5. boot.py file from the DroneEngineeringEcosystem repository: [DEE repository](https://github.com/dronsEETAC/DroneEngineeringEcosystemDEE)

Once all these files are collected into a single directory and each one of them is downloaded from their corresponding repository, they have the correct organization to create the image of services which involves all of them, following instructions which will be detailed in the next chapter.

All these services, once used in production mode, are going to be executed inside a single Docker container. Because of that, when any modification is made to any of them, there's a need to generate a new image, and upload it to Docker Hub.

This image follows the structure indicated in the Dockerfile, collecting all the required dependencies and libraries, needed to execute all the processes of the services.

The way to generate an image is by executing the following instruction, inside the directory where Dockerfile is found:

```
docker build --platform linux/arm64/v8 -t “Docker Hub username”/”image name”:”version” .
```

Being an example:

```
docker build --platform linux/arm64/v8 -t jordillaveria/services_arm64:v11 .
```

__Once the image is created locally__, it can be found inside the Docker Desktop application, and at this moment, it can be pushed into Docker Hub repository, in order to be available for download from the RPi:

```
docker push “Docker Hub username”/”image name”:”versión”
```

## Docker documentation

The following PDF file was developed with the intention of being a quick guide to the usage of Docker, with basic instructions that may be useful to know, how to install this software into a RPi, and several errors experienced that may help new users if experiencing them.

[DockerDocumentation.pdf](https://github.com/JordiLlaveria/OnBoardServicesDEE/blob/manager/DockerIntroduction.pdf)

This is a video tutorial on Docker for en DEE: [![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-video_tutorial_docker-pink.svg)](https://www.youtube.com/playlist?list=PLyAtSQhMsD4oEZIui7W1NN0lEU1pUmxYg)

    
## Deploying the containers

When all these three images are created, and we will suppose that these processes are going to be executed inside de RPi with both Docker and Docker Compose tools downloaded, following the previous documentation, it's time to deploy the corresponding containers, to be able to use the services all of them contain.

For ongoing this process to continue, the docker-compose.yml file found in this repository needs to be added to the RPi, either by transferring it through PFSTP, or creating it manually and indicating all the configurations needed.

At the moment when this file is found inside the RPi, inside the directory where it is found, the following command can be executed to deploy all the containers needed, which use as a reference the images already created:

```
docker compose up
```

A parenthesis needs to be made, as depending on the version of Docker installed, the previous command, and the following ones, can be written as __docker compose__ or as __docker-compose__, and the quickest way to know which one is the corresponding to the version used, is trying any of them and see if Docker does not report any error.

This instruction downloads all the required images from Docker Hub, creates all the indicated containers in the docker-compose.yml, and deploys them to be available as soon as possible for the user, without needing any additional instruction.

Despite that, and to be able to deal with the usage and Docker, the following instructions are useful at the time of using Docker:
- docker compose up -d: same instruction as before, but in this case the terminal does not report any log, allowing the user to continue using it
- docker compose stop: stops all the containers
- docker compose down: stops and eliminates all the containers collected inside a Docker Compose file
- docker ps -a: show all the containers created, either in "UP" or "DOWN" state

       
## Starting on-board services
When operating in production mode, the on-board services must be run in the on-board computer.   
In this repo you will find a python script (boot.py) that can be used to that purpose. 
All on-board services and boot.py must be downloaded in the on-board computer and the requirements must be installed. Of course, the mosquitto broker must also be running on-board. 
The services can be started with this command:
```
sudo python3 boot.py parameters
```
The boot script will detect if there is internet coverage. If not, the green led will keep fixed and all the services will be started in local and production modes.  
If there is internet coverage then the user can select the communication model: green led indicates local mode and blue led color indicates global mode. The user can change the mode with the on board-button. If the button is not pressed during 20 seconds the led will keep fixed, the communication mode will be selected and the services will start accordingly.    
If you are planning to work in global mode you must provide one addicional parameter to the boot.py script. This parameter is the broker that must be used as external broker (either 'broker.hivemq.com' or 'classpip.upc.edu'). In case you choose the second option then you must provide two additional parameters: username and password.

