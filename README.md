# OnBoard services

## Table of Contents

1. [Introduction](#introduction)
2. [Installations](#installations)
1. [Autopilot service](#autopilot-service)
2. [Camera service](#camera-service)
3. [LEDs service](#LEDs-service)
4. [Docker](#docker)

## Introduction

The idea of collecting all these three services together, is to simulate the behavior that they will experience inside the Docker container executed inside the drone. When explaining each of them, it will be detailed the different actions they can process, and the processes they can execute.

## Installations

In order to run and contribute you must install Python 3.7. We recommend PyCharm as IDE for development.    
Contributions must follow the contribution protocol that you will find in the main repo of the Drone Engineering Ecosystem.
[![DroneEngineeringEcosystem Badge](https://img.shields.io/badge/DEE-MainRepo-brightgreen.svg)](https://github.com/dronsEETAC/DroneEngineeringEcosystemDEE)

# Autopilot service  

## Introduction

The autopilot service is an on-board module that controls the operation of the flight controller, as required by the rest of modules in the Drone Engineering Ecosystem.   
Dashboard or mobile applications will requiere the autopilot service to connect to the flight controller, to arm the drone, take-off, go to a certain position or move in a given direction, land, stop, etc. See the table bellow for a complete list of commands that can be accepted by the autopilot service in its current version.

## Commands
In order to send a command to the autopilot service, a module must publish a message in the external (or internal) broker. The topic of the message must be in the form:
```
"XXX/autopilotService/YYY"
```
where XXX is the name of the module requiring the service and YYY is the name of the service that is required. Oviously, some of the commands may require additional data that must be included in the payload of the message to be published. 
In some cases, after completing the service requiered the autopilot service publish a message as an answer. The topic of the answer has the format:
```
"autopilotService/XXX/ZZZ"
```
where XXX is the name of the module requiring the service and ZZZ is the answer. The message can include data in the message payload.

The table bellow indicates all the commands that are accepted by the autopilot service in the current version.   

Command | Description | Payload | Answer | Answer payload
--- | --- | --- | --- |--- 
*connect* | connect with the simulator or the flight controller depending on the operation mode | No | No (see Note 1) | No
*armDrone* | arms the drone (either simulated or real) | No | NO (see Note 2) | No 
*takeOff* | get the drone take off to reach and altitude of 5 meters | No | No (see Note 3)  | No 
*returnToLaunch* | go to launch position |No  | No (see Note 4) | No    
*land* | the dron will land |No  | No (see Note 5) | No     
*disarmDrone* | disarm the drone |No  |  No (see Note 6) | No 
*go* | move in certain direction |"North", "South", "East", "West", "NorthWest", "NorthEast", "SouthWest", "SouthEast" , "Stop"  | No | No 
*disconnect* | disconnect from the simulator or the flight controller depending on the operation mode | No | NO (see Note 1) | No
*executeFlightPlan* | execute the flight plan received | See Note 7 | see Note 7 | see Note 7
*executeFlightPlanMobileApp* | execute the flight plan received from Mobile App | See Note 8 | see Note 8 | see Note 8
*savePicture* | saves a picture in the air APIREST module| "name_picture" | See Note 9 | See Note 9 
*savePictureInterval* | saves a picture, taken every certain interval, in the air APIREST module | "name_picture"  | See Note 10 | See Note 10
*saveVideo* | saves a video in the air APIREST module | "name_video" | See Note 11 | See Note 11
*saveMediaApi* | when the drone has landed after a planned flight, sends its associated information to ground APIREST module | No | See Note 12 | See Note 12


Note 1    
When the autopilot is connected the service will start sending telemetry_info packets every 250 miliseconds. The service will stop sending 
telemetry_info packets as soon as a *disconnect* command is received. This is an example of telemetry_info packet:

```
{
    'lat': 41.124567,
    'lon': 1.9889145,
    'heading': 270,
    'groundSpeed': 4.27,
    'altitude': 6.78,
    'battery': 80,
    'state': state
}

```
The packet includes the state of the autopilot so that the module that receives the packet can take decisions (for instance, change color of the buttons).   

These are the different values for the state:
*'connected'*  
*'arming'*  
*'armed'*   
*'disarmed'*    
*'takingOff'*      
*'flying'*   
*'returningHome'*     
*'landing'*    
*'onHearth'*    

Note 2    
The state will change to *arming* and then to *armed* as soon as the autopilot is armed.    
      
Note 3    
The state will change to *takingOff* and then to *flying* as soon as the autopilot reaches 5 meters of altitude.  
   
Note 4    
The state will change to *returningHome* and then to *onHearth* as soon as the autopilot in on hearth.    
   
Note 5    
The state will change to *landing* and then to *onHearth* as soon as the autopilot in on hearth.    
   
Note 6    
The state will change to *disarmed*.   

Note 7     
The service must receive a json object specifying the flight plan with indications whether a picture must be taken, or a video must be recorded, when reaching a waypoint. This is an example of such json object:    

```
[
  {
    'lat': 41.124567,
    'lon': 1.9889145,
    'takePic': True or False,
    'videoStart': True or False,
    'videoStop': True or False,
    'staticVideo': True or False
  },
  {
    'lat': 41.124567,
    'lon': 1.9889145,
    'takePic': True or False,
    'videoStart': True or False,
    'videoStop': True or False,
    'staticVideo': True or False
  },
  ....
]
```

Also, __the id that identifies the flight plan and the flight executed are also received__, related with saving future information in the APIREST module.

The service will execute the flight plan, changing the state accordingly (*'arming'*, *'armed'*, *'takingOff'*, and so on until *'onHearth'*).    
When arrived to the next waypoint the service will publish this message: *'XXXX/autopilotService/waypointReached'*,, being XXXX the module requesting the service. The topic of the message is a json object containing *'lat'* and *'lon'* of the reached waypoint. 

If a picture must be taken in this waypoint, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/cameraService/takePicture'*. 
If a video must be started in this waypoint, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/cameraService/startVideoMoving'*. 
If a video must be ended in this waypoint, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/cameraService/endVideoMoving*.
If a static video must be recorded in this waypoint, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/cameraService/startStaticVideo'*.

The autopilot will return to launch after the last waypoint is reached. 

Note 8     
The service acts similarly combining instruction _connect_ and _executeFlightPlan_, as this connection from Flutter MobileApp needs both configurations to the able to execute the flight plan sent.
In this case, it also receives a object specifying the flight plan, with the similar indications as __Note 7__.
  
```
[
  {
    'lat': 41.124567,
    'lon': 1.9889145,
    'takePic': True or False,
    'movingVideo': True or False,
    'staticVideo': True or False
  }
  ....
]
```

Note 9     
Asked by the camera when it has taken a picture, saves it into de air APIREST module uploading the flight which is being executed.  
Sends a request to the air APIREST module (/add_picture) to update the flight executed.

Note 10     
Asked by the camera when it has taken a picture after a certain interval, saves it into de air APIREST module uploading the flight which is being executed.  
Sends a request to the air APIREST module (/add_picture) to update the flight executed.

Note 11     
Asked by the camera when it has recorded a video, saves it into de air APIREST module uploading the flight which is being executed.  
Sends a request to the air APIREST module (/add_video) to update the flight executed.

Note 12     
When a flight has ended, and being requested from the Dashboard application, sends the flight, with all its corresponding information, to the ground APIREST module.  
Sends a request to the __ground__ APIREST module (/add_flight) to save the flight ended.

In order to communicate the the camera service that this information needs to be sent, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/cameraService/saveMediaApi'*. 

# Camera service
## Introduction
The camera service is an on-board module that provides images and videos to the rest of modules of the ecosystem, as required.
Dashboard or mobile applications will requiere the camera service to provide a single picturem, to record a video or to stard/stop a video stream.

## Commands
In order to send a command to the camera service, a module must publish a message in the external (or internal) broker. The topic of the message must be in the form:
```
"XXX/cameraService/YYY"
```
where XXX is the name of the module requiring the service and YYY is the name of the service that is required. Some of the commands may require additional data that must be include in the payload of the message to be published.
In some cases, after completing the service requiered the camera service publish a message as an answer. The topic of the answer has the format:
```
"cameraService/XXX/ZZZ"
```
where XXX is the name of the module requiring the service and ZZZ is the answer. The message can include data in the message payload.

The table bellow indicates all the commands that are accepted by the canera service in the current version.

Command | Description | Payload | Answer | Answer payload
--- | --- | --- | --- |---
*takePicture* | provides a picture | No | *picture* | Yes (see Note 1)
*takePictureFlightPlan* | provides a picture | No | *picture* | Yes (see Note 1)
*takePictureInterval* | provides a picture every certain interval| No | *picture* | Yes (see Note 2)
*startVideoMoving* | starts the recording of a video | No | *video* | Yes (see Note 3)
*endVideoMoving* | end the recording of a video | No | No | No (see Note 4)
*startStaticVideo* | records a static video | No | *video* | Yes (see Note 5)
*saveMediaApi* | sends images and videos to ground APIREST module | No | No | Yes (see Note 6)
*startVideoStream* | starts sending pictures every 0.2 seconds | No | No | No
*stopVideoStream* | stop sending pictures | No | No | No

Note 1
Pictures are encoded in base64, as shown here:
```
ret, frame = cap.read()
if ret:
  _, image_buffer = cv.imencode(".jpg", frame)
  jpg_as_text = base64.b64encode(image_buffer)
  client.publish(topic_to_publish, jpg_as_text)
```

In order to save this picture, following process is executed:
```
random_number = generate_random_number(3)
random_name = "picture_" + random_number + ".jpg"
route = 'Pictures/' + random_name
cv.imwrite(route, frame)
```

When the picture is taken, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/autopilotService/savePicture'*. 

Note 2
The process is the same as in the case of a simple picture, but the way of saving it is different, changing the name of the file:
```
random_number = generate_random_number(3)
random_name = "pictureInterval_" + random_number + ".jpg"
route = 'Pictures/' + random_name
cv.imwrite(route, frame)
```

When the picture is taken, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/autopilotService/savePictureInterval'*. 

Note 3
Videos are encoded in base64, as shown here:
```
fourcc = cv.VideoWriter_fourcc(*"mp4v")
output_video = cv.VideoWriter(route, fourcc, 30.0, (640, 480))
while recording == True:
  ret, frame = cap.read()
  output_video.write(frame)
cv.destroyAllWindows()
output_video.release()
```

When the video is recorded, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/autopilotService/saveVideo'*. 

Note 4
Basically, it changes state of _recording_ variable state into False, to end the recording of a video.

Note 5
Videos are recorded in the same way as in __Note 3__, but in this case, _recording_ becomes False after a certain interval of time.

When the video is recorded, the service will publish this message IN THE INTERNAL BROKER: *'XXXX/autopilotService/saveVideo'*. 

Note 6
Basically, once a flight has ended, gets all the images and videos taken and sends them one by one to the ground APIREST module, to have them available at any time in the Classpip server.

# LEDs service

## Introduction

The LEDs service is an on-board module that controls the LEDs and the servo installed in the drone platform, as required by the rest of modules in the Drone Engineering Ecosystem.
Dashboard or mobile applications will requiere the LEDs service to light certain LED with a given RGB color of to move the servo to drop and object.

## Commands
In order to send a command to the LEDs service, a module must publish a message in the external (or internal) broker. The topic of the message must be in the form:
```
"XXX/LEDsService/YYY"
```
where XXX is the name of the module requiring the service and YYY is the name of the service that is required. Oviously, some of the commands include data that must be includes in the payload of the message to be published.

The table bellow indicates all the commands that are accepted by the LEDs service in the current version.

Command | Description | Payload
--- | --- | ---
*startLEDsSequence* | start a clyclic sequence: red, green, yellow | No
*stopLEDsSequence* | stops the sequence | No
*LEDsSequenceForNSeconds* | runs the cyclic sequience during a certain number of seconds | the number of seconds as string
*red* | put in red the first led for 5 seconds | No
*green* | put in green the second led for 5 seconds | No
*blue* | put in lue the third led for 5 seconds | No
*drop* | move the servo to drop the object | No
*reset* | move the servo to its initial position | No
*bluei* | fix the first led to blue | No
*redi* | fix the first led to red | No
*yellowi* | fix the first led to yellow | No
*greeni* | fix the first led to green | No
*pinki* | fix the first led to pink | No
*whitei* | fix the first led to white | No
*blacki* | fix the first led to black | No
*clear* | clear the first led | No

## Docker

This service, once used in production mode, needs to be executed inside a Docker container, as is shown in the representation of the Drone Engineering Ecosystem. Because of that, when any modification is made to it, there's a need to generate a new image, and upload it to Docker Hub.

The way of generating a new image of it is:

```
docker build --platform linux/arm64/v8 -t “Docker Hub username”/”image name”:”version” .
```

Being an example:

```
docker build --platform linux/arm64/v8 -t jordillaveria/services_arm64:v11 .
```

__Once this image is created__, it can be found inside the Docker Desktop application, and at this moment, it can be pushed into Docker Hub repository, in order to be available for download from the RPi:

```
docker push “Docker Hub username”/”image name”:”versión”
```
