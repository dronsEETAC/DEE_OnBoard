
from typing import Any

import cv2 as cv
import shutil
import numpy as np
import paho.mqtt.client as mqtt
import base64
import threading
import time
import random
import string
import requests

def generate_random_number(length):
    for n in range(1,length):
        caracters = string.digits
        random_name = ''.join(random.choice(caracters) for _ in range(length))
    return random_name

def send_video_stream( origin, client):
    global sending_video_stream
    global cap
    topic_to_publish = f"cameraService/{origin}/videoFrame"


    while sending_video_stream:
        # Read Frame
        ret, frame = cap.read()
        if ret:
            _, image_buffer = cv.imencode(".jpg", frame)
            jpg_as_text = base64.b64encode(image_buffer)
            client.publish(topic_to_publish, jpg_as_text)
            time.sleep(0.2)



def process_message(message, client):
    global sending_video_stream
    global cap
    global recording
    #global images

    splited = message.topic.split("/")
    origin = splited[0]
    command = splited[2]
    recording = False
    print('recibo ', command)

    if not cap.isOpened():
        cap = cv.VideoCapture(0)

    if command == "takePicture":
        print("Take picture")
        ret = False
        for n in range(1, 20):
            # this loop is required to discard first frames
            ret, frame = cap.read()
        _, image_buffer = cv.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(image_buffer)

        # Sending image to Dashboard
        client.publish("cameraService/" + origin + "/picture", jpg_as_text)

    if command == "startVideoStream":
        print('start video stream')
        sending_video_stream = True
        w = threading.Thread(
            target=send_video_stream,
            args=(
                origin,
                client
            ),
        )
        w.start()

    if command == "stopVideoStream":
        print('stop video stream')
        sending_video_stream = False

    if command == "takePictureFlightPlan":
        print("Take picture for Flight Plan")
        ret = False
        for n in range(1, 20):
            # this loop is required to discard first frames
            ret, frame = cap.read()
        _, image_buffer = cv.imencode('.jpg', frame)
        if ret:
            random_number = generate_random_number(3)
            random_name = "picture_" + random_number + ".jpg"
            route = 'Pictures/' + random_name
            # Guardar imagen provisionalmente
            cv.imwrite(route, frame)
            internal_client.publish(origin + "/autopilotService/savePicture", random_name)

    if command == "takePictureInterval":
        print("Take picture by interval")
        ret = False
        for n in range(1, 20):
            # this loop is required to discard first frames
            ret, frame = cap.read()
        _, image_buffer = cv.imencode('.jpg', frame)
        #jpg_as_text = base64.b64encode(image_buffer)
        if ret:
            random_number = generate_random_number(3)
            random_name = "pictureInterval_" + random_number + ".jpg"
            route = 'Pictures/' + random_name
            cv.imwrite(route, frame)
            internal_client.publish(origin + "/autopilotService/savePictureInterval", random_name)


    if command == "startVideoMoving":
        def start_recording():
            fourcc = cv.VideoWriter_fourcc(*"mp4v")
            random_number = generate_random_number(3)
            random_name = "videoMoving_" + random_number + ".mp4"
            route = 'Videos/' + random_name
            output_video = cv.VideoWriter(route, fourcc, 30.0, (640, 480))
            print("Filmando video " + random_name)
            while recording == True:
                ret, frame = cap.read()
                output_video.write(frame)
            cv.destroyAllWindows()
            output_video.release()
            print ("Video grabado")
            internal_client.publish(origin + "/autopilotService/saveVideo", random_name)
        if recording == False:
            recording = True
            recording_thread = threading.Thread(target=start_recording)
            recording_thread.start()

    if command == "endVideoMoving":
        recording = False
        print ("Video se tiene que parar")

    if command == "startStaticVideo":
        durationVideo = int(message.payload.decode("utf-8"))
        def start_recording_static_video(duration):
            fourcc = cv.VideoWriter_fourcc(*"mp4v")
            random_number = generate_random_number(3)
            random_name = "staticVideo_" + random_number + ".mp4"
            print ("Filmando video " + random_name)
            route = 'Videos/' + random_name
            output_video = cv.VideoWriter(route, fourcc, 30.0, (640, 480))
            start_time = time.time()
            while time.time() - start_time < duration:
                ret, frame = cap.read()
                output_video.write(frame)
            output_video.release()
            print("Video estático grabado")
            internal_client.publish(origin + "/autopilotService/saveVideo", random_name)
            # shutil.move(random_name, "/Videos/" + random_name)
        recording_thread = threading.Thread(target=start_recording_static_video(durationVideo))
        recording_thread.start()

    if command == "saveMediaApi":
        flight_id = message.payload.decode("utf-8")
        response_json = requests.get('http://192.168.208.6:9000/get_results_flight/' + flight_id).json()
        pictures = response_json["Pictures"]
        for picture in pictures[0:]:
            picture_name = picture["namePicture"]
            image_path = 'Pictures/' + picture_name
            print(f"Buscando picture: {image_path}")
            with open(image_path, 'rb') as file:
                image_buffer = file.read()
            requests.post(f"http://147.83.249.79:8105/save_picture/{picture_name}", image_buffer)
        videos = response_json["Videos"]
        for video in videos[0:]:
            video_name = video["nameVideo"]
            video_path = 'Videos/' + video_name
            print(f"Buscando video: {video_path}")
            with open(video_path, 'rb') as file:
                video_buffer = file.read()
            requests.post(f"http://147.83.249.79:8105/save_video/{video_name}", video_buffer)


def on_internal_message(client, userdata, message):
    print ('recibo internal ', message.topic)
    global internal_client
    process_message(message, internal_client)

def on_external_message(client, userdata, message):
    print ('recibo external ', message.topic)

    global external_client
    process_message(message, external_client)

def CameraService (connection_mode, operation_mode, external_broker, username, password):
    global op_mode
    global external_client
    global internal_client
    global state
    global cap

    sending_video_stream = False

    cap = cv.VideoCapture(0)  # video capture source camera (Here webcam of lap>

    print ('Camera ready')

    print ('Connection mode: ', connection_mode)
    print ('Operation mode: ', operation_mode)
    op_mode = operation_mode

    # The internal broker is always (global or local mode) at localhost:1884
    internal_broker_address = '192.168.208.2'
    #internal_broker_address = 'localhost'

    internal_broker_port = 1884


    if connection_mode == 'global':
        external_broker_address = external_broker
    else:
        external_broker_address = '192.168.208.2'


    print ('External broker: ', external_broker_address)



    # the external broker must run always in port 8000
    external_broker_port = 8000



    external_client = mqtt.Client("Camera_external", transport="websockets")
    if external_broker_address == 'classpip.upc.edu':
        external_client.username_pw_set(username, password)

    external_client.on_message = on_external_message
    external_client.connect(external_broker_address, external_broker_port)


    internal_client = mqtt.Client("Camera_internal")
    internal_client.on_message = on_internal_message
    internal_client.connect(internal_broker_address, internal_broker_port)

    print("Waiting....")
    external_client.subscribe("+/cameraService/#", 2)
    internal_client.subscribe("+/cameraService/#")
    internal_client.loop_start()
    external_client.loop_forever()

if __name__ == '__main__':
    import sys
    connection_mode = sys.argv[1] # global or local
    operation_mode = sys.argv[2] # simulation or production
    username = None
    password = None
    if connection_mode == 'global':
        external_broker = sys.argv[3]
        if external_broker == 'classpip.upc.edu':
            username = sys.argv[4]
            password = sys.argv[5]
    else:
        external_broker = None

    CameraService(connection_mode,operation_mode, external_broker, username, password)

