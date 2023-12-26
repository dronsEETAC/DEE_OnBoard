FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

RUN apt-get update
RUN apt-get install -y python3.7
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-dev
RUN apt-get install -y sudo
RUN apt-get install -y nano

# LEDs
RUN apt install -y python3-gpiozero
RUN sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel
RUN sudo python3 -m pip install --force-reinstall adafruit-blinka
RUN apt-get install -y build-essential
RUN sudo pip install schedule

# Camera
RUN apt-get install -y libgl1-mesa-glx
RUN apt-get install -y --no-install-recommends libgtk2.0-dev
RUN sudo pip3 install -U opencv-contrib-python
RUN apt-get install -y --no-install-recommends portaudio19-dev
RUN sudo apt install -y libraspberrypi-bin
RUN sudo apt install -y libraspberrypi0
RUN sudo apt install -y libraspberrypi-dev
RUN sudo pip install -U vidgear[core]


RUN mkdir /app
WORKDIR /app
ADD . /app

RUN pip install -r requirements.txt

EXPOSE 27017


