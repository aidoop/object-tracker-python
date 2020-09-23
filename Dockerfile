# Use an official ubuntu image
FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

# Set the working directory to /app
WORKDIR /app

# Copy the current files to docker working directory
COPY . .

RUN apt-get update \
    && apt-get install -y python3-pip python3-dev \
    && cd /usr/local/bin \
    && ln -s /usr/bin/python3 python \
    && pip3 install --upgrade pip

RUN apt-get install -y libgl1-mesa-glx libusb-1.0-0 libqt5x11extras5 \
    && pip3 install -r requirements.txt

ENTRYPOINT /bin/bash
