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

#ENTRYPOINT ["python3"]

# Install the required packages

# RUN apt-get update -o Acquire::CompressionTypes::Order::=gz
# RUN apt-get upgrade -y

# RUN echo "deb http://ftp.de.debian.org/debian sid main" > /etc/apt/sources.list

# RUN apt-get update

# RUN apt-get install -y --no-install-recommends apt-utils
# RUN apt-get install -y chromium
# RUN apt-get install -y libcups2-dev 
# RUN apt-get install -y libavahi-compat-libdnssd-dev 
# RUN apt-get install -y gconf-service libasound2 libatk1.0-0 libcairo2 libcups2 libfontconfig1 libgdk-pixbuf2.0-0 libgtk-3-0 libnspr4 libpango-1.0-0 libxss1 fonts-liberation libappindicator1 libnss3 lsb-release xdg-utils

# # install chrome
# RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
# RUN dpkg -i google-chrome-stable_current_amd64.deb; apt-get -fy install
# RUN rm google-chrome-stable_current_amd64.deb



# Make port 3000 available to the world outside this container
# EXPOSE 3000

# CMD [ "npm", "run", "serve" ]