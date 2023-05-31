FROM ros:humble

RUN apt-get update
RUN apt-get install -y \
        python3-pip

RUN pip3 install \
        odrive \
        can \
        cantools

WORKDIR /starq_ws
