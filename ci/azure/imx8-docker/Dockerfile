FROM arm64v8/ubuntu:20.04

ENV TZ=Asia/Dubai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update
RUN apt install -y sudo

RUN sudo apt install -y build-essential cmake python2-dev python3-dev libssl-dev git libopencv-contrib-dev libopencv-dev libgl1-mesa-dev libglfw3-dev 

RUN chmod +x ./setup_docker.sh
RUN ./setup_docker.sh
