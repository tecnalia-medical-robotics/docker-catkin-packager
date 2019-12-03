FROM ubuntu:xenial

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update -qq && apt-get install -qq -y \
    dirmngr \
    gnupg2 \
    lsb-release

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y apt-utils gnupg wget ca-certificates lsb-release

RUN sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list \
    && apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y \
        build-essential \
        python-catkin-tools \
        python-pip \
        python-rosdep \
        python-rosinstall-generator \
        python-wstool \
        ros-kinetic-catkin \
        ruby \
        ruby-dev \
        rubygems \
        ssh-client \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN gem install --no-ri --no-rdoc fpm

VOLUME /root/catkin_ws/src
VOLUME /debout

COPY package.bash /package.bash

ENTRYPOINT ["/package.bash"]
