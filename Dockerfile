FROM ros:kinetic

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y apt-utils gnupg wget ca-certificates lsb-release

RUN sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list \
    && apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y \
        build-essential \
        python-catkin-tools \
        python-pip \
        python-rosdep \
        python-wstool \
        ros-$ROS_DISTRO-catkin \
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
