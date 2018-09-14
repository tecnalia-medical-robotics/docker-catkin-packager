#!/bin/bash

set -e # To fail on errors

export BASE_DISTRO_NAME="kinetic"

if [[ -z ${CUSTOM_DISTRO_NAME} ]]; then
  export CUSTOM_DISTRO_NAME="${BASE_DISTRO_NAME}-custom"
  echo "Custom distro name is not set, defaulting to ${CUSTOM_DISTRO_NAME}"
fi

if [[ -z ${PACKAGE_NAME} ]]; then
  export PACKAGE_NAME=${CUSTOM_DISTRO_NAME}
  echo "Package name is not set, defaulting to ${PACKAGE_NAME}"
fi

if [[ -z ${VERSION_NUMBER} ]]; then
  export VERSION_NUMBER="1.0.0"
  echo "Version number is not set, defaulting to 1.0.0"
fi

WORKSPACE="/root/catkin_ws"

if [[ -n ${SSH_PRIVATE_KEY} ]]; then
  eval $(ssh-agent -s)
  ssh-add <(echo "${SSH_PRIVATE_KEY}")
fi

if [[ -n ${SSH_SERVER_HOSTKEYS} ]]; then
  mkdir -p ~/.ssh
  echo "${SSH_SERVER_HOSTKEYS}" > ~/.ssh/known_hosts
fi

echo "Creating workspace"
cd ${WORKSPACE}
catkin config --no-extend -i /opt/ros/${CUSTOM_DISTRO_NAME} --install

echo "Installing project packages"
if [ -f "${WORKSPACE}/src/.rosinstall" ]; then
  cd ${WORKSPACE}/src
  wstool update
else
  cd ${WORKSPACE}/src
  wstool init
fi

echo "Installing depended ROS packages"
rosdep init
rosdep update
cd ${WORKSPACE}/src
rosinstall_generator --deps --tar --rosdistro ${BASE_DISTRO_NAME} $(comm -23 <(rosdep keys --from-paths . | sort) <(catkin list -w .. -u | sort)) | wstool merge -
wstool update

echo "Installing dependencies"
apt-get update
rosdep install -q -iy --from-paths ${WORKSPACE}/src

echo "Building"
catkin build --summarize --no-status

echo "Generating DEB"
mkdir /udevrules
export UDEVRULES=""
for package in $(catkin list -u -w ${WORKSPACE}); do
  export UDEVFILE="$(catkin locate -w ${WORKSPACE} ${package})/debian/udev"
  if [ -f ${UDEVFILE} ]; then
    cp ${UDEVFILE} /udevrules/99-${package}.rules
    export UDEVRULES="/udevrules/=/etc/udev/rules.d"
  fi
done
cd /debout
DEP_FLAGS="-d $(rosdep resolve $(comm -23 <(rosdep keys --from-paths ${WORKSPACE}/src | sort) <(catkin list -w ${WORKSPACE} -u | sort)) | grep -v '#' | sed '/^\s*$/d' | sed -E -e 's/[[:blank:]]+/\n/g' | sed ':a;N;$!ba;s/\n/ -d /g')"
fpm -s dir -t deb -n ${PACKAGE_NAME} -v ${VERSION_NUMBER} ${DEP_FLAGS} ${UDEVRULES} /opt/ros/${CUSTOM_DISTRO_NAME}
