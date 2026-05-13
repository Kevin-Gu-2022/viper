#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

# Confirm user in correct directory, otherwise docker run will not create volume correctly
read -p "Are you in same directory as script (y/n): " confirm && [[ $confirm == [yY] ]] || exit 1

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root."
  exit 1
fi

CAN=can0
CAN_BITRATE=1000000
GPIO_CAN0_STBY=160

echo "Configuring $CAN for a bitrate of $CAN_BITRATE bits/s"

function finish
{
  ip link set $CAN down
  echo 1 > /sys/class/gpio/gpio$GPIO_CAN0_STBY/value
  echo $GPIO_CAN0_STBY > /sys/class/gpio/unexport
}
trap finish EXIT

echo $GPIO_CAN0_STBY > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$GPIO_CAN0_STBY/direction
echo 0 > /sys/class/gpio/gpio$GPIO_CAN0_STBY/value

ip link set $CAN type can bitrate $CAN_BITRATE
ip link set $CAN txqueuelen 256
ip link set $CAN up

sudo -u fio ifconfig $CAN

# Name for container instance
CONTAINER_NAME="viper_dev_container"

# Check if the container exists (even if stopped)
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME found."
    
    # Check if it's already running
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "Container is already running. Entering..."
    else
        echo "Starting stopped container..."
        docker start $CONTAINER_NAME
    fi
    
    # Enter the container
    docker exec -it $CONTAINER_NAME bash
else
    echo "Creating NEW container instance..."

    # Don't need volume. Just push to the repo
    #    -v "$(pwd)/../../..:/workspace/src" \
    docker run -it \
       --name $CONTAINER_NAME \
       --privileged \
       --network host \
       viper_dev_image /ros_entrypoint.sh bash
fi