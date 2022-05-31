#!/usr/bin/env bash
#
#   Setup Pi
#
#   This updates the Pi and installs a bunch of useful items.
#

######################################################################
# Start with the basic updates.
######################################################################

# General Update and upgrade
sudo apt update
sudo apt -y full-upgrade


######################################################################
# Then install lots of support stuff.
######################################################################

# Basic operations:  ssh, vnc
# Already on Pi
#sudo apt install -y openssh-server tigervnc-standalone-server

# Editors
sudo apt install -y emacs vim geany bluefish

# Support functions:  net-tools, curl, git, etc.
sudo apt install -y net-tools curl git dos2unix tmux cmake

# Image display/processing/capture.
sudo apt install -y cheese gimp peek

# Not available on Pi - it has Chromium
# sudo apt install -y firefox

# Programming: C/C++
sudo apt install -y gcc g++

# Programming Python3.
sudo apt install -y python3 python3-pip python3-numpy python3-scipy
sudo apt install -y python3-wheel python3-opencv

# To support the Realsense:
sudo apt install -y automake libtool vim cmake libusb-1.0-0-dev
sudo apt install -y libx11-dev xorg-dev libglu1-mesa-dev

# To recognize the Realsense USB device.
sudo apt install -y curl
if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    sudo curl -s -o /etc/udev/rules.d/99-realsense-libusb.rules \
	 https://raw.githubusercontent.com/\
IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
fi

# To support the sparkfun QWICC interface
sudo pip3 install sparkfun-qwiic-scmd

# General I2C interface
sudo apt install -y i2c-tools python3-smbus

# To support the pigpio (already installed?), auto-start deamon.
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable pigpiod


######################################################################
# Finally, configure/set up the user preferences.
######################################################################

# Create the extra bash configuration:
cat <<EOF > ~/.bash_aliases
#
# Extra Shell Initialization
#

# Ignore Ctrl-D.  Otherwise Ctrl-D would log you out. 
set -o ignoreeof

# Personal/safety stuff.  Ask before removing, etc.
alias mv='mv -i'
alias rm='rm -i'
alias dir='ls -alv'

# Allow VNC to accept unencrypted connections.
# alias vncserver='vncserver -localhost no'
alias vncserver='echo "The Pi does this automatically"'

# On Pi, check the temperature.
alias pitemp='/opt/vc/bin/vcgencmd measure_temp'

# Check whether we are using ROS.
if [ -f ~/.bash_ros ]; then
    source ~/.bash_ros
else
    echo "Welcome - ready to drive a robot!"
fi

EOF
