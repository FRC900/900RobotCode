# Set up a Ubuntu laptop to forward internet conntection to the internal
# robot network.
# For this to work -
#  1. easiest to set the laptop ethernet connection to replace the radio.
#    - unplug the radio on the robot
#    - Set up a network connection on the laptop to mimic the router
#      - click on network icon, edit connections..., add new connection
#      - pick ethernet, click create...
#      - Name it Robot Router or something similar to distinguish it from a normal wired connection to the robot
#      - Under the ethernet tab, select the ethernet device to use
#      - Under general, deselect "Automatically connect to this network..."
#      - IPv4 settings, switch to manual, add an address of 10.9.0.1 with a netmask of 24
#      - Set dns servers to "8.8.8.8, 8.8.4.4" (not sure this matters)

# Wire in to the robot network
# Select the new Robot Router network connection
#  - grab the name of the adapter, update PRIVATE_HW
# Connect to the internet via wifi
# Grap the ip address of the internet-connected wifi adapter plus its name, update INTERNET_IP and INTERNET_HW

# run this script

# If the Jetson and/or Rio can't get on the net after that, a few things to check
# Gateway needs to be set to 10.9.0.1 on them
# DNS server should be set to "8.8.8.8,8.8.8.4" 

# Use the robot_ipmasq_off script to disable sharing

#-------------------------------------------------------
# ip address and hardware connnected on the internet side
# TODO - try to grab one of these pieces of info from the other
INTERNET_IP=192.168.0.168
INTERNET_HW=wlp3s0

# IP address and hardware connected to the private Rio network side
PRIVATE_IP=10.9.0.1
PRIVATE_HW=enp2s0

sudo sysctl net.ipv4.ip_forward=1

sudo iptables -A FORWARD -o $INTERNET_HW -i $PRIVATE_HW -s $PRIVATE_IP/24 -m conntrack --ctstate NEW -j ACCEPT
sudo iptables -A FORWARD -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
sudo iptables -t nat -F POSTROUTING
sudo iptables -t nat -A POSTROUTING -o $INTERNET_HW -j MASQUERADE
