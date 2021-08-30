#!/bin/sh
# Start LXC containers 
lxc-start -n left
lxc-start -n right

#show container status 
lxc-ls -f

ip link set vethLeft promisc on
ip link set vethLeft1 promisc on
ip link set vethRight promisc on
ip link set vethRight1 promisc on

#show interfaces 
ip a