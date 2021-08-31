#!/bin/sh
ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1
ip route add 11.0.0.0/8 via 13.0.0.1 dev eth0 

# disable checksum 
ethtool --offload  eth0  rx off  tx off
ethtool --offload  eth1  rx off  tx off

# do not need line below , just to ping test
ip route add 16.0.0.0/8 via 14.0.0.1 dev eth1

# tc add delay to interface
tc qdisc add dev eth0 root netem delay 55ms
tc qdisc add dev eth1 root netem delay 20ms

