#!/bin/sh
# Start LXC containers 

# add namespaces 
ip netns add left
ip netns add right

# create links left
ip link add eth0 type veth peer name vethLeft
ip link add eth1 type veth peer name vethLeft1

ip link set eth0 netns left
ip link set eth1 netns left

# create links right
ip link add eth0 type veth peer name vethRight
ip link add eth1 type veth peer name vethRight1

ip link set eth0 netns right
ip link set eth1 netns right 

# set UP in promisc
ip link set dev vethLeft up
ip link set dev vethLeft1 up
ip link set dev vethRight up
ip link set dev vethRight1 up

ip link set vethLeft promisc on
ip link set vethLeft1 promisc on
ip link set vethRight promisc on
ip link set vethRight1 promisc on

#show interfaces 
ip link

#### config left ####
ip netns exec left ip addr add 11.0.0.2/8 dev eth0
ip netns exec left ip addr add 15.0.0.2/8 dev eth1

ip netns exec left ip link set dev lo up
ip netns exec left ip link set dev eth0 up
ip netns exec left ip link set dev eth1 up

ip netns exec left ip route add 14.0.0.0/8 via 15.0.0.1 dev eth1
ip netns exec left ip route add 13.0.0.0/8 via 11.0.0.1 dev eth0
# ip route add default via 10.0.1.1 dev H1-eth0

# disable checksum tcp
ip netns exec left ethtool --offload  eth0  rx off  tx off
ip netns exec left ethtool --offload  eth1  rx off  tx off

ip netns exec left ip r

#### config right ####
ip netns exec right ip addr add 13.0.0.2/8 dev eth0
ip netns exec right ip addr add 14.0.0.2/8 dev eth1

ip netns exec right ip link set dev lo up
ip netns exec right ip link set dev eth0 up
ip netns exec right ip link set dev eth1 up

ip netns exec right ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1
ip netns exec right ip route add 11.0.0.0/8 via 13.0.0.1 dev eth0

# disable checksum tcp
ip netns exec right ethtool --offload  eth0  rx off  tx off
ip netns exec right ethtool --offload  eth1  rx off  tx off

# tc add delay to interface
ip netns exec right tc qdisc add dev eth0 root netem delay 55ms
ip netns exec right tc qdisc add dev eth1 root netem delay 20ms

ip netns exec right ip r

if [  -z "$1" ]
  then
    echo "argument not  supplied"
    #### configure mptcp path manager by iproute2  ###

    # Set the per connection and IP address limits to 1 on the server
    ip netns exec right ip mptcp limits set subflow 1

    #Add IP address as a new MPTCP endpoint on the server
    ip netns exec right ip mptcp endpoint add 13.0.0.2 dev eth0 signal
    ip netns exec right ip mptcp endpoint add 14.0.0.2 dev eth1 signal


    # Set the per connection and IP address limits to 1 on the client
    ip netns exec left ip mptcp limits set subflow 1 add_addr_accepted 1
fi

