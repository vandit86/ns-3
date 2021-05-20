#!/bin/sh

echo $1
TIME=60 

#./waf build

# start capture traffic on infaces
tshark -i 1 -a duration:$TIME -w left-0.pcap -q  &
tshark -i 6 -a duration:$TIME -w left-1.pcap -q  &
tshark -i 2 -a duration:$TIME -w right-0.pcap -q  &
tshark -i 7 -a duration:$TIME -w right-1.pcap -q  &

# start simulation 
./waf --run "mp-tap-wifi-lte --simTime=${TIME}"