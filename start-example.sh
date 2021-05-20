#!/bin/sh


if [ -z "$1" ]; 
    then echo "run with default time : 60 sec ";
    TIME=60; 

else echo "rum simulation for '$1' sec";
    TIME=$1
fi


if [ -z "$2" ]; 
    then echo "run with default path2delay : 1 msec ";
    P2delay=1; 

else echo "delay for second path is '$2' milisec";
    P2delay=$2
fi

#./waf build

# get interfaces list 
i_list=`tshark -D`

#search for interface numbers 
i_left=`echo "$i_list" | grep 'tap-left' -m 1 | cut -d "." -f1`
i_left_1=`echo "$i_list" | grep 'tap-left-1' -m 1 | cut -d "." -f1`
i_right=`echo "$i_list" | grep 'tap-right' -m 1 | cut -d "." -f1`
i_right_1=`echo "$i_list" | grep 'tap-right-1' -m 1 | cut -d "." -f1`

# just to check 
echo "tap-left is on $i_left"
echo "tap-left-1 is on $i_left_1"
echo "tap-right is on $i_right"
echo "tap-right-1 is on $i_right_1"

# start capture traffic on infaces
tshark -i $i_left -a duration:$TIME -w left-0.pcap -q  &
tshark -i $i_left_1 -a duration:$TIME -w left-1.pcap -q  &
tshark -i $i_right -a duration:$TIME -w right-0.pcap -q  &
tshark -i $i_right_1 -a duration:$TIME -w right-1.pcap -q  &

# start simulation 
./waf --run "mp-tap-wifi-lte --simTime=${TIME} --path2delay=${P2delay}"

sleep 1

# merging .pcap files 
mergecap -w left.pcap left-0.pcap left-1.pcap
mergecap -w right.pcap right-0.pcap right-1.pcap
