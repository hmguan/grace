#!/bin/bash

##iptables need to ping, change it before use "ping_ips.sh"!
ips=("10.1.0.254")

sleep_sec=2


mkdir -p /gzrobot/log/
file_date=`date "+%Y%m%d_%H%M%S"`
while [ true ]
do
	date_today=`date "+%Y%m%d"`
	if [ "${file_date:0:8}" != "$date_today" ]; then
		file_date=`date "+%Y%m%d_%H%M%S"`
	fi
	for s in ${ips[@]}; do
		date >> /gzrobot/log/"ping_"$file_date".log"
		ping -c 1 -w 1 $s >> /gzrobot/log/"ping_"$file_date".log"
	done;
	sleep $sleep_sec;
done

