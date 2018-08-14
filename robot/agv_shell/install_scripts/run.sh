#!/bin/bash
echo 'core-%t-%e' > /proc/sys/kernel/core_pattern
ulimit -c unlimited

#graceup
if [ -d /agvshell/standard/patch/ ]; then 
	cd /agvshell/standard/patch/
	if [ -f /agvshell/standard/patch/graceup.sh ]; then
		. /agvshell/standard/patch/graceup.sh
	fi
fi

#start agv_shell
cd /agvshell/
/bin/chmod +x /agvshell/agv_shell
/agvshell/agv_shell -s &
