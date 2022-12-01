#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./proyecto22_rt
else
	EXEC=./proyecto22_nrt
fi

$EXEC -n x4_0 -a 172.26.213.62 -p 9000 -l /tmp -x setup_x4_intelaero.xml -t aero
