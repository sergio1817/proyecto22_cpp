#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./proyecto22_rt
else
	EXEC=./proyecto22_nrt
fi

$EXEC -n x8_0 -a 127.0.0.1 -p 9000 -l ./ -x setup_x8.xml -t x8_simu
