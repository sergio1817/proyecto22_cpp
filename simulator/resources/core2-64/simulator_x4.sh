#! /bin/bash
if [ -f /proc/xenomai/version ];then
	EXEC=./proyecto22_simulator_rt
else
	EXEC=./proyecto22_simulator_nrt
fi

$EXEC -n x4_0 -t x4 -p 9000 -a 127.0.0.1 -x simulator_x4.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml
