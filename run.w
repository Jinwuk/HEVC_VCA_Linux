#!/bin/bash

for p in `ps aux | grep Hevc | grep -v grep | awk '{print $2}'`
    do kill -9 $p
done

if [ "$LD_LIBRARY_PATH" != "/home/sgs" ]  
then 
	export LD_LIBRARY_PATH=/home/sgs
fi

cd /home/sgs
echo -n "" > worker.log
./HevcEncWorker -c configure.ini > worker.log 2>&1 &

