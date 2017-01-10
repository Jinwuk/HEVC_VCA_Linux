#!/bin/bash

for p in `ps aux | grep Hevc | grep -v grep | awk '{print $2}'`
    do kill -9 $p
done

cd /home/sgs
echo -n "" > worker.log
./HevcEncWorker -c configure.ini > worker.log 2>&1 &

