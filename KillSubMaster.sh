#!/bin/bash

for p in `ps aux | grep Hevc | grep -v grep | awk '{print $2}'`
    do
    echo "Stop Sub-Master Process: ${p}"
    kill -9 $p
done



