#!/bin/bash

for p in `ps aux | grep Hevc | grep -v grep | awk '{print $2}'`
    do 
		echo "HEVC Encoder Process : $p is going to be Killed"
		kill -9 $p
done
