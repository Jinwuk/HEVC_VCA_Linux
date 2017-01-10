#!/bin/bash

for v in {1..12}
    do
    echo "Stop Worker on 172.31.${v}.1"
    ssh root@172.31.${v}.1 "./KillWorkerEncoder.sh"
done
