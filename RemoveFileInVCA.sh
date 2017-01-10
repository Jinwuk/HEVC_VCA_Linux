#!/bin/bash

echo off
echo ==========================================
echo Usage ./RemoveFileInVCA.sh [File] [Directory]
echo Example RemoveFileInVCA.sh ETRI_RA_main10.cfg home
echo ==========================================

for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/home/sgs
		echo "Starting Worker on ${VCADIR}"
		
		TargetFile=${VCADIR}/$1
		echo "Remove in ${TargetFile}"
		rm -f $1 ${TargetFile}

## The contents of Configure_ip.ini in eacg VCA_node are different to each other!!!		
		
	done
done
