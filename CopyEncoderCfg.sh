#!/bin/bash

for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/home/sgs
		echo "Starting Worker on ${VCADIR}"
		
		TargetFile=${VCADIR}/ETRI_RA_main10.cfg
		echo "Copy to ${TargetFile}"
		cp -f ETRI_RA_main10.cfg ${TargetFile}

## The contents of Configure_ip.ini in eacg VCA_node are different to each other!!!		
		
	done
done
