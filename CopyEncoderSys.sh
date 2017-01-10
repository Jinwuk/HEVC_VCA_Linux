#!/bin/bash

echo off
echo ==========================================
echo Copy Core Encoder Shared Library to VCA
echo Copy ETRI_RA_main10.cfg to VCA
echo Copy configure.ini to VCA
echo Example ./CopyEncoderSys.sh 
echo ==========================================

for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/home/sgs
		echo "Starting Worker on ${VCADIR}"

		cp -f ETRI_RA_main10.cfg ${VCADIR}/libTDllEncoder.so
		cp -f ETRI_RA_main10.cfg ${VCADIR}/libTDllEncoder.so.0.1		
		cp -f ETRI_RA_main10.cfg ${VCADIR}/libTDllEncoderSlice.so		
		cp -f ETRI_RA_main10.cfg ${VCADIR}/libTDllEncoderSlice.so.0.1		
		
		TargetFile=${VCADIR}/ETRI_RA_main10.cfg
		echo "Copy to ${TargetFile}"
		cp -f ETRI_RA_main10.cfg ${TargetFile}

		TargetFile=${VCADIR}/configure.ini
		echo "Copy to ${TargetFile}"
		cp -f configure.ini ${TargetFile}
		
## The contents of Configure_ip.ini in eacg VCA_node are different to each other!!!		

	done
done
