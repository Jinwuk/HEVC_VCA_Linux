#!/bin/bash

## /mnt/vca_node_00/home/sgs

SliceStatic=libTDllEncoderSliceStatic.a
FrameStatic=libTDllEncoderStatic.a
SliceDLL=libTDllEncoderSlice.so.0.1
SliceLnk=libTDllEncoderSlice.so
FrameDLL=libTDllEncoder.so.0.1
FrameLnk=libTDllEncoder.so

for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/home/sgs
		echo "Starting Worker on ${VCADIR}"
		pwd
		./subtest.sh ${VCADIR} $SliceStatic R30rc
		./subtest.sh ${VCADIR} $FrameStatic R30rc
		./subtest.sh ${VCADIR} $SliceDLL R30rc
		./subtest.sh ${VCADIR} $SliceLnk R30rc
		./subtest.sh ${VCADIR} $FrameDLL R30rc
		./subtest.sh ${VCADIR} $FrameLnk R30rc
		pwd
		
		TargetFile=${VCADIR}/ETRI_RA_main10.cfg
		echo "Copy to ${TargetFile}"
		cp -f ETRI_RA_main10.cfg ${TargetFile}
		
		TargetFile=${VCADIR}/configure.ini
		echo "Copy to ${TargetFile}"
		cp -f configure.ini ${TargetFile}

## The contents of Configure_ip.ini in eacg VCA_node are different to each other!!!		
		
	done
done
