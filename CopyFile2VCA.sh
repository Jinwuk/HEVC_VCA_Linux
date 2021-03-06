#!/bin/bash

echo off
echo ==========================================
echo Usage ./CopyFile2VCA [File] [Directory]
echo Example CopyFile2VCA ETRI_RA_main10.cfg home/sgs
echo ==========================================

if [ "$1" = "-h" ]
then
	echo 
	echo "This Script is to copy a file to the directory indicated as [Directory] "
	echo " As a result, the file is copied to the directory /mnt/vca_node_xx/[Directory]"
	echo "Example CopyFile2VCA ETRI_RA_main10.cfg home/sgs"
	echo
	exit 0
fi


for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/$2
		echo "Starting Worker on ${VCADIR}"
		
		TargetFile=${VCADIR}/$1
		echo "Copy to ${TargetFile}"
		cp -f $1 ${TargetFile}

## The contents of Configure_ip.ini in eacg VCA_node are different to each other!!!		
		
	done
done
