#!/bin/bash

echo "==============================="
echo "Set HDR Encoder Libraries to VCA "
echo "Usage : SetHDR [SetSDR: 0/ SetHDR 1 / Help -h][EncoderVersion]"
echo "==============================="
echo

if [ "$1" = "-h" ]
then
	echo 
	echo "This Script is to Set Encoder Libraries as SDR or HDR Encoder "
	echo "Usage : SetHDR [SetSDR: 0/ SetHDR 1 / Help -h][EncoderVersion]"
	echo "Example ./SetHDR.sh 1 R31rc : To Set HDR Encoder with R31rc (Directory Name)"
	echo
	exit 0
fi

if [ "$1" -eq 1 ]
then
	echo "Set HDR Encoder from $2"
	./ChangeEncoder.sh ./R31rc ./R31rc/HDR
else
	echo "Set SDR Encoder from $2"
	./ChangeEncoder.sh ./R31rc/HDR ./R31rc
fi
