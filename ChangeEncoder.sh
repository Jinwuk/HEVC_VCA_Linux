#!/bin/bash

echo off
echo ==========================================
echo Usage ./ChangeEncoder.sh [StoreDirectory][RestoreDirectory]
echo Example ./ChangeEncoder.sh ./R31rc ./R31rc/HDR
echo ==========================================

if [ "$1" = "-h" ]
then
	echo 
	echo "This Script is to move Encoder Libraries to the directory indicated as [StoreDirectory], "
	echo " and to move Encoder Libraries from [RestoreDirectory] to current Directory"
	echo " If Store Directory is not empty, then the Encoder Libraries are all replaced by those in [StoreDirectory]"
	echo "Example ./ChangeEncoder.sh EncoderBackup/20170201 EncoderBackup/HDR/20170201"
	echo
	exit 0
fi

echo "Store Encoder Libraries [$1]"
if [ -d "$1" ]
then
	echo "$1 is Directory and including Encoder Libraries : Remove"
	rm -rf libTDllEncoder.so
	rm -rf libTDllEncoder.so.0.1
	rm -rf libTDllEncoderSlice.so
	rm -rf libTDllEncoderSlice.so.0.1
	rm -rf libTDllEncoderStatic.a
	rm -rf libTDllEncoderSliceStatic.a
	
else
	echo "$1 is Directory and no Encoder Libraries : Copy"
	mv libTDllEncoder.so
	mv libTDllEncoder.so.0.1
	mv libTDllEncoderSlice.so
	mv libTDllEncoderSlice.so.0.1
	mv libTDllEncoderStatic.a
	mv libTDllEncoderSliceStatic.a
fi 

echo "Copy Restore Encoder Libraries [$2]"
cp -f $2/libTDllEncoder.so ./
cp -f $2/libTDllEncoder.so.0.1 ./
cp -f $2/libTDllEncoderSlice.so ./
cp -f $2/libTDllEncoderSlice.so.0.1 ./

## Remove previous Encoder Libraries
./CopyEncoderLib.sh 1 

## Copy new Encoder Libraries
./CopyEncoderLib.sh  


