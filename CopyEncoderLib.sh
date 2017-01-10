#!/bin/bash

echo ==========================================
echo Copy Core Encoder Shared Library to VCA
echo Example ./CopyEncoderLib.sh [Default=Copy/ Any Parameter=Remove]
echo ==========================================

if [ "$1" = "-h" ]
then
echo "Example ./CopyEncoderLib.sh [Default=Copy/ Any Parameter=Remove]"
exit 0
fi

if [ "$1" ];
then
	echo "** Output Remove : $1 **"
	for y in {0..3}
    do
		for x in {0..2}
		do
			VCADIR=/mnt/vca_node_${y}${x}/home/sgs
			echo "Starting Worker on ${VCADIR}"
			rm -rf $VCADIR/libTDllEncoder.so
			rm -rf $VCADIR/libTDllEncoder.so.0.1
			rm -rf $VCADIR/libTDllEncoderSlice.so
			rm -rf $VCADIR/libTDllEncoderSlice.so.0.1
			rm -rf $VCADIR/libThreadpool.so
			rm -rf $VCADIR/libThreadpool.so.0.1
			rm -rf $VCADIR/libThreadpoolStatic.a
		done
	done		
else
	echo "** Output Copy : $1 **"
	for y in {0..3}
    do
		for x in {0..2}
		do
			VCADIR=/mnt/vca_node_${y}${x}/home/sgs
			echo "Starting Worker on ${VCADIR}"
			cp -f libTDllEncoder.so $VCADIR/libTDllEncoder.so
			cp -f libTDllEncoder.so.0.1 $VCADIR/libTDllEncoder.so.0.1
			cp -f libTDllEncoderSlice.so $VCADIR/libTDllEncoderSlice.so
			cp -f libTDllEncoderSlice.so.0.1 $VCADIR/libTDllEncoderSlice.so.0.1
			cp -f libThreadpool.so $VCADIR/libThreadpool.so
			cp -f libThreadpool.so.0.1 $VCADIR/libThreadpool.so.0.1
			cp -f libThreadpoolStatic.a $VCADIR/libThreadpoolStatic.a
		done
	done		
fi

# libTDllEncoder.so
# libTDllEncoder.so.0.1
# libTDllEncoderSlice.so
# libTDllEncoderSlice.so.0.1
# libThreadpool.so
# libThreadpool.so.0.1
# libThreadpoolStatic.a

