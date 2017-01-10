#!/bin/bash

echo "=========================================="
echo "  Store Encoder DLL into Specific Directory in VCA"  
echo "Example ./StoreEncoderVCA EncoderBackup/20161215_OK"
echo "=========================================="

for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/home/sgs
		cd $VCADIR

		echo "Store Encoder DLL into $VCADIR/$1"
		if [ "$1" ]; then
			# cp -f libTDll* $VCADIR/$1/
		fi 
	done
done
