#!/bin/bash

echo "=========================================="
echo "./MakeDirVCA [Directory Name or -h]"
echo "Example ./MakeDirVCA Backup"
echo "=========================================="

if [ "$1" = "-h" ]
then
	echo 
	echo "This Script is to make a directory under /mnt/vca_node_xx/home/sgs "
	echo "Example ./MakeDirVCA Backup"
	echo "then the new directory is made such as /mnt/vca_node_xx/home/sgs/Backup"
	echo
	exit 0
fi


for y in {0..3}
    do
    for x in {0..2}
		do
		VCADIR=/mnt/vca_node_${y}${x}/home/sgs
		cd $VCADIR

		if [ "$1" ]; then
			echo "Make Directory $VCADIR/$1"
			mkdir $1
		fi	
	done
done
