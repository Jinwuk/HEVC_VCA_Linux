#!/bin/bash

# ============================================
# Working Directory : $1
# FileName          : $2
# Source Directory  : $3
# Example : ./subtest /mnt/vca_node_${y}${x}/home/sgs libTDllEncoder.so.0.1 R30rc
# ===========================================

cd $1
pwd
echo "Remove File :"$2
if [ -f $2 ]
then
	echo "File Remove"
	rm -f $2
else
echo $2" is Removed"
fi

cp /home/etri/workspace/CopyFolder/$3/$2 $2

