#!/bin/bash

./KillSubMaster.sh

echo "Temporal pausing for automatic stop in worker encoders"

sleep 1

echo "Wait 1 second"

sleep 1

echo "Wait 2 second"

sleep 1

echo "Wait 3 second"

./Killrun.sh

