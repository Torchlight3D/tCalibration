#!/bin/bash

chmod a+x MultiBridges
chmod a+x CalibrationTool
export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
PASSWORD=$(zenity --password)
echo $PASSWORD | sudo -S ./MultiBridges &
./CalibrationTool