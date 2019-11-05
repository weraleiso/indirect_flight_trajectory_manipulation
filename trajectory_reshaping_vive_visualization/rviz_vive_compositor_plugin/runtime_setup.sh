#!/bin/bash

export openvr=~/catkin_ws/src/trajectory_reshaping/trajectory_reshaping_tools/htc_vive/newer_openvr
export steamvr=~/.steam/steam/steamapps/common/SteamVR
export steamruntimesmall=~/catkin_ws/src/trajectory_reshaping/trajectory_reshaping_tools/htc_vive/steam_runtime

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/usr/lib/:\
/usr/lib32/:\
$openvr/lib/linux32/:\
$openvr/lib/linux64/:\
$steamruntimesmall/:\
$steamvr/bin/linux32/:\
$steamvr/bin/linux64/:\
$steamvr/drivers/lighthouse/bin/linux32/:\
$steamvr/drivers/lighthouse/bin/linux64/:\

