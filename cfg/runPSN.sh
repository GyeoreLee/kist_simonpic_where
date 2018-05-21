#!/bin/bash
kill -9  $(ps aux | grep 'roscore' | grep -v grep | awk '{print $2}')
sleep 3

echo "runs roscore"
roscore &

sleep 1
rosparam set /psn_unit2 "[ 1.5, 1.97, 0.0, 0.35, 0.0, 0.0]"
rosparam set /psn_unit1 "[-1.5, 1.97, 0.0, 0.35, 0.0, 0.0]"
rosparam set /psn_unit4 "[ 1.5, 1.97, 4.9, 0.35, 3.14, 0.0]"
rosparam set /psn_unit3 "[-1.5, 1.97, 4.9, 0.35, 3.14, 0.0]"
echo "set /psn_unit" 

#rosparam set /psn_unit2 "[ 1.5, 1.95, 0.0, 0.35, 0.0, 0.0]"
#rosparam set /psn_unit1 "[-1.5, 1.95, 0.0, 0.35, 0.0, 0.0]"
#rosparam set /psn_unit4 "[ 1.5, 1.95, 5.0, 0.35, 3.14, 0.0]"
#rosparam set /psn_unit3 "[-1.5, 1.95, 5.0, 0.35, 3.14, 0.0]"
#echo "set /psn_unit" 

#rosparam set /psn_unit2 "[ 1.5, 1.95, 0.0, 0.35, 0.0, 0.0]"
#rosparam set /psn_unit1 "[-1.5, 1.95, 0.0, 0.35, 0.0, 0.0]"
#rosparam set /psn_unit4 "[ 1.5, 1.95, 5.0, 0.35, 3.14, 0.0]"
#rosparam set /psn_unit3 "[-1.5, 1.95, 5.0, 0.35, 3.14, 0.0]"
#echo "set /psn_unit" 


#rosparam get /psn_unit2
#rosparam get /psn_unit1
#rosparam get /psn_unit4
#rosparam get /psn_unit3
