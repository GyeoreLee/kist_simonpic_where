echo "set /psn_unit" 
rosparam set /psn_unit1 "[-0.92, 2.37, 0.0, 0.35, 0.0, 0.0]"
rosparam set /psn_unit2 "[0.81, 2.37, -0.23, 0.35, 6.29, 0.0]"
rosparam set /psn_unit3 "[ -2.41, 2.37, 2.47, 0.35, -0.34, 0.0]"
rosparam set /psn_unit4 "[2.52, 2.37, 3.42, 0.35, 2.14, 0.0]"

rosparam set /FP_filter False

echo "set /facesize"
rosparam set /facesize 90

echo "get /psn_unit" 
rosparam get /psn_unit1
rosparam get /psn_unit2
rosparam get /psn_unit3
rosparam get /psn_unit4

rosparam get /FP_filter

echo "### set rosparam of WHERE part ###"


rosparam set /verbose "False"
echo -n "/verbose "
rosparam get /verbose

rosparam set /szW 70
echo -n "/szW "
rosparam get /szW

rosparam set /szH 70
echo -n "/szH "
rosparam get /szH

rosparam set /szWH 1000
echo -n "/szWH "
rosparam get /szWH

rosparam set /szTOP 2.4
echo -n "/szTOP "
rosparam get /szTOP

rosparam set /scale 1.0
echo -n "/scale "
rosparam get /scale

rosparam set /display 3 
echo -n "/display "
rosparam get /display


