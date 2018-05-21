echo "set /psn_unit" 
rosparam set /psn_unit2 "[ 1, 1.95, 0.0, 0.35, 0.0, 0.0]"
rosparam set /psn_unit1 "[-1.1, 1.95, 0.0, 0.35, 0.0, 0.0]"
rosparam set /psn_unit4 "[-1.2, 1.95, 3.4, 0.35, 3.92, 0.0]"
rosparam set /psn_unit3 "[-1.4, 1.95, 5.0, 0.35, 3.14, 0.0]"
rosparam set /FP_filter True

echo "set /facesize"
rosparam set /facesize 90

echo "get /psn_unit" 
rosparam get /psn_unit2
rosparam get /psn_unit1
rosparam get /psn_unit4
rosparam get /psn_unit3

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

sleep 2


row1=1.5
row2=2.6
row3=3.9
row4=5.0
row5=6.2
h=1.2

rosparam set /seat_location [[-1.8,$row1],[-1.2,$row1],[1.3,$row1],[2.0,$row1],[-2.3,$row2],[-1.5,$row2],[-0.8,$row2],[1.0,$row2],[1.7,$row2],[2.4,$row2],[-2.6,$row3],[-1.8,$row3],[-1.2,$row3],[0.1,$row3],[0.9,$row3],[1.6,$row3],[2.4,$row3],[-2.9,$row4],[-2.2,$row4],[-1.4,$row4],[-0.7,$row4],[0.7,$row4],[1.5,$row4],[2.1,$row4],[-2.2,$row5],[-1.6,$row5],[-0.8,$row5],[0.7,$row5],[1.3,$row5],[2.1,$row5]]

rosparam set /seat_height $h
