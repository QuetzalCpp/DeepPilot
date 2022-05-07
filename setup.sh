cp -r ./gazebo_models/* ~/.gazebo/models/
rm -r gazebo_models
cp -r ./launch/* ~/bebop_ws/src/iROS_drone/rotors_simulator/rotors_gazebo/launch
rm -r launch
cp -r ./race_tracks/* ~/bebop_ws/src/iROS_drone/rotors_simulator/rotors_gazebo/
rm -r race_tracks

