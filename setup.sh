cp -r ./gazebo_models/* ~/.gazebo/models/
rm -r gazebo_models
cp -r ./launch/* ~/bebop_ws/src/iROS_drone/rotors_simulator/rotors_gazebo/launch
rm -r launch
cp -r ./adr_worlds ~/bebop_ws/src/iROS_drone/rotors_simulator/rotors_gazebo/
rm -r adr_worlds

sudo apt install python3-pip
pip install -r requirements.txt
