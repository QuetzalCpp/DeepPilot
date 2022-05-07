cp -r ./gazebo_models/* ~/.gazebo/models/
rm -r gazebo_models
cp -r ./launch/* ~/bebop_ws/src/iROS_drone/rotors_simulator/rotors_gazebo/launch
rm -r launch
cp -r ./adr_worlds ~/bebop_ws/src/iROS_drone/rotors_simulator/rotors_gazebo/
rm -r adr_worlds

cp -r ./keyboard ~/bebop_ws/src/
rm -r keyboard

sudo add-apt-repository ppa:rock-core/qt4
sudo apt update
sudo apt install qt4-default

sudo apt install python3-pip
pip install -r requirements.txt
