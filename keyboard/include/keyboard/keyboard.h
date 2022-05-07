#ifndef KEYBOARD_H 
#define KEYBOARD_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Int8.h>
#include "std_msgs/Empty.h"

#include <QWidget>
#include <QtGui>

#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QFont>
#include <QKeyEvent>

using namespace std;

class KeyPress: public QWidget 
{

  public:
    KeyPress(QWidget *parent = 0);//constructor
    ~KeyPress();//destructor
	
  protected:
    void keyPressEvent(QKeyEvent * e);
  
  private:
  
  	ros::NodeHandle nh_;
  	
  	ros::Publisher OvR;
	ros::Publisher pubLand1_;
	ros::Publisher pubTakeoff1_;
	ros::Publisher pubCommandPilot1_;
	ros::Publisher pubCommandCamera1_;
	
	geometry_msgs::Twist commandPilot;
	std_msgs::Empty msgLand;
	std_msgs::Empty msgTakeoff;
	std_msgs::Int8 override;
	
	geometry_msgs::Twist commandCam;
	
	QGridLayout *grid;
	QLabel *Speed;
	QLabel *Alt_speed;
	
	QLabel *Speed_value;
	QLabel *Alt_speed_value;
	QLabel *Fovea_value_y;
	QLabel *Command;

	QFont font;
	
	float speed;
	float altitude_speed;
	
	float pitch;
	float roll;
	float yaw;
	float altitude;

};

#endif
