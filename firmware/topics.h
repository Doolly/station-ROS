#ifndef __topics_h__
#define __topics_h__

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>

enum {
  LiftPos_Parked,
  LiftPos_Moving
}

enum {
  LiftItem_Empty,
  LiftItem_Entering,
  LiftItem_Leaving,
  LiftItem_Entered
}

void handleLiftGoal(const std_msgs::Int32& msg);
void handlePushItem(const std_msgs::Empty& msg);
void handleSendToJames(const std_msgs::String& msg);

void initTopics(os::NodeHandle nh);

#endif
