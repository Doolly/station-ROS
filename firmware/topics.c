#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>

#include "topics.h"
#include "lift.h"

extern std_msgs::ByteMultiArray item_available_msg;
extern std_msgs::String lift_pos_state_msg;
extern std_msgs::Int32 lift_pos_msg;
extern std_msgs::String lift_item_state_msg;

extern const char *LiftPosState = { "parked", "moving" };
extern const char *LiftItemState = { "empty", "entering", "leaving", "entered" };


/* ========
 * Handlers
 * ======== */

void handleLiftGoal(const std_msgs::Int32& msg) {
  moveLiftTo(msg.data);
}

void handlePushItem(const std_msgs::Empty& msg) {
  pushItemToLift();
}

void handleSendToJames(const std_msgs::String& msg) {
  sendItemToJames(msg.data.c_str());
}

/* ========================
 * Publishers & Subscribers
 * ======================== */

extern ros::Publisher item_available("item_available", &item_available_msg);
extern ros::Subscriber<std_msgs::Int32> lift_goal("lift_goal", &handleLiftGoal);
extern ros::Publisher lift_pos_state("lift_pos_state", &lift_pos_state_msg);
extern ros::Publisher lift_pos("lift_pos_state", &lift_pos_msg);
extern ros::Publisher lift_item_state("lift_item_state", &item_state_msg);
extern ros::Subscriber<std_msgs::Empty> push_item("push_item", &handlePushItem);
extern ros::Subscriber<std_msgs::String> send_to_james("send_to_james", &handleSendToJames);

void initTopics(os::NodeHandle nh) {
  nh.subscribe(lift_goal);
  nh.advertise(lift_pos_state);
  nh.advertise(lift_pos);
  nh.advertise(lift_item_state);
  nh.advertise(item_available);
  nh.subscribe(push_item);
  nh.subscribe(send_to_james);
}
