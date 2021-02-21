#include <ros>

#include "topics.h"
#include "lift.h"

os::NodeHandle nh;

void setup()
{
  nh.initNode();
  initTopics(nh);

  initLift();
  initConveyor();
}

void loop()
{
  updateConveyorSensors();
  nh.spinOnce();
  delay(1);
}
