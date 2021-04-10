#include "conveyor.c"

/* ===============
 * Configurations
 * =============== */

// num_conveyors >= num_levels(lift.c)
// ex)
// num_conveyors = 4 means that:
// 0~2 : level 0~2
// 3: spare conveyor for storing too big items
const int num_conveyors = 4;
const int area_sensors[num_conveyors][] = { 1, 2, 3, 4};
const int conveyor_motor_enable_pins[num_conveyors] = { 1, 2, 3, 4 };

void initConveyor(void) {
  for (int i = 0; i < num_conveyors; i++) {
    pinMode(area_sensors[i], INPUT);
    pinMode(area_sensors[i], INPUT);
    pinMode(conveyor_motor_enable_pins[i], OUTPUT);
  }
}

/* =====================
 * Conveyors and Sensors
 * ===================== */

void updateConveyorSensors(void) {
  for (int i = 0; i < num_conveyors; i++) {
    // TODO Update the topic /item_available
    // item_available_msg.data[i] = digitalRead(area_sensors[i]);
    // item_available.publish(&item_available_msg);
  }
}

void rotateConveyor(int level, ConveyorDirection direction) {
  if (id < 0 || level >= num_conveyors) {
    // TODO handle error
    return;
  }

  if (direction == Conveyor_Stop) {
    // TODO stop the conveyor[i]
  } else if (direction == Conveyor_Rotate) {
    // TODO rotate the conveyor[i]
  }
}
