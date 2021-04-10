#include "lift.h"

/* ===============
 * Configurations
 * =============== */

// Setup parameters such as number of levels and pin numbers

const int num_levels = 3;
const int level_sensors[num_levels] = { 10, 11, 12 };
const int area_sensor = 13;
const int lift_motor_enable = 1;
const int lift_motor_dir = 2;
const int conveyor_motor_enable = 3;
const int conveyor_motor_dir = 4;

void initLift() {
  for (int i = 0; i < num_levels; i++) {
    pinMode(level_sensors[i], INPUT);
  }
  pinMode(area_sensor, INPUT);
  // TODO setup pins for the motor and the conveyor

  // move the lift to level 0
  rotateLiftMotor(LiftMotor_Down);
  while (!detectedAt(0)) { }
  rotateLiftMotor(LiftMotor_Stop);
  current_level = 0;

  lift_pos_state_msg.data = LiftPosState[LiftPos_Parked];
  lift_pos_state.publish(&lift_pos_state_msg);
  lift_pos_msg.data = current_level;
  lift_pos.publish(&lift_pos_msg);
  lift_item_staet_msg.data = LiftItem_Empty;
  lift_item_state.publish(&lift_pos_msg);
}

/* =======
 * Levels
 * ======= */

int current_level = 0;

bool detectedAt(int level) {
  return digitalRead(level_sensors[level]);
}

/* =============
 * Lift Control
 * ============= */

void rotateLiftMotor(LiftMotorDir dir) {
  if (dir == LiftMotor_Stop) {
    // TODO: Stop the motor
  } else if (dir == LiftMotor_Up) {
    // TODO: Turn the motor in upward direction
  } else if (dir == LiftMotor_Down) {
    // TODO: Turn the motor in downward direction
  }
}

void moveLiftTo(int target_level) {
  if (target_level >= num_levels || target_level < 0) {
    // TODO: Handle error
    return;
  }

  if (target_level == current_level) {
    // Do nothing
    return;
  }

  LiftMotorDir dir = current_level > target_level
                   ? LiftMotor_Down
                   : LiftMotor_Up;

  lift_pos_state_msg.data = LiftPosState[LiftPos_Moving];
  lift_pos_state.publish(&lift_pos_state_msg);

  rotateLiftMotor(dir);
  while (!detectedAt(target_level)) { }
  rotateLiftMotor(LiftMotor_Stop);

  current_level = target_level;

  lift_pos_state_msg.data = LiftPosState[LiftPos_Parked];
  lift_pos_state.publish(&lift_pos_state_msg);
  lift_pos_msg.data = current_level;
  lift_pos.publish(&lift_pos_msg);
}

/* =====================
 * Lift Item Management
 * ===================== */

const int item_entering_delay = 1000;

void rotateLiftConveyor(LiftConveyorDirection dir) {
  if (dir == LiftConveyor_Stop) {
    // TODO: Stop the conveyor
  } else if (dir == LiftConveyor_Left) {
    // TODO: Rotate the conveyor in the left direction
  } else if (dir == LiftConveyor_Right) {
    // TODO: Rotate the conveyor in the right direction
  }
}

bool isItemDetected() {
  return digitalRead(areaSensor);
}

void pushItemToLift() {
  if (isItemDetected()) {
    // TODO: Handle error
    return;
  }

  // Rotate the conveyors
  rotateConveyor(current_level, Conveyor_Run);
  rotateLiftConveyor(LiftConveyor_Left);

  while (!isItemDetected) { }

  lift_item_state_msg.data = LiftItemState[LiftItem_Entering];
  lift_item_state.publish(&lift_item_state_msg);

  // Keep rotating the conveyor until the item completely comes in
  delay(item_entering_delay);
  
  // Stop the conveyors
  rotateConveyor(current_level, Conveyor_Stop);
  rotateLiftConveyor(LiftConveyor_Stop);

  lift_item_state_msg.data = LiftItemState[LiftItem_Entered];
  lift_item_state.publish(&lift_item_state_msg);
}

void sendItemToJames(char *address) {
  if (current_level != 0) {
    // TODO: Handle error
    return;
  }

  // TODO: send the address to james

  lift_item_state_msg.data = LiftItemState[LiftItem_Leaving];
  lift_item_state.publish(&lift_item_state_msg);

  rotateLiftConveyor(LiftConveyor_Left);
  // TODO: rotate james conveyor
  
  rotateLiftConveyor(LiftConveyor_Stop);
  // TODO: stop james conveyor

  lift_item_state_msg.data = LiftItemState[LiftItem_Empty];
  lift_item_state.publish(&lift_item_state_msg);
}
