#ifndef __lift_h__
#define __lift_h__

/* ===============
 * Configurations
 * =============== */

void initLift();

/* =======
 * Levels
 * ======= */

bool detectedAt(int level);

/* =============
 * Lift Control
 * ============= */

typedef enum {
  LiftMotor_Stop,
  LiftMotor_Up,
  LiftMotor_Down
} LiftMotorDir;

void rotateLiftMotor(LiftMotorDir dir);
void moveLiftTo(int target_level);

/* =====================
 * Lift Item Management
 * ===================== */

typedef enum {
  LiftConveyor_Stop,
  LiftConveyor_Left,
  LiftConveyor_Right
} LiftConveyorDirection;

void rotateLiftConveyor(LiftConveyorDirection dir);
bool isItemDetected();
void pushItemToLift();
void sendItemToJames(char *address);

#endif
