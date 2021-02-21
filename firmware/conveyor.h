#ifndef __conveyor_h__
#define __conveyor_h__

/* ===============
 * Configurations
 * =============== */

void initConveyor(void);

/* =====================
 * Conveyors and Sensors
 * ===================== */

typedef enum {
  Conveyor_Stop,
  Conveyor_Rotate,
} ConveyorDirection;

void updateConveyorSensors(void);
void rotateConvyeor(int id, ConveyorDirection direction);

#endif
