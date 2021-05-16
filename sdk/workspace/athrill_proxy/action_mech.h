#ifndef __ACTION_MECH_H
#define __ACTION_MECH_H

typedef void (*RobotFunc)(unsigned int data, unsigned int additionalKey);
// return 1 means OK, 0 means not used, -1 means error
typedef int (*SensorFunc)(unsigned int additionalKey, int *value);
typedef void (*SensorVisitor)(void *userData, unsigned int offset, int newValue);

typedef struct robot_func_entry {
	unsigned int offset;
	unsigned int additionalKey;
	RobotFunc func;
} RobotFuncEntry;

typedef struct robot_sensor_func_entry {
	unsigned int offset;
	unsigned int additionalKey;
	SensorFunc func;
} RobotSensorFuncEntry;


typedef struct robot_action_index {
	unsigned int additionalKey;
	RobotFunc func;
} RobotActionIndex;

extern void ActionMech_InitializeRobotAction(RobotActionIndex *actionIndex, RobotFuncEntry *entries, int entryNum);
extern void ActionMech_ProcessData(const RobotActionIndex *actionIndex,unsigned int offset, unsigned int data);
extern void ActionMech_VisitSensorData(const RobotSensorFuncEntry *entries, int entryNum, SensorVisitor visitor , void *userData);


#endif // !__ACTION_MECH_H