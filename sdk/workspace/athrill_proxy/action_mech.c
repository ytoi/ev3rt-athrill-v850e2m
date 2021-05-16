#include "log.h"
#include "action_mech.h"

void ActionMech_InitializeRobotAction(RobotActionIndex *actionIndex, RobotFuncEntry *entries, int entryNum)
{
	RobotFuncEntry *p;

    LOG("InitAction num=%d",entryNum);
	for ( p = entries; p < entries+entryNum; p++) {
		RobotActionIndex *ai = actionIndex+(p->offset/4);
//        LOG("  ai=%p func=%p addkey=%d ",ai,p->func,p->additionalKey);
		ai->additionalKey = p->additionalKey;
		ai->func = p->func;
//        LOG("  CONFM:ai=%p func=%p addkey=%d ",ai,ai->func,ai->additionalKey);

	}

}

void ActionMech_ProcessData(const RobotActionIndex *actionIndex,unsigned int offset, unsigned int data)
{
	if ( offset % 4 != 0 ) {
		LOG("OFFSET value is invalid %d¥n", offset);
		return;
	}

	int index = offset / 4;
	// TODO: range check

	const RobotActionIndex *ai = actionIndex+index;
//    LOG("offset=%d ai=%p func=%p addkey=%d ",offset,ai,ai->func,ai->additionalKey);
	if ( ai->func ) {
		ai->func(data, ai->additionalKey);
	}
	return;

}

void ActionMech_VisitSensorData(const RobotSensorFuncEntry *entries, int entryNum, SensorVisitor visitor , void *userData)
{
    const RobotSensorFuncEntry *p;
	for ( p = entries; p < entries+entryNum; p++) {
        if ( p->func ) {
            int value;
            int ret = p->func(p->additionalKey,&value);
            // Call visitor to process derived data
            if ( ret == 1 && visitor ) {
                visitor(userData, p->offset, value);
            }
        }
    }
}