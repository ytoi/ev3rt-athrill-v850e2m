#include "app.h"
#include <stdio.h>
#include <stdint.h>
#include "ev3api.h"
#include "action_mech.h"
#include "log.h"
#include "msgid.h"
#define ATHRILL_DEVICE_MEM_SIZE (1024)
#define MAX_MESSAGE_PAYLOAD (ATHRILL_DEVICE_MEM_SIZE)



// Robot Controller 
static unsigned char athrillDeviceMem[ATHRILL_DEVICE_MEM_SIZE];
static unsigned char ev3DeviceMem[ATHRILL_DEVICE_MEM_SIZE];


static void robotSetLEDColor(unsigned int data, unsigned int additionalKey)
{
#if 0
	ledcolor_t color = LED_OFF;
	if ( data & 0x1 ) {
		color = LED_RED;
	} else if ( data & 0x2 ) {
		color = LED_GREEN;
	} else if ( data & 0x4 ) {
		color = LED_YELLOW;
	} else if ( data & 0x8 ) {
		color = LED_BLUE;
	}
#endif
	ev3_led_set_color(data);

}

static void robotMotorSetPower(unsigned int data, unsigned int additionalKey)
{
//	LOG("MotorSetPower port=%d pow=%d\n",additionalKey,data);
	ev3_motor_set_power((motor_port_t)additionalKey, data);

}

static void robotMotorStop(unsigned int data, unsigned int additionalKey)
{
	ev3_motor_stop((motor_port_t)additionalKey, data);

}

static void robotMotorResetCounts(unsigned int data, unsigned int additionalKey)
{
	if ( data == 1 ) {
		ev3_motor_reset_counts(additionalKey);
	}
}

static void robotGyroSensorReset(unsigned int data, unsigned int additionalKey)
{
	static sensor_port_t gyroPort = -1;

	if ( gyroPort == -1 ) {
		sensor_port_t port;
		for ( port = EV3_PORT_1; port < TNUM_SENSOR_PORT; port++ ) {
			if ( ev3_sensor_get_type(port) == GYRO_SENSOR ) {
				gyroPort = port;
				break;
			}
		}
		if ( port == TNUM_SENSOR_PORT ) {
			// There is no gyro sensor;
			gyroPort = TNUM_SENSOR_PORT;
		}
	}
	if ( gyroPort != TNUM_SENSOR_PORT ) {
		ev3_gyro_sensor_reset(gyroPort);
	}

}



// 

static RobotFuncEntry robotFuncEntryTable[] = {
	{ 32, 0, robotSetLEDColor },
	{ 36, EV3_PORT_A, robotMotorSetPower },
	{ 40, EV3_PORT_B, robotMotorSetPower },
	{ 44, EV3_PORT_C, robotMotorSetPower },
	{ 48, EV3_PORT_D, robotMotorSetPower },
	{ 52, EV3_PORT_A, robotMotorStop },
	{ 56, EV3_PORT_B, robotMotorStop },
	{ 60, EV3_PORT_C, robotMotorStop },
	{ 64, EV3_PORT_D, robotMotorStop },
	{ 68, EV3_PORT_A, robotMotorResetCounts },
	{ 72, EV3_PORT_B, robotMotorResetCounts },
	{ 76, EV3_PORT_C, robotMotorResetCounts },
	{ 80, EV3_PORT_D, robotMotorResetCounts },
	{ 84, 0, robotGyroSensorReset }
};

// index is offset / 4
static RobotActionIndex robotActionIndex[ATHRILL_DEVICE_MEM_SIZE/4];

#if 0
static void initializeRobotAction(void)
{
	RobotFuncEntry *p;

	for ( p = robotFuncEntryTable; p < robotFuncEntryTable+NUMOF(robotFuncEntryTable); p++) {
		RobotActionIndex *ai = robotActionIndex+(p->offset/4);
		ai->additionalKey = p->additionalKey;
		ai->func = p->func;
	}
}

static void processData(unsigned int offset, unsigned int data)
{
	if ( offset % 4 != 0 ) {
		LOG("OFFSET value is invalid %d¥n", offset);
		return;
	}

	int index = offset / 4;
	// TODO: range check

	RobotActionIndex *ai = robotActionIndex+index;
	if ( ai->func ) {
		ai->func(data, ai->additionalKey);
	}
	return;

}

#endif

// Senser 
// Athrillから来る128byte目をセンサーのTypeとして使用する
// これはev3rt-athrillのDRI_COL_XXX (target/v850_gcc/uart/src/uart_dri.c)にあわせる
#define COLOR_SENSOR_MODE_INDEX (128)
#define COLOR_SENSOR_MODE_REFLECT (0)
#define COLOR_SENSOR_MODE_AMBIENT (1)
#define COLOR_SENSOR_MODE_COLOR (2)
#define COLOR_SENSOR_MODE_RGB (3)

static int sensorButtonIsPressed(unsigned int additionalKey,  int *value)
{
	unsigned int ret = 0;
	typedef struct {
		button_t button;
		int shift;
	} ButtonElement;

	const ButtonElement buttons[] = {
		{ LEFT_BUTTON, 0},
		{ RIGHT_BUTTON, 1},
		{ UP_BUTTON, 2},
		{ DOWN_BUTTON, 3},
		{ ENTER_BUTTON, 4},
		{ BACK_BUTTON, 5},
	};

	const ButtonElement *p;

	for ( p = buttons; p < buttons+NUMOF(buttons); p++ ) {
		if ( ev3_button_is_pressed(p->button) ) {
			ret |= (1 << p->shift);
		}
	}
	*value = ret;
	return 1;

}

static int getColorSensorType(void)
{
	unsigned int color_sensor_type = *(unsigned int*)(athrillDeviceMem + COLOR_SENSOR_MODE_INDEX);

	return color_sensor_type;
}

static sensor_port_t getColorSensorPort(void)
{
	static sensor_port_t color_sensor_port = TNUM_SENSOR_PORT; // Not used value

	if ( color_sensor_port == TNUM_SENSOR_PORT ) {
		sensor_port_t port;
		for ( port = EV3_PORT_1; port <= EV3_PORT_4; port++ ) {
			if ( ev3_sensor_get_type(port) == COLOR_SENSOR ) {
				color_sensor_port = port;
				break;
			}
		}
	}
	return color_sensor_port;
}

static int sensorColorSensorGetAmbient(unsigned int additionalKey,  int *value) 
{
	int col_type = getColorSensorType();
	if ( col_type !=  COLOR_SENSOR_MODE_AMBIENT ) {
		return 0;
	}
	sensor_port_t port = getColorSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		*value = ev3_color_sensor_get_ambient(port);
		return 1;
	}
	return 0; // Not Used
}

static int sensorColorSensorGetColor(unsigned int additionalKey,  int *value) 
{
	int col_type = getColorSensorType();
	if ( col_type !=  COLOR_SENSOR_MODE_COLOR ) {
		return 0;
	}
	sensor_port_t port = getColorSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		*value = ev3_color_sensor_get_color(port);
		return 1;
	}
	return 0;
}

static int sensorColorSensorGetReflect(unsigned int additionalKey, int *value) 
{
	int col_type = getColorSensorType();
	if ( col_type !=  COLOR_SENSOR_MODE_REFLECT ) {
		return 0;
	}
	sensor_port_t port = getColorSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		*value =  ev3_color_sensor_get_reflect(port);
		return 1;
	}
	return 0;
}

static int sensorColorSensorGetRgbRaw(unsigned int additionalKey, int *value) 
{
	// RGB Raw corrects value when R is requested( it means additiona key is 0)
	// Otherwise returns saved value
	static rgb_raw_t rgb;
	int col_type = getColorSensorType();
	if ( col_type !=  COLOR_SENSOR_MODE_RGB ) {
		return SENSOR_NOT_USED;
	}
	sensor_port_t port = getColorSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		if ( additionalKey == 0 ) {
			 ev3_color_sensor_get_rgb_raw(port,&rgb);
		}
		switch(additionalKey){
			case 0:
				*value = rgb.r;
				return 1;
			case 1:
				*value = rgb.g;
				return 1;
			case 2:
				*value = rgb.b;
				return 1;
			default:
				// nothing to do
				break;
		}
	}
	return 0;
}

// Gyro
static sensor_port_t getGyroSensorPort(void)
{
	static sensor_port_t gyro_sensor_port = TNUM_SENSOR_PORT; // Not used value

	if ( gyro_sensor_port == TNUM_SENSOR_PORT ) {
		sensor_port_t port;
		for ( port = EV3_PORT_1; port <= EV3_PORT_4; port++ ) {
			if ( ev3_sensor_get_type(port) == GYRO_SENSOR ) {
				gyro_sensor_port = port;
				break;
			}
		}
	}
	return gyro_sensor_port;
}

static int sensorGyroSensorGet(unsigned int additionalKey, int *value) 
{
	sensor_port_t port = getGyroSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		if (additionalKey == 0 ) {
			*value = ev3_gyro_sensor_get_angle(port);
		} else {
			*value = ev3_gyro_sensor_get_rate(port);
		}
		return 1;
	}
	return 0;
}

// UltraSonic
static sensor_port_t getUltraSonicSensorPort(void)
{
	static sensor_port_t ultra_sonic_sensor_port = TNUM_SENSOR_PORT; // Not used value

	if ( ultra_sonic_sensor_port == TNUM_SENSOR_PORT ) {
		sensor_port_t port;
		for ( port = EV3_PORT_1; port <= EV3_PORT_4; port++ ) {
			if ( ev3_sensor_get_type(port) == ULTRASONIC_SENSOR ) {
				ultra_sonic_sensor_port = port;
				break;
			}
		}
	}
	return ultra_sonic_sensor_port;
}

static int sensorUltrasonicSensorGet(unsigned int additionalKey, int *value) 
{
	// 超音波センサーは取得周期をはやめすぎると精度が悪くなるので、前回から一定の時間経っていない場合は取得しない
	// 40msec
	#define ULTRASONIC_CORRECT_THREATH_TIME (40*1000)
	static SYSTIM last = 0;
	int canCorrectData = 0;
	SYSTIM now;

	get_tim(&now);

	if ( (now - last) >= ULTRASONIC_CORRECT_THREATH_TIME ) {
		canCorrectData = 1;
	} 
	
	
	sensor_port_t port = getUltraSonicSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		if ( additionalKey == 0 ) {
			static unsigned int lastDistance = 0;
			if ( canCorrectData ) {
				lastDistance =  ev3_ultrasonic_sensor_get_distance(port);
			}
			*value = lastDistance;
		} else if ( additionalKey == 1 ) {
			static unsigned int lastListen = 0;
			if ( canCorrectData ) {
				lastListen = ((ev3_ultrasonic_sensor_listen(port))?1:0);
			}
			*value = lastListen;
		}
		return 1;
	}
	return 0;
}

static sensor_port_t getTouchSensorPort(void)
{
	static sensor_port_t touch_sensor_port = TNUM_SENSOR_PORT; // Not used value

	if ( touch_sensor_port == TNUM_SENSOR_PORT ) {
		sensor_port_t port;
		for ( port = EV3_PORT_1; port <= EV3_PORT_4; port++ ) {
			if ( ev3_sensor_get_type(port) == TOUCH_SENSOR ) {
				touch_sensor_port = port;
				break;
			}
		}
	}
	return touch_sensor_port;
}

static int sensorTouchSensorIsPressed(unsigned int additionalKey, int *value) 
{
	sensor_port_t port = getTouchSensorPort();
	if ( port != TNUM_SENSOR_PORT ) {
		*value =  (ev3_touch_sensor_is_pressed(port)?2049:0);
		return 1;
	}
	return 0;

}

// Battery
static int sensorBatteryGet(unsigned int additionalKey, int *value) 
{
	if ( additionalKey == 0 ) {
		*value = ev3_battery_current_mA();
	} else {
		*value = ev3_battery_voltage_mV();
	}
	return 1;
}

// Motor Angle
static int sensorMotorGetCounts(unsigned int additionalKey, int *value) 
{
	// additionalKeyからの変換テーブル
	static const motor_port_t motor_ports[] = {
		EV3_PORT_A,
		EV3_PORT_B,
		EV3_PORT_C,
		EV3_PORT_D,
	};
	// 何回もチェックをするのを避けるためのもの
	static int motor_checks[TNUM_MOTOR_PORT] = {0};
	motor_port_t port = motor_ports[additionalKey];
	int motor_check = motor_checks[additionalKey];

	if ( motor_check == -1 ) {
		// 使用していない
		return 0;
	} else if ( motor_check == 0 ) {
		motor_type_t type = ev3_motor_get_type(port);
		if ( type == MEDIUM_MOTOR || type == LARGE_MOTOR ) {
			motor_checks[additionalKey] = 1;
		} else {
			motor_checks[additionalKey] = -1;
			return 0;
		}
	}

	*value = ev3_motor_get_counts(port);
	return 1;
}
static RobotSensorFuncEntry sensorEntryTable[] = {
	{ 32, 0, sensorButtonIsPressed },
	{ 36, 0, sensorColorSensorGetAmbient },
	{ 40, 0, sensorColorSensorGetColor },
	{ 44, 0, sensorColorSensorGetReflect },
	{ 48, 0, sensorColorSensorGetRgbRaw }, // R
	{ 52, 1, sensorColorSensorGetRgbRaw }, // G
	{ 56, 2, sensorColorSensorGetRgbRaw }, // B
	{ 60, 0, sensorGyroSensorGet },
	{ 64, 1, sensorGyroSensorGet },
	{ 120,0, sensorUltrasonicSensorGet },
	{ 124,1, sensorUltrasonicSensorGet },
	{ 144,0, sensorTouchSensorIsPressed},
	{ 148,0, sensorBatteryGet},
	{ 152,1, sensorBatteryGet},
	{ 288,0, sensorMotorGetCounts},
	{ 292,1, sensorMotorGetCounts},
	{ 296,2, sensorMotorGetCounts},
	{ 300,3, sensorMotorGetCounts},
};


// Connections
static FILE *getConnection(void)
{
	static FILE *fp = 0;
	if ( fp == 0 ) {
		fp = ev3_serial_open_file(EV3_SERIAL_BT);
	}
	return fp;
}

static int receiveMessage(FILE *fp, unsigned char *buf, size_t len) 
{
	int ret = fread(buf, 1, len, fp);
	return ret;
}

static int sendMessage(FILE *fp, unsigned char *buf, size_t len) 
{
	int ret = fwrite(buf, 1, len, fp);

	int i;
#if 0	
	int num = (ret>32)?32:ret;
	for ( i = 0; i < num; i++ ){
		printf("%d ",buf[i]);
	}

	printf("\nfwrite  ret=%d \n",ret);
#endif

	return ret;
}



typedef uint16_t CmdID;
typedef struct message_common {
	CmdID cmd;
	uint16_t len;
} MessageCommon;

typedef struct message_data {
	unsigned char data[MAX_MESSAGE_PAYLOAD];
} MessageData;

typedef struct athrill_update_element {
	unsigned int offset;
	unsigned int data;
} AthrillUpdateElement;

typedef struct message_athrill_update {
	unsigned int num;
	AthrillUpdateElement elements[];
} MessageAthrillUpdate;


typedef struct message_target_update {
	uint64_t ev3time;
	unsigned int num;
} MessageTargetUpdate;

// receiver status enum
enum {
	RECV_STATUS_WAIT_HELLO = 0,
	RECV_STATUS_WAIT_INITIALIZE,
	RECV_STATUS_INITIALIZING,
	RECV_STATUS_RUNNING,
	MAX_RECV_STATUS,
	RECV_STATUS_CANNOT_HAPPEN = -1,
	RECV_STATUS_IGNORE = -2

};


enum {
	RECV_EVENT_NONE = 0,
	RECV_EVENT_HELLO = MAKE_EVENT_ID_FROM_MSG(ATHRILL_MSG_HELLO), // 1
	RECV_EVENT_INITIALIZE = MAKE_EVENT_ID_FROM_MSG(ATHRILL_MSG_INITIALIZE), // 2
	RECV_EVENT_ATHRILL_UPDATE = MAKE_EVENT_ID_FROM_MSG(ATHRILL_MSG_ATHRILL_UPDATE), // 3
	MAX_RECV_EVENTS
};


// Receiver Action Table
typedef int (*StateAction)(const MessageData *data);

void resetMotorsAndSensors(void)
{
	// TODO: implement
}

int receiverStateWaitHello(const MessageData *data) {
	// Nothing to do
	LOG("RCV:WaitHello\n");
	return 0;
}

int receiverStateWaitInitialize(const MessageData *data)
{
	LOG("RCV:WaitInitialize\n");

	resetMotorsAndSensors();
	stp_cyc(CYC_SEND_TASK);

	MessageCommon msg;
	msg.cmd = TARGET_MSG_WELCOME;
	msg.len = 0;

	FILE *fp = getConnection();
	sendMessage(fp, (unsigned char*)&msg, sizeof(msg));
	fflush(fp);
	
	return 0;
}

int receiverStateInitializing(const MessageData *data)
{
	LOG("RCV:Initializing\n");

	MessageCommon msg;
	msg.cmd = TARGET_MSG_INITIALIZE_ACK;
	msg.len = ATHRILL_DEVICE_MEM_SIZE;

	FILE *fp = getConnection();
	sendMessage(fp, (unsigned char*)&msg, sizeof(msg));
	sendMessage(fp, ev3DeviceMem, ATHRILL_DEVICE_MEM_SIZE);
	fflush(fp);

	sta_cyc(CYC_SEND_TASK);
	return 0;

}

int receiveStateRunning(const MessageData *data)
{
//	LOG("RCV:Running\n");
	const MessageAthrillUpdate *update = (const MessageAthrillUpdate *)data;
	unsigned int num = update->num;

	int i;
	const AthrillUpdateElement *p = update->elements;
//	LOG("RCV:num=%d\n",num);

	for ( i = 0; i < num; i++ ) {
		unsigned int offset = p->offset;
		unsigned int data   = p->data;
		ActionMech_ProcessData(robotActionIndex, offset, data);
		// update device memory
		*(unsigned int*)(athrillDeviceMem+offset) = data;
		p++;
	}

	return 0;

}

StateAction receiverActionTable[] = {
	receiverStateWaitHello,
	receiverStateWaitInitialize,
	receiverStateInitializing,
	receiveStateRunning
};


int receiverTransitionTable[MAX_RECV_STATUS][MAX_RECV_EVENTS] = 
{ // RECV_STATUS_WAIT_HELLO
	{ RECV_STATUS_IGNORE, RECV_STATUS_WAIT_INITIALIZE, RECV_STATUS_IGNORE, RECV_STATUS_IGNORE},
  // RECV_STATUS_WAIT_INITIALIZE
	{ RECV_STATUS_CANNOT_HAPPEN, RECV_STATUS_WAIT_INITIALIZE, RECV_STATUS_INITIALIZING, RECV_STATUS_CANNOT_HAPPEN},
  // RECV_STATUS_INITIALIZING
	{ RECV_STATUS_CANNOT_HAPPEN, RECV_STATUS_WAIT_INITIALIZE, RECV_STATUS_CANNOT_HAPPEN,RECV_STATUS_RUNNING},
  // RECV_STATUS_RUNNING
  	{ RECV_STATUS_CANNOT_HAPPEN, RECV_STATUS_WAIT_INITIALIZE, RECV_STATUS_CANNOT_HAPPEN, RECV_STATUS_RUNNING}
};  

static int dispatch(int current_state, const MessageCommon *msg, const MessageData *data)
{
	CmdID cmd = MAKE_EVENT_ID_FROM_MSG(msg->cmd);
	int next_state = current_state;

	int tmp_next_state = receiverTransitionTable[current_state][cmd];

//	printf("cur=%d\n",tmp_next_state);
	if ( tmp_next_state >= 0 ) {
		next_state = tmp_next_state;
		StateAction ap = receiverActionTable[tmp_next_state];
		int ret = 0;
		if ( ap ) {
			ret = ap(data);
		}

	} else if ( tmp_next_state == RECV_STATUS_CANNOT_HAPPEN ) {
		// TODO: Error Handling
		next_state = RECV_STATUS_CANNOT_HAPPEN;
	} else {
		// No Action
	}

	return next_state;
}


// Sender
typedef struct target_update_data
{
	int num;
	AthrillUpdateElement *updates;
	unsigned char *devMem;
} TargetUpadateData;

static void senderVisitor(void *userData, unsigned int offset, int newValue)
{
	TargetUpadateData *p = (TargetUpadateData*)userData;

	// Get Previous Value
	unsigned int prevValue = *(unsigned int*)(p->devMem+offset);
	unsigned int u_newValue  = *(unsigned int*)&newValue;
	// Only Set when value is changed
	if ( prevValue != u_newValue ) {
		AthrillUpdateElement *element = p->updates + p->num;
		element->offset = offset;
		element->data = u_newValue;
		p->num++;
		// Store the newvalue in device Memory
		unsigned int *devMem = (unsigned int*)(p->devMem+offset);
		*devMem = u_newValue;
	}

}
int senderGetUpdates(AthrillUpdateElement *elements, int num)
{
	TargetUpadateData updateData;
	updateData.num = 0;
	updateData.devMem = ev3DeviceMem;
	updateData.updates = elements;

	ActionMech_VisitSensorData(sensorEntryTable,NUMOF(sensorEntryTable),senderVisitor,(void*)&updateData);

	return updateData.num;
}





static AthrillUpdateElement updateElements[ATHRILL_DEVICE_MEM_SIZE/4];
void senderLoop(void)
{
	FILE *fp = getConnection();
	int num = senderGetUpdates(updateElements,NUMOF(updateElements));
	SYSTIM cur;

	get_tim(&cur);

	MessageCommon msg;

	msg.cmd = TARGET_MSG_TARGET_UPDATE;
	// as sizeof might returns 16 because of alignment, use direct size
	msg.len = 8 + 4 + sizeof(AthrillUpdateElement)*num;

	MessageTargetUpdate update;
	update.ev3time = cur;
	update.num = num;

//	printf("cmd=%d len=%d num=%d time=%d num=%d \n",msg.cmd,msg.len,num,update.ev3time,num);

//	sendMessage(fp, (unsigned char*)&msg, sizeof(msg));
	sendMessage(fp, (unsigned char*)&msg.cmd, 2);
	sendMessage(fp, (unsigned char*)&msg.len, 2);
	sendMessage(fp, (unsigned char*)&update.ev3time, 8);
	sendMessage(fp, (unsigned char*)&update.num, 4);
	if ( update.num > 0 ) {
		sendMessage(fp, (unsigned char*)updateElements, sizeof(AthrillUpdateElement)*num);
	}
	fflush(fp);

}

void senderInitialize(void)
{
	// Initialize ev3DevMemory
	// senderGetUpdates updates ev3DevMemory
	int num = senderGetUpdates(updateElements,NUMOF(updateElements));

}

static MessageData payload;
void mainLoop(void)
{
	// 
	FILE *fp = getConnection();
	int current_state = RECV_STATUS_WAIT_HELLO;

	LOG("mainLoop\n");

	while ( 1 ) {
		MessageCommon msg;
		int ret;
		// 最初に共通メッセージを取得する
		ret = receiveMessage(fp, (unsigned char*)&msg, sizeof(msg));
	//		LOG("Msg=%d\n",msg.cmd);

		if ( ret > 0 ) {
			ret = receiveMessage(fp, payload.data, msg.len);
			//LOG("Msg PL msg.len=%d size=%d\n",msg.len,ret);
		}  else if ( ret < 0 ) {
			// TODO: Eror handling
			// Close connection and reset state
			LOG("ret=%d",ret);
		}
		
		current_state = dispatch(current_state, &msg, &payload);

		if ( current_state == RECV_STATUS_CANNOT_HAPPEN ) {
			// TODO:Error Handling
		}

	}


}


void setupRobotConfig(void) 
{
	// TODO: make the contiguration change using configuration file or something

	// This is for HackEV configuration
	static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

	static const motor_port_t
	    left_motor      = EV3_PORT_C,
	    right_motor     = EV3_PORT_B;

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
	ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
	
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

}


void main_task(intptr_t unused) {

	ActionMech_InitializeRobotAction(robotActionIndex,robotFuncEntryTable,NUMOF(robotFuncEntryTable));
	setupRobotConfig();

	senderInitialize();

	// Not Return
	mainLoop();

}

void send_task(intptr_t unused) {
	senderLoop();
	ext_tsk();
}
