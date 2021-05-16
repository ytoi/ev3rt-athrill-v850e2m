#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

enum {
	RECV_EVENT_NONE = 0,
	RECV_EVENT_HELLO = 1,
	RECV_EVENT_INITIALIZE = 2,
	RECV_EVENT_ATHRILL_UPDATE = 3,
	MAX_RECV_EVENTS
};

#define BT_FILE "../__ev3rt_bt_in"

typedef unsigned short CmdID;
typedef struct message_common {
	CmdID cmd;
	unsigned short len;
} MessageCommon;

typedef struct message_athrill_update_element {
    unsigned int offset;
    unsigned int data;
} MessageAthrillUpdateElement;

#define NUMOF(table) ((sizeof(table)/sizeof(table[0])))

unsigned char payload[1024];
void main(void)
{
    FILE *fp = fopen(BT_FILE,"w");

    if ( fp ==  0 ) {
        printf("Open Faild err=%d\n",errno);
        return;
    }
    printf("open success fp=%p\n",fp);
    MessageCommon hello = {RECV_EVENT_HELLO,0};

    int len = fwrite((void*)&hello, 1, sizeof(hello),fp);
    printf("write ret=%d errno=%d \n",len,errno);

    memset(payload,0,sizeof(payload));

    MessageCommon init = {RECV_EVENT_INITIALIZE,1024};
    len = fwrite((void*)&init, 1, sizeof(init),fp);
    printf("write ret=%d errno=%d \n",len,errno);
    len = fwrite((void*)payload, 1, sizeof(payload),fp);
    printf("write ret=%d errno=%d \n",len,errno);


    MessageAthrillUpdateElement updateTable[] = {
        // LED
        { 32, 1 },
        // POWER_B
        { 40, 20},
        // POWER_C
        { 44, 30},

        // RESET ANGLE A
        { 68,  1},
        // RESET GYRO 
        { 84,  1}
    } ;

    MessageCommon update = {RECV_EVENT_ATHRILL_UPDATE,4+sizeof(updateTable)};

    len = fwrite((void*)&update, 1, sizeof(update),fp);
    printf("write ret=%d errno=%d \n",len,errno);
    unsigned int num = NUMOF(updateTable);
    len = fwrite((void*)&num, 1, sizeof(num),fp);
    printf("write ret=%d errno=%d \n",len,errno);
    len = fwrite((void*)updateTable, 1, sizeof(updateTable),fp);
    printf("write ret=%d errno=%d \n",len,errno);


    len = fwrite((void*)&update, 1, sizeof(update),fp);
    printf("write ret=%d errno=%d \n",len,errno);
    len = fwrite((void*)&num, 1, sizeof(num),fp);
    printf("write ret=%d errno=%d \n",len,errno);

    len = fwrite((void*)updateTable, 1, sizeof(updateTable),fp);
    printf("write ret=%d errno=%d \n",len,errno);


    fclose(fp);
    
    return ;

}
