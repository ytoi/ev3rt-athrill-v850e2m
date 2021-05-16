#ifndef __MSGID_H
#define __MSGID_H

// Message Cmd Definition

#define MSG_EVENT_MASK (0x7f)
#define MAKE_EVENT_ID_FROM_MSG(msg) ((msg) & MSG_EVENT_MASK)

#define ATHRILL_MSG_HELLO (0x01)
#define ATHRILL_MSG_INITIALIZE (0x02)
#define ATHRILL_MSG_ATHRILL_UPDATE (0x03)

#define TARGET_MSG_WELCOME (0x81)
#define TARGET_MSG_INITIALIZE_ACK (0x82)
#define TARGET_MSG_TARGET_UPDATE (0x83)

#endif // !__MSGID_H