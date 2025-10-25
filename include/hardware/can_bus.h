#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <FlexCAN_T4.h>
#include <types.h>

// CAN bus parsing function
void can_parse(const CAN_message_t &msg);

// CAN send command utility
void send_command(u32 node_id, u32 cmd_id, bool remote, float val);

#endif