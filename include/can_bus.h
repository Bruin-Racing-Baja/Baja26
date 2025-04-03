#ifndef CANBUS_H
#define CANBUS_H
#include <FlexCAN_T4.h>
#include <types.h>
#include <odrive.h>
class Can_Bus
{
    public:
        Can_Bus();
        static void set_odrive_ecvt(ODrive* odrive);
        static void set_odrive_ecent(ODrive* odrive);
        static u8 send_command(u32 func_id, u32 node_id, bool remote, u8 buf[]);
        static u8 send_command(CAN_message_t msg);
        static void can_parse(const CAN_message_t &msg);
    
    private:
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
        static ODrive* odrive_ecvt;
        static ODrive* odrive_ecent;
        static const u8 ODRIVE_ECVT_NODE_ID = 1;
        static const u8 ODRIVE_ECENT_NODE_ID = 2;
        static const u8 CONTROLS_PCB_NODE_ID = 3;
};
  
    
#endif