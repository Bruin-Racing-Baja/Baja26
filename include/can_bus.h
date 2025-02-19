#ifndef CANBUS_H
#define CANBUS_H
#include <FlexCAN_T4.h>
#include <types.h>
class CAN_BUS
{
    public:
        CAN_BUS(FlexCAN_T4FD<CAN1, RX_SIZE_256, TX_SIZE_16>* flexcan_bus);
        void init();
        u8 send_command(u32 func_id, u32 node_id, bool remote, u8 buf[]);
        void can_parse(const CANFD_message_t &msg);

    private:
        FlexCAN_T4FD<CAN1, RX_SIZE_256, TX_SIZE_16>* flexcan_bus;
};
  
    
#endif