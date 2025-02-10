#include "constants.h"
#include <FlexCAN_T4.h>
#include <odrive.h>
#include <can_bus.h>
#include <string>
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> can_test;

CAN_BUS::CAN_BUS()
{
  flexcan_bus.begin();
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = FLEXCAN_BAUD_RATE;
  config.baudrateFD = FLEXCAN_BAUD_RATE;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 75;
  flexcan_bus.setBaudRate(config);
  //flexcan_bus.setMB(FLEXCAN_MAX_MAILBOX);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableMBInterrupts();
  flexcan_bus.onReceive(can_parse);
}
/**
 * @brief Send a command
 * @param cmd_id ODrive CAN command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for
 * requesters)
 * @param buf 8-wide array of command data bytes
 * @return Status code of command send
 */
u8 CAN_BUS::send_command(u32 func_id, u32 node_id, bool remote, u8 buf[], FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16>* can_test) {
  // TODO: Fix error mwessages
  CANFD_message_t msg;

  if (func_id < 0x00 || 0x1f < func_id) {
    return ODrive::CMD_ERROR_INVALID_COMMAND;
  }

  msg.id = (node_id << 5) | func_id;
  msg.len = 64;
  memcpy(&msg.buf, buf, 64);
  //msg.flags.remote = remote;

  int write_code = can_test->write(msg);
  if (write_code == -1) {
    return ODrive::CMD_ERROR_WRITE_FAILED;
  }
  return ODrive::CMD_SUCCESS;
}

void CAN_BUS::can_parse(const CANFD_message_t &msg)
{
  std::string s = "";
  for (int i = 0; i < msg.len; i++)
    s += std::to_string(msg.buf[i]);
  Serial.print(s.c_str());
}