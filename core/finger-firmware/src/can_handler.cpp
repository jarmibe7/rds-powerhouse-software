#include "can_handler.hpp"

// TODO: make this safer
void packBuffer(uint8_t* buf, int32_t value, uint8_t start_index) {
  uint8_t index = start_index;
  buf[index++] = 0xFF & (value >> 24);
  buf[index++] = 0xFF & (value >> 16);
  buf[index++] = 0xFF & (value >> 8);
  buf[index] = 0xFF & (value);
}

void sendCanMsgInt(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf, uint32_t id, bool extended, uint8_t length, int32_t data) {
    CAN_message_t msg;

    msg.id = id;
    msg.flags.extended = extended;
    msg.len = length;

    packBuffer(msg.buf, data, 0);

    can_intf.write(msg);
}

void sendCanBuffer(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf, uint32_t id, bool extended, uint8_t length, uint8_t* buffer) {
  CAN_message_t msg;

  msg.id = id;
  msg.flags.extended = extended;
  msg.len = length;

  for(uint8_t i = 0; i < length; i++) {
    msg.buf[i] = buffer[i];
  }

  can_intf.write(msg);
}