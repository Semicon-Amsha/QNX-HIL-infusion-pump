#ifndef PUMP_IPC_H
#define PUMP_IPC_H

#include <stdint.h>
#include <sys/neutrino.h>

#include "protocol.h"

#define PUMP_SERVER_NAME   "pump_motor_ctrl"
#define PUMP_ALARM_NAME    "pump_alarm_mgr"
#define PUMP_MSG_TYPE_CMD  0x1001

#define PUMP_STATUS_OK                0
#define PUMP_STATUS_TIMEOUT          -1
#define PUMP_STATUS_IO_ERROR         -2
#define PUMP_STATUS_PROTOCOL_ERROR   -3
#define PUMP_STATUS_REMOTE_FAULT     -4
#define PUMP_STATUS_BAD_REQUEST      -5
#define PUMP_STATUS_HEARTBEAT_MISS   -6

#define PUMP_ALARM_PRIO 10

#define PULSE_FAULT_UART_TIMEOUT     (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_FAULT_PROTOCOL_ERROR   (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_FAULT_REMOTE_FAULT     (_PULSE_CODE_MINAVAIL + 3)
#define PULSE_FAULT_HEARTBEAT_MISS   (_PULSE_CODE_MINAVAIL + 4)

typedef struct {
    uint16_t type;
    uint16_t seq;
    uint8_t  cmd;
    uint8_t  data_len;
    uint8_t  data[PACKET_MAX_DATA];
} pump_msg_t;

typedef struct {
    int32_t  status;
    uint8_t  rsp_cmd;
    uint8_t  data_len;
    uint8_t  data[PACKET_MAX_DATA];
} pump_reply_t;

#endif /* PUMP_IPC_H */
