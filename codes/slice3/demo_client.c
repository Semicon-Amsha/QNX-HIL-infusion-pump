#include <stdio.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "pump_ipc.h"

static int parse_command(const char *arg, uint8_t *cmd)
{
    if (strcmp(arg, "start") == 0) {
        *cmd = CMD_MOTOR_START;
        return 0;
    }

    if (strcmp(arg, "stop") == 0) {
        *cmd = CMD_MOTOR_STOP;
        return 0;
    }

    if (strcmp(arg, "status") == 0) {
        *cmd = CMD_STATUS_REQ;
        return 0;
    }

    return -1;
}

static const char *status_text(int32_t status)
{
    switch (status) {
    case PUMP_STATUS_OK:              return "OK";
    case PUMP_STATUS_TIMEOUT:         return "TIMEOUT";
    case PUMP_STATUS_IO_ERROR:        return "IO_ERROR";
    case PUMP_STATUS_PROTOCOL_ERROR:  return "PROTOCOL_ERROR";
    case PUMP_STATUS_REMOTE_FAULT:    return "REMOTE_FAULT";
    case PUMP_STATUS_BAD_REQUEST:     return "BAD_REQUEST";
    default:                          return "UNKNOWN";
    }
}

static void print_status_payload(const pump_reply_t *reply)
{
    if (reply->rsp_cmd != CMD_STATUS_RSP || reply->data_len != 1)
        return;

    switch (reply->data[0]) {
    case MOTOR_STATE_IDLE:
        printf("Motor state: IDLE\n");
        break;

    case MOTOR_STATE_RUNNING:
        printf("Motor state: RUNNING\n");
        break;

    case MOTOR_STATE_FAULT:
        printf("Motor state: FAULT\n");
        break;

    default:
        printf("Motor state: UNKNOWN (0x%02X)\n", reply->data[0]);
        break;
    }
}

int main(int argc, char *argv[])
{
    int coid;
    uint8_t cmd;
    pump_msg_t msg;
    pump_reply_t reply;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s [start|stop|status]\n", argv[0]);
        return 1;
    }

    if (parse_command(argv[1], &cmd) != 0) {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        return 1;
    }

    coid = name_open(PUMP_SERVER_NAME, 0);
    if (coid < 0) {
        perror("name_open");
        return 1;
    }

    memset(&msg, 0, sizeof(msg));
    memset(&reply, 0, sizeof(reply));

    msg.type = PUMP_MSG_TYPE_CMD;
    msg.seq = 1;
    msg.cmd = cmd;

    if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) < 0) {
        perror("MsgSend");
        name_close(coid);
        return 1;
    }

    printf("Reply status: %s (%ld)\n", status_text(reply.status), (long)reply.status);
    printf("Response cmd: 0x%02X\n", reply.rsp_cmd);
    printf("Data len: %u\n", (unsigned)reply.data_len);

    if (reply.status == PUMP_STATUS_OK)
        print_status_payload(&reply);

    name_close(coid);
    return (reply.status == PUMP_STATUS_OK) ? 0 : 1;
}
