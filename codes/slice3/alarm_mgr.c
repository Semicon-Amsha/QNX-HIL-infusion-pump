#include <errno.h>
#include <stdio.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "pump_ipc.h"

typedef union {
    struct _pulse pulse;
    char raw[32];
} alarm_recv_t;

static const char *pulse_text(int code)
{
    switch (code) {
    case PULSE_FAULT_UART_TIMEOUT:
        return "UART_TIMEOUT";
    case PULSE_FAULT_PROTOCOL_ERROR:
        return "PROTOCOL_ERROR";
    case PULSE_FAULT_REMOTE_FAULT:
        return "REMOTE_FAULT";
    default:
        return "UNKNOWN";
    }
}

int main(void)
{
    name_attach_t *attach;
    alarm_recv_t recv;

    attach = name_attach(NULL, PUMP_ALARM_NAME, 0);
    if (attach == NULL) {
        perror("name_attach");
        return 1;
    }

    printf("alarm_mgr ready on name '%s'\n", PUMP_ALARM_NAME);

    for (;;) {
        int rcvid = MsgReceive(attach->chid, &recv, sizeof(recv), NULL);
        if (rcvid < 0) {
            perror("MsgReceive");
            continue;
        }

        if (rcvid == 0) {
            if (recv.pulse.code == _PULSE_CODE_DISCONNECT) {
                ConnectDetach(recv.pulse.scoid);
                continue;
            }

            printf("alarm_mgr pulse: %s (code=%d value=%d)\n",
                   pulse_text(recv.pulse.code),
                   recv.pulse.code,
                   recv.pulse.value.sival_int);
            continue;
        }

        MsgError(rcvid, ENOSYS);
    }

    name_detach(attach, 0);
    return 0;
}
