#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "pump_ipc.h"

#define SERIAL_PORT "/dev/ser1"
#define HEARTBEAT_TIMEOUT_MS 2000

typedef union {
    pump_msg_t msg;
    struct _pulse pulse;
} receive_msg_t;

typedef struct {
    uint8_t cmd;
    uint8_t data_len;
    uint8_t data[PACKET_MAX_DATA];
} packet_info_t;

typedef struct {
    int fd;
    int alarm_coid;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int response_ready;
    int response_error;
    packet_info_t response;
    int waiting_for_response;
    uint8_t expected_cmd;
    int heartbeat_seen;
    int heartbeat_missed;
    uint64_t last_heartbeat_ms;
} controller_state_t;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static void make_raw_mode(struct termios *tio)
{
    tio->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio->c_oflag &= ~OPOST;
    tio->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio->c_cflag &= ~(CSIZE | PARENB);
    tio->c_cflag |= CS8;
}

static int configure_serial(int fd)
{
    struct termios tio;

    if (tcgetattr(fd, &tio) != 0) {
        perror("tcgetattr");
        return -1;
    }

    make_raw_mode(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag |= (CLOCAL | CREAD);
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSAFLUSH, &tio) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

static int write_full(int fd, const uint8_t *buf, uint8_t len)
{
    uint8_t total_written = 0;

    while (total_written < len) {
        ssize_t written = write(fd, buf + total_written, len - total_written);

        if (written < 0) {
            if (errno == EINTR)
                continue;

            perror("write");
            return -1;
        }

        if (written == 0) {
            fprintf(stderr, "write returned 0 before full packet send\n");
            return -1;
        }

        total_written += (uint8_t)written;
    }

    return 0;
}

static void print_packet(const char *prefix, const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    printf("%s [%u bytes]: ", prefix, (unsigned)len);
    for (i = 0; i < len; i++)
        printf("%02X ", buf[i]);
    printf("\n");
}

static int connect_alarm_mgr(void)
{
    int coid = name_open(PUMP_ALARM_NAME, 0);
    if (coid < 0)
        perror("name_open alarm_mgr");
    return coid;
}

static void notify_alarm(int *alarm_coid, int pulse_code, int pulse_value)
{
    if (*alarm_coid < 0)
        *alarm_coid = connect_alarm_mgr();

    if (*alarm_coid < 0)
        return;

    if (MsgSendPulse(*alarm_coid, PUMP_ALARM_PRIO, pulse_code, pulse_value) != 0) {
        perror("MsgSendPulse");
        name_close(*alarm_coid);
        *alarm_coid = -1;
        return;
    }

    printf("alarm pulse sent: code=%d value=%d\n", pulse_code, pulse_value);
}

static uint8_t expected_response_cmd(uint8_t request_cmd)
{
    switch (request_cmd) {
    case CMD_MOTOR_START:
    case CMD_MOTOR_STOP:
        return CMD_ACK;
    case CMD_STATUS_REQ:
        return CMD_STATUS_RSP;
    default:
        return 0;
    }
}

static int response_matches_request(uint8_t request_cmd, const packet_info_t *packet)
{
    uint8_t expected_cmd = expected_response_cmd(request_cmd);

    if (expected_cmd == 0)
        return 0;
    if (packet->cmd != expected_cmd)
        return 0;
    if (request_cmd == CMD_STATUS_REQ && packet->data_len != 1)
        return 0;

    return 1;
}

static int valid_client_command(uint8_t cmd)
{
    return (cmd == CMD_MOTOR_START ||
            cmd == CMD_MOTOR_STOP  ||
            cmd == CMD_STATUS_REQ);
}

static void publish_response(controller_state_t *state, int error_code, const packet_info_t *packet)
{
    pthread_mutex_lock(&state->lock);
    if (state->waiting_for_response) {
        state->response_error = error_code;
        if (packet != NULL)
            state->response = *packet;
        state->response_ready = 1;
        pthread_cond_signal(&state->cond);
    }
    pthread_mutex_unlock(&state->lock);
}

static void *uart_rx_thread(void *arg)
{
    controller_state_t *state = (controller_state_t *)arg;
    packet_parser_t parser;
    uint8_t rx_byte;

    parser_init(&parser);

    for (;;) {
        ssize_t bytes_read = read(state->fd, &rx_byte, 1);

        if (bytes_read < 0) {
            if (errno == EINTR)
                continue;
            perror("read");
            continue;
        }

        if (bytes_read == 0)
            continue;

        printf("RX byte: %02X\n", rx_byte);

        switch (parse_byte(&parser, rx_byte)) {
        case PARSE_IN_PROGRESS:
            break;

        case PARSE_CRC_ERROR:
            notify_alarm(&state->alarm_coid, PULSE_FAULT_PROTOCOL_ERROR, 0);
            publish_response(state, PUMP_STATUS_PROTOCOL_ERROR, NULL);
            break;

        case PARSE_COMPLETE: {
            packet_info_t packet;
            packet.cmd = parser.cmd;
            packet.data_len = parser.data_len;
            memcpy(packet.data, parser.data, parser.data_len);

            if (packet.cmd == CMD_HEARTBEAT) {
                pthread_mutex_lock(&state->lock);
                state->heartbeat_seen = 1;
                state->heartbeat_missed = 0;
                state->last_heartbeat_ms = now_ms();
                pthread_mutex_unlock(&state->lock);
                break;
            }

            if (packet.cmd == CMD_FAULT) {
                notify_alarm(&state->alarm_coid, PULSE_FAULT_REMOTE_FAULT, 1);
                publish_response(state, PUMP_STATUS_REMOTE_FAULT, &packet);
                break;
            }

            publish_response(state, PUMP_STATUS_OK, &packet);
            break;
        }
        }
    }

    return NULL;
}

static void *heartbeat_thread(void *arg)
{
    controller_state_t *state = (controller_state_t *)arg;

    for (;;) {
        delay(200);

        pthread_mutex_lock(&state->lock);
        if (state->heartbeat_seen &&
            !state->heartbeat_missed &&
            (now_ms() - state->last_heartbeat_ms) > HEARTBEAT_TIMEOUT_MS) {
            state->heartbeat_missed = 1;
            pthread_mutex_unlock(&state->lock);
            notify_alarm(&state->alarm_coid, PULSE_FAULT_HEARTBEAT_MISS, HEARTBEAT_TIMEOUT_MS);
            continue;
        }
        pthread_mutex_unlock(&state->lock);
    }

    return NULL;
}

static int process_uart_request(controller_state_t *state, const pump_msg_t *msg, pump_reply_t *reply)
{
    uint8_t tx_buf[PACKET_MAX_SIZE];
    uint8_t tx_len;
    uint64_t deadline;

    pthread_mutex_lock(&state->lock);
    if (state->heartbeat_missed) {
        pthread_mutex_unlock(&state->lock);
        return PUMP_STATUS_HEARTBEAT_MISS;
    }
    pthread_mutex_unlock(&state->lock);

    tx_len = build_packet(msg->cmd, (uint8_t *)msg->data, msg->data_len, tx_buf);
    if (tx_len == 0)
        return PUMP_STATUS_BAD_REQUEST;

    tcflush(state->fd, TCIFLUSH);
    print_packet("TX", tx_buf, tx_len);

    pthread_mutex_lock(&state->lock);
    state->response_ready = 0;
    state->response_error = PUMP_STATUS_TIMEOUT;
    memset(&state->response, 0, sizeof(state->response));
    state->waiting_for_response = 1;
    pthread_mutex_unlock(&state->lock);

    if (write_full(state->fd, tx_buf, tx_len) != 0) {
        pthread_mutex_lock(&state->lock);
        state->waiting_for_response = 0;
        pthread_mutex_unlock(&state->lock);
        return PUMP_STATUS_IO_ERROR;
    }

    deadline = now_ms() + 700;

    pthread_mutex_lock(&state->lock);
    while (!state->response_ready) {
        uint64_t remaining_ms;
        struct timespec ts;

        if (state->heartbeat_missed) {
            state->waiting_for_response = 0;
            pthread_mutex_unlock(&state->lock);
            return PUMP_STATUS_HEARTBEAT_MISS;
        }

        if (now_ms() >= deadline) {
            state->waiting_for_response = 0;
            pthread_mutex_unlock(&state->lock);
            notify_alarm(&state->alarm_coid, PULSE_FAULT_UART_TIMEOUT, msg->cmd);
            return PUMP_STATUS_TIMEOUT;
        }

        remaining_ms = deadline - now_ms();
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += (time_t)(remaining_ms / 1000);
        ts.tv_nsec += (long)((remaining_ms % 1000) * 1000000L);
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000L;
        }

        pthread_cond_timedwait(&state->cond, &state->lock, &ts);
    }

    state->waiting_for_response = 0;

    if (state->response_error != PUMP_STATUS_OK) {
        int status = state->response_error;
        pthread_mutex_unlock(&state->lock);
        return status;
    }

    if (!response_matches_request(msg->cmd, &state->response)) {
        uint8_t bad_cmd = state->response.cmd;
        pthread_mutex_unlock(&state->lock);
        notify_alarm(&state->alarm_coid, PULSE_FAULT_PROTOCOL_ERROR, bad_cmd);
        return PUMP_STATUS_PROTOCOL_ERROR;
    }

    reply->status = PUMP_STATUS_OK;
    reply->rsp_cmd = state->response.cmd;
    reply->data_len = state->response.data_len;
    memcpy(reply->data, state->response.data, state->response.data_len);
    pthread_mutex_unlock(&state->lock);
    return PUMP_STATUS_OK;
}

static void init_reply(pump_reply_t *reply)
{
    memset(reply, 0, sizeof(*reply));
    reply->status = PUMP_STATUS_PROTOCOL_ERROR;
}

int main(void)
{
    name_attach_t *attach;
    controller_state_t state;
    receive_msg_t recv;
    pump_reply_t reply;
    pthread_t rx_tid;
    pthread_t hb_tid;

    memset(&state, 0, sizeof(state));
    state.fd = -1;
    state.alarm_coid = -1;
    pthread_mutex_init(&state.lock, NULL);
    pthread_cond_init(&state.cond, NULL);

    attach = name_attach(NULL, PUMP_SERVER_NAME, 0);
    if (attach == NULL) {
        perror("name_attach");
        return 1;
    }

    state.fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (state.fd < 0) {
        perror("open " SERIAL_PORT);
        name_detach(attach, 0);
        return 1;
    }

    if (configure_serial(state.fd) != 0) {
        close(state.fd);
        name_detach(attach, 0);
        return 1;
    }

    state.alarm_coid = connect_alarm_mgr();
    state.last_heartbeat_ms = now_ms();

    if (pthread_create(&rx_tid, NULL, uart_rx_thread, &state) != 0) {
        perror("pthread_create rx");
        close(state.fd);
        name_detach(attach, 0);
        return 1;
    }

    if (pthread_create(&hb_tid, NULL, heartbeat_thread, &state) != 0) {
        perror("pthread_create heartbeat");
        close(state.fd);
        name_detach(attach, 0);
        return 1;
    }

    printf("motor_ctrl ready on name '%s'\n", PUMP_SERVER_NAME);
    printf("UART port: %s\n", SERIAL_PORT);
    printf("Heartbeat timeout: %d ms\n", HEARTBEAT_TIMEOUT_MS);

    for (;;) {
        int rcvid = MsgReceive(attach->chid, &recv, sizeof(recv), NULL);
        if (rcvid < 0) {
            perror("MsgReceive");
            continue;
        }

        if (rcvid == 0) {
            if (recv.pulse.code == _PULSE_CODE_DISCONNECT)
                ConnectDetach(recv.pulse.scoid);
            continue;
        }

        init_reply(&reply);

        if (recv.msg.type != PUMP_MSG_TYPE_CMD ||
            recv.msg.data_len > PACKET_MAX_DATA ||
            !valid_client_command(recv.msg.cmd)) {
            reply.status = PUMP_STATUS_BAD_REQUEST;
            MsgReply(rcvid, EOK, &reply, sizeof(reply));
            continue;
        }

        reply.status = process_uart_request(&state, &recv.msg, &reply);
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
}
