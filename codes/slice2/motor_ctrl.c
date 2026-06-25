#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "pump_ipc.h"

#define SERIAL_PORT "/dev/ser1"

typedef union {
    pump_msg_t msg;
    struct _pulse pulse;
} receive_msg_t;

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
    tio.c_cc[VTIME] = 5;

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

static int response_matches_request(uint8_t request_cmd, packet_parser_t *parser)
{
    uint8_t expected_cmd = expected_response_cmd(request_cmd);

    if (expected_cmd == 0)
        return 0;

    if (parser->cmd != expected_cmd)
        return 0;

    if (request_cmd == CMD_STATUS_REQ && parser->data_len != 1)
        return 0;

    return 1;
}

static int valid_client_command(uint8_t cmd)
{
    return (cmd == CMD_MOTOR_START ||
            cmd == CMD_MOTOR_STOP  ||
            cmd == CMD_STATUS_REQ);
}

static int process_uart_request(int fd, const pump_msg_t *msg, pump_reply_t *reply)
{
    uint8_t tx_buf[PACKET_MAX_SIZE];
    uint8_t tx_len;
    uint8_t rx_byte;
    ssize_t bytes_read;
    int total_read = 0;
    packet_parser_t parser;

    tx_len = build_packet(msg->cmd, (uint8_t *)msg->data, msg->data_len, tx_buf);
    if (tx_len == 0)
        return PUMP_STATUS_BAD_REQUEST;

    tcflush(fd, TCIFLUSH);
    print_packet("TX", tx_buf, tx_len);

    if (write_full(fd, tx_buf, tx_len) != 0)
        return PUMP_STATUS_IO_ERROR;

    parser_init(&parser);

    while (total_read < (PACKET_MAX_SIZE * 4)) {
        bytes_read = read(fd, &rx_byte, 1);
        if (bytes_read < 0) {
            if (errno == EINTR)
                continue;

            perror("read");
            return PUMP_STATUS_IO_ERROR;
        }

        if (bytes_read == 0) {
            printf("RX timeout waiting for response\n");
            return PUMP_STATUS_TIMEOUT;
        }

        printf("RX byte: %02X\n", rx_byte);
        total_read++;

        switch (parse_byte(&parser, rx_byte)) {
        case PARSE_IN_PROGRESS:
            break;

        case PARSE_CRC_ERROR:
            printf("RX packet failed CRC validation\n");
            return PUMP_STATUS_PROTOCOL_ERROR;

        case PARSE_COMPLETE:
            if (parser.cmd == CMD_HEARTBEAT) {
                printf("Ignoring heartbeat while waiting for command response\n");
                parser_init(&parser);
                break;
            }

            if (parser.cmd == CMD_FAULT) {
                printf("STM32 reported FAULT packet\n");
                return PUMP_STATUS_REMOTE_FAULT;
            }

            if (!response_matches_request(msg->cmd, &parser)) {
                printf("Unexpected response cmd=0x%02X for request 0x%02X\n",
                       parser.cmd, msg->cmd);
                return PUMP_STATUS_PROTOCOL_ERROR;
            }

            reply->status = PUMP_STATUS_OK;
            reply->rsp_cmd = parser.cmd;
            reply->data_len = parser.data_len;
            memcpy(reply->data, parser.data, parser.data_len);
            return PUMP_STATUS_OK;
        }
    }

    return PUMP_STATUS_PROTOCOL_ERROR;
}

static void init_reply(pump_reply_t *reply)
{
    memset(reply, 0, sizeof(*reply));
    reply->status = PUMP_STATUS_PROTOCOL_ERROR;
}

int main(void)
{
    name_attach_t *attach;
    int fd;
    receive_msg_t recv;
    pump_reply_t reply;

    attach = name_attach(NULL, PUMP_SERVER_NAME, 0);
    if (attach == NULL) {
        perror("name_attach");
        return 1;
    }

    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open " SERIAL_PORT);
        name_detach(attach, 0);
        return 1;
    }

    if (configure_serial(fd) != 0) {
        close(fd);
        name_detach(attach, 0);
        return 1;
    }

    printf("motor_ctrl ready on name '%s'\n", PUMP_SERVER_NAME);
    printf("UART port: %s\n", SERIAL_PORT);

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

        reply.status = process_uart_request(fd, &recv.msg, &reply);
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }

    close(fd);
    name_detach(attach, 0);
    return 0;
}
