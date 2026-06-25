/*
 * QNX Neutrino — Slice 2 UART Test
 * Sends framed MOTOR_START / MOTOR_STOP / STATUS_REQ to STM32
 * Validates ACK / STATUS_RSP / FAULT behavior with CRC checking
 *
 * Build in Momentics for aarch64-qnx target, deploy via qconn.
 * Usage: ./slice2_test [start|stop|status]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include "protocol.h"

#define SERIAL_PORT  "/dev/ser1"

static void make_raw_mode(struct termios *tio)
{
    tio->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio->c_oflag &= ~OPOST;
    tio->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio->c_cflag &= ~(CSIZE | PARENB);
    tio->c_cflag |= CS8;
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
            fprintf(stderr, "write returned 0 before packet was fully sent\n");
            return -1;
        }

        total_written += (uint8_t)written;
    }

    return 0;
}

/*
 * Configure serial port: 115200 8N1, raw mode, no echo.
 * VMIN=0, VTIME=5 gives 500ms read timeout.
 */
static int configure_serial(int fd)
{
    struct termios tio;

    if (tcgetattr(fd, &tio) != 0) {
        perror("tcgetattr");
        return -1;
    }

    /* Raw mode — no line editing, no echo, no signals */
    make_raw_mode(&tio);

    /* 115200 baud */
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    /* 8 data bits, no parity, 1 stop bit */
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    /* Enable receiver, local mode */
    tio.c_cflag |= (CLOCAL | CREAD);

    /* No hardware flow control */
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif

    /*
     * VMIN=0, VTIME=5: read() returns when either:
     *   - at least 1 byte arrives, OR
     *   - 500ms passes with no data (returns 0)
     * This prevents blocking forever if STM32 doesn't respond.
     */
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 5;   /* 5 tenths of a second = 500ms */

    if (tcsetattr(fd, TCSAFLUSH, &tio) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

/*
 * Send a command and wait for a framed response.
 * Returns: 1 = got valid response, 0 = timeout, -1 = error
 */
static int send_and_receive(int fd, uint8_t cmd, packet_parser_t *parser)
{
    uint8_t tx_buf[PACKET_MAX_SIZE];
    uint8_t tx_len;
    uint8_t rx_byte;
    ssize_t bytes_read;
    int     total_read = 0;

    /* Build the outgoing packet */
    tx_len = build_packet(cmd, NULL, 0, tx_buf);
    if (tx_len == 0) {
        fprintf(stderr, "ERROR: build_packet failed for cmd 0x%02X\n", cmd);
        return -1;
    }

    /* Print TX packet for debugging */
    printf("TX [%d bytes]: ", tx_len);
    for (int i = 0; i < tx_len; i++)
        printf("%02X ", tx_buf[i]);
    printf("\n");

    /* Flush any stale data in the RX buffer */
    tcflush(fd, TCIFLUSH);

    /* Send */
    if (write_full(fd, tx_buf, tx_len) != 0) {
        return -1;
    }

    /* Receive byte-by-byte, feed to parser */
    parser_init(parser);

    printf("RX: ");
    while (total_read < PACKET_MAX_SIZE) {
        bytes_read = read(fd, &rx_byte, 1);

        if (bytes_read < 0) {
            perror("read");
            return -1;
        }

        if (bytes_read == 0) {
            /* Timeout — no more data */
            printf(" [TIMEOUT after %d bytes]\n", total_read);
            return 0;
        }

        printf("%02X ", rx_byte);
        total_read++;

        int result = parse_byte(parser, rx_byte);

        if (result == PARSE_COMPLETE) {
            if (parser->cmd == CMD_FAULT) {
                printf(" [FAULT]\n");
                return -1;
            }

            if (response_matches_request(cmd, parser)) {
                printf(" [VALID]\n");
                return 1;
            }

            printf(" [IGNORED CMD 0x%02X] ", parser->cmd);
            parser_init(parser);
        }
        else if (result == PARSE_CRC_ERROR) {
            printf(" [CRC ERROR]\n");
            return -1;
        }
    }

    printf(" [OVERFLOW — no valid packet]\n");
    return -1;
}

/*
 * Print the parsed response in human-readable form.
 */
static void print_response(packet_parser_t *parser)
{
    printf("  Response CMD: 0x%02X", parser->cmd);

    switch (parser->cmd) {
    case CMD_ACK:   printf(" (ACK)");   break;
    case CMD_FAULT: printf(" (FAULT)"); break;
    case CMD_STATUS_RSP: printf(" (STATUS_RSP)"); break;
    default:        printf(" (???)");   break;
    }

    if (parser->data_len > 0) {
        printf("  Data [%d]: ", parser->data_len);
        for (int i = 0; i < parser->data_len; i++)
            printf("%02X ", parser->data[i]);

        if (parser->cmd == CMD_STATUS_RSP && parser->data_len == 1) {
            switch (parser->data[0]) {
            case MOTOR_STATE_IDLE:
                printf(" (motor IDLE)");
                break;
            case MOTOR_STATE_RUNNING:
                printf(" (motor RUNNING)");
                break;
            case MOTOR_STATE_FAULT:
                printf(" (motor FAULT)");
                break;
            default:
                printf(" (motor UNKNOWN)");
                break;
            }
        }
    }

    printf("\n");
}

int main(int argc, char *argv[])
{
    uint8_t cmd;
    int     fd;
    packet_parser_t parser;

    /* Parse command line argument */
    if (argc != 2) {
        fprintf(stderr, "Usage: %s [start|stop|status]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "start") == 0) {
        cmd = CMD_MOTOR_START;
    } else if (strcmp(argv[1], "stop") == 0) {
        cmd = CMD_MOTOR_STOP;
    } else if (strcmp(argv[1], "status") == 0) {
        cmd = CMD_STATUS_REQ;
    } else {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        fprintf(stderr, "Usage: %s [start|stop|status]\n", argv[0]);
        return 1;
    }

    /* Open serial port */
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open " SERIAL_PORT);
        return 1;
    }

    /* Configure serial port */
    if (configure_serial(fd) != 0) {
        close(fd);
        return 1;
    }

    printf("=== Slice 2 Protocol Test ===\n");
    printf("Port: %s\n", SERIAL_PORT);
    printf("Command: %s (0x%02X)\n\n", argv[1], cmd);

    /* Send command and get response */
    int result = send_and_receive(fd, cmd, &parser);

    if (result == 1) {
        print_response(&parser);
        printf("\n*** SLICE 2 TEST PASSED ***\n");
    } else if (result == 0) {
        printf("\n*** TIMEOUT — no response from STM32 ***\n");
        printf("Check: UART wiring, STM32 running, baud rate match\n");
    } else {
        printf("\n*** TEST FAILED ***\n");
    }

    close(fd);
    return (result == 1) ? 0 : 1;
}
