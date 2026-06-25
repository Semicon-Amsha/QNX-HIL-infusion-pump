#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Sync byte */
#define PACKET_SYNC        0xAA

/* Command IDs */
#define CMD_MOTOR_START    0x01
#define CMD_MOTOR_STOP     0x02
#define CMD_STATUS_REQ     0x03
#define CMD_HEARTBEAT      0x04
#define CMD_ACK            0x05
#define CMD_FAULT          0x06
#define CMD_STATUS_RSP     0x07

/*
 * STATUS_RSP payload format
 * data_len = 1
 * data[0]  = motor state
 */
#define MOTOR_STATE_IDLE      0x00
#define MOTOR_STATE_RUNNING   0x01
#define MOTOR_STATE_FAULT     0x02

/* Packet limits */
#define PACKET_MAX_DATA    16
#define PACKET_MAX_SIZE    (PACKET_MAX_DATA + 4)

/* Parser states */
#define STATE_WAIT_SYNC    0
#define STATE_WAIT_CMD     1
#define STATE_WAIT_LEN     2
#define STATE_WAIT_DATA    3
#define STATE_WAIT_CRC     4

/* Parser return values */
#define PARSE_IN_PROGRESS  0
#define PARSE_COMPLETE     1
#define PARSE_CRC_ERROR   -1

typedef struct {
    uint8_t state;
    uint8_t cmd;
    uint8_t data_len;
    uint8_t data[PACKET_MAX_DATA];
    uint8_t data_idx;
} packet_parser_t;

uint8_t crc8(uint8_t *data, uint8_t len);
uint8_t build_packet(uint8_t cmd, uint8_t *data, uint8_t data_len, uint8_t *out_buf);
void    parser_init(packet_parser_t *parser);
int     parse_byte(packet_parser_t *parser, uint8_t byte);

#endif /* PROTOCOL_H */
