#include "protocol.h"
#include <stddef.h>

uint8_t crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];

        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }

    return crc;
}

uint8_t build_packet(uint8_t cmd, uint8_t *data, uint8_t data_len, uint8_t *out_buf)
{
    if (data_len > PACKET_MAX_DATA)
        return 0;
    if (data_len > 0 && data == NULL)
        return 0;

    out_buf[0] = PACKET_SYNC;
    out_buf[1] = cmd;
    out_buf[2] = data_len;

    for (uint8_t i = 0; i < data_len; i++)
        out_buf[3 + i] = data[i];

    out_buf[3 + data_len] = crc8(&out_buf[1], 2 + data_len);
    return 4 + data_len;
}

void parser_init(packet_parser_t *parser)
{
    parser->state    = STATE_WAIT_SYNC;
    parser->cmd      = 0;
    parser->data_len = 0;
    parser->data_idx = 0;
}

int parse_byte(packet_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {
    case STATE_WAIT_SYNC:
        if (byte == PACKET_SYNC)
            parser->state = STATE_WAIT_CMD;
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_CMD:
        parser->cmd = byte;
        parser->state = STATE_WAIT_LEN;
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_LEN:
        parser->data_len = byte;
        parser->data_idx = 0;

        if (parser->data_len > PACKET_MAX_DATA) {
            parser_init(parser);
            return PARSE_CRC_ERROR;
        }

        parser->state = (parser->data_len == 0) ? STATE_WAIT_CRC : STATE_WAIT_DATA;
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_DATA:
        parser->data[parser->data_idx] = byte;
        parser->data_idx++;

        if (parser->data_idx >= parser->data_len)
            parser->state = STATE_WAIT_CRC;
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_CRC: {
        uint8_t crc_buf[2 + PACKET_MAX_DATA];
        crc_buf[0] = parser->cmd;
        crc_buf[1] = parser->data_len;

        for (uint8_t i = 0; i < parser->data_len; i++)
            crc_buf[2 + i] = parser->data[i];

        if (byte == crc8(crc_buf, 2 + parser->data_len)) {
            parser->state = STATE_WAIT_SYNC;
            parser->data_idx = 0;
            return PARSE_COMPLETE;
        }

        parser_init(parser);
        return PARSE_CRC_ERROR;
    }

    default:
        parser_init(parser);
        return PARSE_CRC_ERROR;
    }
}
