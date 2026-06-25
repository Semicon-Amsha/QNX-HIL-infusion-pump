#include "protocol.h"
#include <stddef.h>   /* for NULL */

/*
 * CRC8-MAXIM (Dallas) — polynomial 0x31, reflected as 0x8C
 * LSB-first implementation
 * Init: 0x00, no final XOR
 * Standard check: CRC8("123456789") = 0xA1
 */
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

/*
 * Build a framed packet into out_buf.
 * Format: [0xAA] [CMD] [LEN] [DATA...] [CRC8]
 * CRC is computed over CMD + LEN + DATA.
 * Returns total packet length, or 0 on error.
 */
uint8_t build_packet(uint8_t cmd, uint8_t *data, uint8_t data_len, uint8_t *out_buf)
{
    /* Validate inputs */
    if (data_len > PACKET_MAX_DATA)
        return 0;
    if (data_len > 0 && data == NULL)
        return 0;

    /* Assemble packet */
    out_buf[0] = PACKET_SYNC;
    out_buf[1] = cmd;
    out_buf[2] = data_len;

    for (uint8_t i = 0; i < data_len; i++)
        out_buf[3 + i] = data[i];

    /* CRC over CMD + LEN + DATA */
    out_buf[3 + data_len] = crc8(&out_buf[1], 2 + data_len);

    return 4 + data_len;
}

/*
 * Initialize (or reset) the parser to its idle state.
 * Call once at startup, and the parser resets itself
 * internally after each completed or failed packet.
 */
void parser_init(packet_parser_t *parser)
{
    parser->state    = STATE_WAIT_SYNC;
    parser->cmd      = 0;
    parser->data_len = 0;
    parser->data_idx = 0;
}

/*
 * Feed one byte into the parser state machine.
 *
 * Returns:
 *   PARSE_IN_PROGRESS (0)  — need more bytes
 *   PARSE_COMPLETE    (1)  — valid packet ready in parser->cmd / parser->data
 *   PARSE_CRC_ERROR  (-1)  — CRC mismatch, packet discarded
 *
 * After PARSE_COMPLETE or PARSE_CRC_ERROR, the parser
 * automatically resets to STATE_WAIT_SYNC.
 */
int parse_byte(packet_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {

    case STATE_WAIT_SYNC:
        if (byte == PACKET_SYNC) {
            parser->state = STATE_WAIT_CMD;
        }
        /* Anything else: stay here, keep scanning */
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_CMD:
        parser->cmd   = byte;
        parser->state = STATE_WAIT_LEN;
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_LEN:
        parser->data_len = byte;
        parser->data_idx = 0;

        if (parser->data_len > PACKET_MAX_DATA) {
            /* Invalid length — discard and resync */
            parser_init(parser);
            return PARSE_CRC_ERROR;
        }

        if (parser->data_len == 0) {
            /* No data bytes — jump straight to CRC */
            parser->state = STATE_WAIT_CRC;
        } else {
            parser->state = STATE_WAIT_DATA;
        }
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_DATA:
        parser->data[parser->data_idx] = byte;
        parser->data_idx++;

        if (parser->data_idx >= parser->data_len) {
            parser->state = STATE_WAIT_CRC;
        }
        return PARSE_IN_PROGRESS;

    case STATE_WAIT_CRC: {
        /*
         * Reconstruct the CRC input: CMD + LEN + DATA
         * Reuse a temp buffer to avoid recomputing from out_buf
         */
        uint8_t crc_buf[2 + PACKET_MAX_DATA];
        crc_buf[0] = parser->cmd;
        crc_buf[1] = parser->data_len;

        for (uint8_t i = 0; i < parser->data_len; i++)
            crc_buf[2 + i] = parser->data[i];

        uint8_t expected = crc8(crc_buf, 2 + parser->data_len);

        if (byte == expected) {
            /* Success — reset state but preserve cmd/data_len/data for caller */
            parser->state    = STATE_WAIT_SYNC;
            parser->data_idx = 0;
            return PARSE_COMPLETE;
        } else {
            /* CRC mismatch — full reset, corrupted data is useless */
            parser_init(parser);
            return PARSE_CRC_ERROR;
        }
    }

    default:
        /* Should never happen — reset */
        parser_init(parser);
        return PARSE_CRC_ERROR;
    }
}
