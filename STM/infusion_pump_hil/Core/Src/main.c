#include "main.h"
#include "protocol.h"

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);

uint8_t rx_byte;
static packet_parser_t rx_parser;

static volatile uint8_t pending_cmd_ready = 0;
static volatile uint8_t pending_cmd = 0;
static volatile uint8_t pending_data_len = 0;
static volatile uint8_t pending_data[PACKET_MAX_DATA];
static volatile uint8_t pending_crc_fault = 0;

static uint8_t motor_state = MOTOR_STATE_IDLE;
static uint32_t last_heartbeat_ms = 0;
static uint8_t last_button_level = 1;

static void set_motor_state(uint8_t state)
{
    motor_state = state;

    if (state == MOTOR_STATE_RUNNING) {
        last_heartbeat_ms = HAL_GetTick();
    }
}

static void update_led(uint32_t now_ms)
{
    static uint32_t last_blink_ms = 0;
    static GPIO_PinState blink_level = GPIO_PIN_RESET;

    if (motor_state == MOTOR_STATE_RUNNING) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        return;
    }

    if (motor_state == MOTOR_STATE_IDLE) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        return;
    }

    if ((now_ms - last_blink_ms) >= 200U) {
        last_blink_ms = now_ms;
        blink_level = (blink_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, blink_level);
    }
}

static void send_packet(uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    uint8_t buf[PACKET_MAX_SIZE];
    uint8_t len = build_packet(cmd, data, data_len, buf);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, buf, len, 100);
    }
}

static void send_fault_packet(void)
{
    send_packet(CMD_FAULT, NULL, 0);
}

static void send_heartbeat_if_due(uint32_t now_ms)
{
    if (motor_state != MOTOR_STATE_RUNNING) {
        return;
    }

    if ((now_ms - last_heartbeat_ms) >= 500U) {
        last_heartbeat_ms = now_ms;
        send_packet(CMD_HEARTBEAT, NULL, 0);
    }
}

static void check_fault_button(void)
{
    uint8_t current_level = (uint8_t)HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    if ((last_button_level == 1U) && (current_level == 0U)) {
        set_motor_state(MOTOR_STATE_FAULT);
        send_fault_packet();
    }

    last_button_level = current_level;
}

static void handle_packet(uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    (void)data;
    (void)data_len;

    switch (cmd) {
    case CMD_MOTOR_START:
        if (motor_state == MOTOR_STATE_FAULT) {
            send_fault_packet();
            break;
        }

        set_motor_state(MOTOR_STATE_RUNNING);
        send_packet(CMD_ACK, NULL, 0);
        break;

    case CMD_MOTOR_STOP:
        set_motor_state(MOTOR_STATE_IDLE);
        send_packet(CMD_ACK, NULL, 0);
        break;

    case CMD_STATUS_REQ: {
        uint8_t status = motor_state;
        send_packet(CMD_STATUS_RSP, &status, 1);
        break;
    }

    default:
        set_motor_state(MOTOR_STATE_FAULT);
        send_fault_packet();
        break;
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    parser_init(&rx_parser);
    set_motor_state(MOTOR_STATE_IDLE);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    last_heartbeat_ms = HAL_GetTick();
    last_button_level = (uint8_t)HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    while (1)
    {
        uint32_t now_ms = HAL_GetTick();

        if (pending_crc_fault) {
            pending_crc_fault = 0;
            set_motor_state(MOTOR_STATE_FAULT);
            send_fault_packet();
        }

        if (pending_cmd_ready) {
            uint8_t local_cmd = pending_cmd;
            uint8_t local_data_len = pending_data_len;
            uint8_t local_data[PACKET_MAX_DATA];

            for (uint8_t i = 0; i < local_data_len; i++) {
                local_data[i] = pending_data[i];
            }

            pending_cmd_ready = 0;
            handle_packet(local_cmd, local_data, local_data_len);
        }

        check_fault_button();
        update_led(now_ms);
        send_heartbeat_if_due(now_ms);

        HAL_Delay(10);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        int result = parse_byte(&rx_parser, rx_byte);

        if (result == PARSE_COMPLETE) {
            if (!pending_cmd_ready) {
                pending_cmd = rx_parser.cmd;
                pending_data_len = rx_parser.data_len;

                for (uint8_t i = 0; i < rx_parser.data_len; i++) {
                    pending_data[i] = rx_parser.data[i];
                }

                pending_cmd_ready = 1;
            }
        }
        else if (result == PARSE_CRC_ERROR) {
            pending_crc_fault = 1;
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* ================= CLOCK ================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/* ================= GPIO ================= */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PA5 = onboard LED LD2 */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* PC13 = user button B1, active low on Nucleo board */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* ================= USART1 ================= */
void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}
