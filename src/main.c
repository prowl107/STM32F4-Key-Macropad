#include <stdio.h>
#include <stdint.h>
#include <STM32F4xx_GPIO.h>

void configure_matrix_rows(GPIO_Handle_t *matrix);
void configure_matrix_colums(GPIO_Handle_t *matrix);
void configure_led_pins();
uint8_t find_key_row(uint8_t columnNum);

int main()
{

    GPIO_Handle_t keyswitches;
    //Configure Matrix rows

    configure_matrix_rows(&keyswitches);
    configure_matrix_colums(&keyswitches);
    configure_led_pins();

    while (1)
    {
        /**
         * Column 1
         */
        if (GPIO_read_input_pin(GPIOD, 4) == 0)
        {
            find_key_row(1);
        }

        /**
         * Column 2
         */
        if (GPIO_read_input_pin(GPIOD, 5) == 0)
        {
            find_key_row(1);
        }

        /**
         * Column 3
         */

        /**
         * Column 4
         */
    }
}

void configure_matrix_rows(GPIO_Handle_t *matrix)
{
    matrix->pGPIOx = GPIOC;
    matrix->PinConfig.GPIO_PinMode = INPUT;
    // matrix->PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    matrix->PinConfig.GPIO_PinPuPdControll = GPIO_PIN_PU;
    matrix->PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //PC8, PC9, PC10, PC11
    for (uint8_t i = 8; i < 12; i++)
    {
        matrix->PinConfig.GPIO_PinNumber = i;
        GPIO_init(matrix);
    }

    return;
}

void configure_matrix_colums(GPIO_Handle_t *matrix)
{
    matrix->pGPIOx = GPIOD;
    matrix->PinConfig.GPIO_PinMode = INPUT;
    // matrix->PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    matrix->PinConfig.GPIO_PinPuPdControll = GPIO_PIN_PU;
    matrix->PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //PD4, PD5, PD6, PD7
    for (uint8_t i = 4; i < 8; i++)
    {
        matrix->PinConfig.GPIO_PinNumber = i;
        GPIO_init(matrix);
    }
}

void configure_led_pins()
{
    GPIO_Handle_t LEDpins;
    LEDpins.pGPIOx = GPIOE;
    LEDpins.PinConfig.GPIO_PinMode = OUTPUT;
    LEDpins.PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    LEDpins.PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    for (uint8_t i = 3; i < 7; i++)
    {
        LEDpins.PinConfig.GPIO_PinNumber = i;
        GPIO_init(&LEDpins);
    }
}

uint8_t find_key_row(uint8_t columnNum)
{
    //ROW 1
    if (GPIO_read_input_pin(GPIOC, 8) == 0)
    {
        GPIO_toggle_output_pin(GPIOE, 4);

        while (GPIO_read_input_pin(GPIOC, 8) == 0)
            ;

        GPIO_toggle_output_pin(GPIOE, 4);
    }

    //ROW 2
    if (GPIO_read_input_pin(GPIOC, 9) == 0)
    {
        GPIO_toggle_output_pin(GPIOE, 5);

        while (GPIO_read_input_pin(GPIOC, 9) == 0)
            ;

        GPIO_toggle_output_pin(GPIOE, 5);
    }

    //ROW 3
    if (GPIO_read_input_pin(GPIOC, 10) == 0)
    {
        GPIO_toggle_output_pin(GPIOE, 6);

        while (GPIO_read_input_pin(GPIOC, 10) == 0)
            ;

        GPIO_toggle_output_pin(GPIOE, 6);
    }

    //ROW 4
    if (GPIO_read_input_pin(GPIOC, 11) == 0)
    {
        GPIO_toggle_output_pin(GPIOE, 3);

        while (GPIO_read_input_pin(GPIOC, 11) == 0)
            ;

        GPIO_toggle_output_pin(GPIOE, 3);
    }
}