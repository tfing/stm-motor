#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

void Delay(uint32_t nTime);

// Motor pin:
// 1:PB 1  BLU
// 2:PB 15 PIK
// 3:PB 14 YEL
// 4:PB 13 ORG

// Pin 1,2,3,4 = Bit 3,2,1,0
#define MOTOR_PINS 4
typedef struct MotorPin_s {
    uint8_t bit;
    uint16_t gpio_pin;
} MotorPin_t;

static MotorPin_t motor_pins[] = {
    {.bit = 0x8, .gpio_pin = GPIO_Pin_1},
    {.bit = 0x4, .gpio_pin = GPIO_Pin_15},
    {.bit = 0x2, .gpio_pin = GPIO_Pin_14},
    {.bit = 0x1, .gpio_pin = GPIO_Pin_13},
};

// counter clock wise steps
static uint8_t ccw[8] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xc, 0x8, 0x9};

void MotorPinActive(uint8_t steps)
{
    uint8_t i;
    for (i = 0; i < MOTOR_PINS; i++)
    {
        if (steps & motor_pins[i].bit)
        {
            GPIO_WriteBit(GPIOB, motor_pins[i].gpio_pin, Bit_SET);
        }
        else
        {
            GPIO_WriteBit(GPIOB, motor_pins[i].gpio_pin, Bit_RESET);
        }
    }
}

int main(void)
{
    (void)ccw;

    GPIO_InitTypeDef gpio_init;

    // Enable Peripheral clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Configure Pins
    int i;
    for (i = 0; i < MOTOR_PINS; i++)
    {
        gpio_init.GPIO_Pin = motor_pins[i].gpio_pin;
        gpio_init.GPIO_Mode = GPIO_Mode_OUT;
        gpio_init.GPIO_OType = GPIO_OType_PP;
        gpio_init.GPIO_Speed = GPIO_Low_Speed;
        GPIO_Init(GPIOB, &gpio_init);
    }

    // Configure SysTick Timer
    if (SysTick_Config(SystemCoreClock / 1000))
        while(1);

    while (1) {
        static int step = 8;

        MotorPinActive(ccw[step-1]);

        step -= 2;
        if (step <= 0)
            step = 8;

        Delay(20);
    }
}

// Timer Code
static uint32_t TimingDelay;

void Delay(uint32_t nTime)
{
    TimingDelay = nTime;
    while(TimingDelay != 0);
}

void SysTick_Handler(void)
{
    if (TimingDelay != 0)
        TimingDelay--;
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while(1);
}
#endif
