#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_exti.h>
#include <misc.h>

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

// ascending  : rotate counterclockwise
// descending : rotate clockwise
static uint8_t ccw[8] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xc, 0x8, 0x9};
static int rotate_dir = 0;

void MotorPinActive(uint8_t active_dir)
{
    uint8_t i;
    for (i = 0; i < MOTOR_PINS; i++)
    {
        if (active_dir & motor_pins[i].bit)
        {
            GPIO_WriteBit(GPIOB, motor_pins[i].gpio_pin, Bit_SET);
        }
        else
        {
            GPIO_WriteBit(GPIOB, motor_pins[i].gpio_pin, Bit_RESET);
        }
    }
}

static int units;
void MotorUnitSteps(uint8_t units)
{
    // gear:1/64, core:angle/step = 11.25 = 32 steps/cycle
    uint8_t core_steps_per_cycle = 32;
    //uint8_t inverse_reduction_ratio = 64;
    uint16_t total_cycles = core_steps_per_cycle * units;
    uint16_t i, dir=1;
    for (i = 0; i < total_cycles; i++)
    {
        MotorPinActive(ccw[dir]);
        dir+=2;
        if (dir > 7)
            dir = 1;

        Delay(10);
    }
}

int main(void)
{
    (void)ccw;

    GPIO_InitTypeDef gpio_init;
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    // Enable Peripheral clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Configure Pins
    // Led
    gpio_init.GPIO_Pin = GPIO_Pin_5;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(GPIOA, &gpio_init);

    // Motor ctrl
    int i;
    for (i = 0; i < MOTOR_PINS; i++)
    {
        gpio_init.GPIO_Pin = motor_pins[i].gpio_pin;
        gpio_init.GPIO_Mode = GPIO_Mode_OUT;
        gpio_init.GPIO_OType = GPIO_OType_PP;
        gpio_init.GPIO_Speed = GPIO_Low_Speed;
        GPIO_Init(GPIOB, &gpio_init);
    }

    // User Button
    gpio_init.GPIO_Pin = GPIO_Pin_13;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_Speed = GPIO_Low_Speed;
    gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio_init);

    // Connect EXTI13 with PC13
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    // Configure EXTI Line 13
    exti_init.EXTI_Line = EXTI_Line13;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising;
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    nvic_init.NVIC_IRQChannel = EXTI15_10_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0x0F;
    nvic_init.NVIC_IRQChannelSubPriority = 0x0F;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    // Configure SysTick Timer
    if (SysTick_Config(SystemCoreClock / 1000))
        while(1);

    while (1) {
#if 0
        static int step = 8;
        //uint8_t btn = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
        MotorPinActive(ccw[step-1]);
        if (rotate_dir == 0)
        {
            // cw
            step -= 2;
            if (step < 2)
                step = 8;

        }
        else
        {
            // ccw
            step += 2;
            if (step > 8)
                step = 2;
        }
#endif
        if (units > 0)
        {
            MotorUnitSteps(1);
            units--;
        }

        Delay(50);
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

void ToggleLed(void)
{
    static int ledval = 0;
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, (ledval) ? Bit_SET : Bit_RESET);
    ledval = 1 - ledval;
}


void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line13) == SET)
    {
        ToggleLed();
        rotate_dir = 1 - rotate_dir;
        units += 64;
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while(1);
}
#endif
