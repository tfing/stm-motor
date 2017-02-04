#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

void Delay(uint32_t nTime);

int main(void)
{
    GPIO_InitTypeDef gpio_init;

    // Enable Peripheral clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configure Pins
    gpio_init.GPIO_Pin = GPIO_Pin_5;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(GPIOA, &gpio_init);

    // Configure SysTick Timer
    if (SysTick_Config(SystemCoreClock / 1000))
        while(1);


    while (1) {
        static int ledval = 0;
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, (ledval) ? Bit_SET : Bit_RESET);
        ledval = 1 - ledval;

        Delay(250);
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
