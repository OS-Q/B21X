
#include "stm32h7xx.h"
#define LEDPORT (GPIOB)
#define LED1 (0)
#define ENABLE_GPIO_CLOCK (RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN)
#define _MODER    MODER
#define GPIOMODER (GPIO_MODER_MODE0_0)


void ms_delay(int ms)
{
    while (ms-- > 0) {
        volatile int x=500;
        while (x-- > 0)
            __asm("nop");
    }
}


int main(void)
{
    ENABLE_GPIO_CLOCK;              // enable the clock to GPIO
    LEDPORT->_MODER |= GPIOMODER;   // set pins to be general purpose output
    for (;;) {
    ms_delay(500);
    LEDPORT->ODR ^= (1<<LED1);  // toggle diodes
    }

    return 0;
}
