#include <cmsis_boot/stm32f10x.h>
#include <stm_lib/inc/stm32f10x_rcc.h>
#include <stm_lib/inc/stm32f10x_gpio.h>
#include <stm_lib/inc/stm32f10x_tim.h>
#include <stm_lib/inc/misc.h>

#include <stdbool.h>

#include <tic120.h>
#include <ds18b20.h>

void led_init();
void delay();
void one_second_counter_init();

volatile FlagStatus DataReady = RESET;
bool true_temp = false;

int main(void) {
    SystemInit();
    led_init();
    u8 td_cnt = ds18b20_init();

    tic120_init();
    tic120_clear();
    tic120_write_string_pos("Термометр", 0, 0);
    tic120_printf("датчики:%d", 0, 1, td_cnt);

    one_second_counter_init();

    while (1) {
        if (DataReady == SET) {
            DataReady = RESET;
            GPIOC->ODR ^= GPIO_Pin_13;
            if (true_temp) {
                u8 sign = 0;
                u16 temp = 0;
                ds18b20_result();
                for (u8 i = 0; i < td_cnt; i++) {
                    ds18b20_get_temp(&sign, &temp, i);
                    tic120_printf("%c%d.%s%cC ", 0, 2 + i, (sign ? '-' : '+'),
                            temp >> 4, t_frac[temp & 0xf], 0xb7);
                }
            }
            ds18b20_start();
            true_temp = true;
        }
    }
}

/**
 * @brief  Delay loop.
 */
void delay() {
    for(u32 i = 0; i < 1000000; i++) {
    }
}

/**
 * @brief  Initializes the LED.
 */
void led_init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOC->ODR |= GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
 * @brief   TIM3 timer set for one second interrupt.
 */
void one_second_counter_init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 7200 - 1;       // New clock = 10kHz.
    TIM3->ARR = 10000 - 1;      // Period = 1sec.
    TIM3->CR1 |= TIM_CR1_DIR;   // Used as downcounter.
    TIM3->CR1 |= TIM_CR1_OPM;   // Counter stops counting at the next update event.
    TIM3->CR2 |= TIM_CR2_MMS_0; // COUNTER_ENABLE signal to TIM1, used as trigger output (TRGO).
    TIM3->DIER |= TIM_DIER_UIE; // Enable interrupt.
    NVIC_EnableIRQ(TIM3_IRQn);  // Enable interrupt.
    TIM3->CR1 |= TIM_CR1_CEN;   // Start timer.
}

/*
 * @brief   TIM3 timer interrupt handler.
 */
void TIM3_IRQHandler() {
    TIM3->SR &= ~TIM_SR_UIF;  // Reset interrupt flag.
    DataReady = SET;          // Set access flag.
    TIM3->CR1 |= TIM_CR1_CEN; // To another circle.
}
