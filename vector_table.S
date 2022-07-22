.syntax unified
.cpu cortex-m4
.thumb

.macro defisr name
    .global \name
    .weak \name
    .thumb_set \name, Default_Handler
    .word \name
.endm

.global VectorTable
.section .VectorTable, "a"
.type VectorTable, %object
VectorTable:
.word _StackEnd
/*** Cortex-M4 Processor Exceptions Numbers ***/
defisr Reset_Handler
defisr NonMaskableInt_Handler
defisr HardFault_Handler
defisr MemoryManagement_Handler
defisr BusFault_Handler
defisr UsageFault_Handler
.space 4 * 3
defisr SVCall_Handler
defisr DebugMonitor_Handler
.space 4
defisr PendSV_Handler
defisr SysTick_Handler
/*** STM32-specific Interrupt Numbers ***/
defisr WWDG_Handler
defisr PVD_PVM_Handler
defisr RTC_TAMP_LSECSS_Handler
defisr RTC_WKUP_Handler
defisr FLASH_Handler
defisr RCC_Handler
defisr EXTI0_Handler
defisr EXTI1_Handler
defisr EXTI2_Handler
defisr EXTI3_Handler
defisr EXTI4_Handler
defisr DMA1_Channel1_Handler
defisr DMA1_Channel2_Handler
defisr DMA1_Channel3_Handler
defisr DMA1_Channel4_Handler
defisr DMA1_Channel5_Handler
defisr DMA1_Channel6_Handler
defisr DMA1_Channel7_Handler
defisr ADC1_2_Handler
defisr USB_HP_Handler
defisr USB_LP_Handler
defisr FDCAN1_IT0_Handler
defisr FDCAN1_IT1_Handler
defisr EXTI9_5_Handler
defisr TIM1_BRK_TIM15_Handler
defisr TIM1_UP_TIM16_Handler
defisr TIM1_TRG_COM_TIM17_Handler
defisr TIM1_CC_Handler
defisr TIM2_Handler
defisr TIM3_Handler
defisr TIM4_Handler
defisr I2C1_EV_Handler
defisr I2C1_ER_Handler
defisr I2C2_EV_Handler
defisr I2C2_ER_Handler
defisr SPI1_Handler
defisr SPI2_Handler
defisr USART1_Handler
defisr USART2_Handler
defisr USART3_Handler
defisr EXTI15_10_Handler
defisr RTC_Alarm_Handler
defisr USBWakeUp_Handler
defisr TIM8_BRK_Handler
defisr TIM8_UP_Handler
defisr TIM8_TRG_COM_Handler
defisr TIM8_CC_Handler
defisr ADC3_Handler
defisr FMC_Handler
defisr LPTIM1_Handler
defisr TIM5_Handler
defisr SPI3_Handler
defisr UART4_Handler
defisr UART5_Handler
defisr TIM6_DAC_Handler
defisr TIM7_DAC_Handler
defisr DMA2_Channel1_Handler
defisr DMA2_Channel2_Handler
defisr DMA2_Channel3_Handler
defisr DMA2_Channel4_Handler
defisr DMA2_Channel5_Handler
defisr ADC4_Handler
defisr ADC5_Handler
defisr UCPD1_Handler
defisr COMP1_2_3_Handler
defisr COMP4_5_6_Handler
defisr COMP7_Handler
defisr HRTIM1_Master_Handler
defisr HRTIM1_TIMA_Handler
defisr HRTIM1_TIMB_Handler
defisr HRTIM1_TIMC_Handler
defisr HRTIM1_TIMD_Handler
defisr HRTIM1_TIME_Handler
defisr HRTIM1_FLT_Handler
defisr HRTIM1_TIMF_Handler
defisr CRS_Handler
defisr SAI1_Handler
defisr TIM20_BRK_Handler
defisr TIM20_UP_Handler
defisr TIM20_TRG_COM_Handler
defisr TIM20_CC_Handler
defisr FPU_Handler
defisr I2C4_EV_Handler
defisr I2C4_ER_Handler
defisr SPI4_Handler
defisr FDCAN2_IT0_Handler
defisr FDCAN2_IT1_Handler
defisr FDCAN3_IT0_Handler
defisr FDCAN3_IT1_Handler
defisr RNG_Handler
defisr LPUART1_Handler
defisr I2C3_EV_Handler
defisr I2C3_ER_Handler
defisr DMAMUX_OVR_Handler
defisr QUADSPI_Handler
defisr DMA1_Channel8_Handler
defisr DMA2_Channel6_Handler
defisr DMA2_Channel7_Handler
defisr DMA2_Channel8_Handler
defisr CORDIC_Handler
defisr FMAC_Handler

.text

.type Default_Handler, %function
.global Default_Handler
Default_Handler:
    bkpt
    b.n Default_Handler
