.syntax unified
.cpu cortex-m4
.thumb

NVIC_BASE = 0xe000e100
NVIC_ISER0 = NVIC_BASE

// Each ISERx registers contains the enable bits for 32 interrupts
// To enable some interrupt x, we must set bit (x mod 32) in NVIC_ISERy
// where y = x/32.
// The address of NVIC_ISERy is y * 4

// r0 = IRQ Number
.type EnableIRQ, %function
EnableIRQ:
    ldr r1, =NVIC_ISER0

    movs r2, 1
    and r3, r0, (32 - 1)    // calculate r0 mod 32 to find x
    lsls r2, r2, r3         // 1 << (x mod 32)

                            // now let's calculate y * 4
    lsrs r3, r0, 5          // right shift by 5 to divide by 32
    lsrs r3, r3, 2          // left shift by 2 to multiply by 4

    str r2, [r1, r3]        // enable the interrupt
    bx lr

.include "stm32g474re.inc"

TimerValue = 1500
TimerPrescaler = 1000

.data
TimerEvents: .byte 2

.bss
BlinkStep: .space 1

.text
.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:
	// initialize data and bss sections
	ldr r0, =_DataStart
	ldr r1, =_DataEnd
	ldr r2, =_DataLoad

	b 2f
1:	ldr r3, [r2], #4
	str r3, [r0], #4
2:	cmp r0, r1
	blo 1b

	ldr r0, =_BssStart
	ldr r1, =_BssEnd
	ldr r2, =0

	b 2f
1:	str r2, [r0], #4
2:	cmp r0, r1
	blo 1b
