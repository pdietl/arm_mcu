.syntax unified
.cpu cortex-m4
.thumb

RCC_BASE    = 0x40021000
RCC_AHB2ENR = RCC_BASE + 0x4c
GPIOA_EN    = 1

GPIOA_BASE  = 0x48000000
GPIOA_MODER = GPIOA_BASE
GPIO_MODE_OUTPUT = 0b01
GPIOA_PORT5_SHIFT = 5 * 2 // 2 bits per mode setting
GPIOA_PORT5_MASK = ~(0b11 << (5 * 2)) // 2 bits per mode setting

GPIOA_ODR = GPIOA_BASE + 0x14

DELAY = (1024 * 1024 * 16 / 8)

.word 0x2000400
.word 0x080000ed
.space 0xe4

_start:
ldr r0, =0xe0001000
ldr r1, [r0]
and r1, 0x40000000
str r1, [r0]    // disable everything

ldr r0, =0xe0001004
ldr r1, =0
str r1, [r0]    // clear cycle counter




bkpt

// Enable GPIOA peripheral clock
ldr r0, =RCC_AHB2ENR
ldr r1, =GPIOA_EN
str r1, [r0]

// Setup GPIO PA5 as output
ldr r0, =GPIOA_MODER
ldr r1, [r0]
and r1, GPIOA_PORT5_MASK
orr r1, GPIO_MODE_OUTPUT << GPIOA_PORT5_SHIFT
str r1, [r0]

ldr r0, =GPIOA_ODR
ldr r1, =1 << 5
toggle_loop:
    str r1, [r0]

    ldr r2, =DELAY
delay:
    subs r2, r2, 1
    bne delay

    ldr r1, =0
    str r1, [r0]
    ldr r1, =1 << 5

    ldr r2, =DELAY
delay2:
    subs r2, r2, 1
    bne delay2

    b toggle_loop

b .
