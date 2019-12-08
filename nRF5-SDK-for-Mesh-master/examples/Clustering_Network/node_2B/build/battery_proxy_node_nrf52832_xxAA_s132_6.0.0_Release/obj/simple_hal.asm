	.cpu cortex-m4
	.eabi_attribute 27, 1
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 4
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"simple_hal.c"
	.text
	.section	.text.led_timeout_handler,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	led_timeout_handler, %function
led_timeout_handler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r4, .L4
	ldr	r0, [r4]
	cbnz	r0, .L2
	bl	app_error_handler_bare
.L2:
	mov	r2, #1342177280
	ldr	r1, .L4+4
	ldr	r3, [r2, #1284]
	ldr	r1, [r1]
	eors	r3, r3, r1
	str	r3, [r2, #1284]
	ldr	r3, [r4]
	subs	r3, r3, #1
	str	r3, [r4]
	cbnz	r3, .L1
	ldr	r0, .L4+8
	pop	{r4, lr}
	b	app_timer_stop
.L1:
	pop	{r4, pc}
.L5:
	.align	2
.L4:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.size	led_timeout_handler, .-led_timeout_handler
	.section	.text.hal_led_pin_get,"ax",%progbits
	.align	1
	.global	hal_led_pin_get
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	hal_led_pin_get, %function
hal_led_pin_get:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #1342177280
	ldr	r2, [r3, #1284]
	movs	r3, #1
	lsl	r0, r3, r0
	tst	r0, r2
	ite	eq
	moveq	r0, r3
	movne	r0, #0
	bx	lr
	.size	hal_led_pin_get, .-hal_led_pin_get
	.section	.text.hal_led_pin_set,"ax",%progbits
	.align	1
	.global	hal_led_pin_set
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	hal_led_pin_set, %function
hal_led_pin_set:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r3, #1
	lsl	r0, r3, r0
	mov	r3, #1342177280
	cbz	r1, .L8
	str	r0, [r3, #1292]
	bx	lr
.L8:
	str	r0, [r3, #1288]
	bx	lr
	.size	hal_led_pin_set, .-hal_led_pin_set
	.section	.text.hal_led_mask_set,"ax",%progbits
	.align	1
	.global	hal_led_mask_set
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	hal_led_mask_set, %function
hal_led_mask_set:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #1342177280
	cbz	r1, .L11
	str	r0, [r3, #1292]
	bx	lr
.L11:
	str	r0, [r3, #1288]
	bx	lr
	.size	hal_led_mask_set, .-hal_led_mask_set
	.global	__aeabi_uldivmod
	.global	__aeabi_ldivmod
	.section	.text.hal_led_blink_ms,"ax",%progbits
	.align	1
	.global	hal_led_blink_ms
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	hal_led_blink_ms, %function
hal_led_blink_ms:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r6, r7, lr}
	cbz	r2, .L13
	cmp	r1, #19
	bls	.L13
	ldr	r4, .L17
	ldr	r3, .L17+4
	str	r0, [r4]
	lsls	r2, r2, #1
	mov	r0, #32768
	mov	r6, #500
	movs	r7, #0
	subs	r2, r2, #1
	umlal	r6, r7, r0, r1
	str	r2, [r3]
	mov	r0, r6
	mov	r2, #1000
	movs	r3, #0
	mov	r1, r7
	bl	__aeabi_uldivmod
	movs	r2, #0
	mov	r1, r0
	ldr	r0, .L17+8
	bl	app_timer_start
	cbnz	r0, .L13
	mov	r2, #1342177280
	ldr	r1, [r4]
	ldr	r3, [r2, #1284]
	bic	r3, r3, r1
	str	r3, [r2, #1284]
.L13:
	pop	{r4, r6, r7, pc}
.L18:
	.align	2
.L17:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.size	hal_led_blink_ms, .-hal_led_blink_ms
	.section	.text.hal_leds_init,"ax",%progbits
	.align	1
	.global	hal_leds_init
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	hal_leds_init, %function
hal_leds_init:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	mov	r3, #1342177280
	movs	r2, #3
	mov	r1, #131072
	str	r2, [r3, #1860]
	str	r1, [r3, #1288]
	mov	r1, #262144
	str	r2, [r3, #1864]
	str	r1, [r3, #1288]
	mov	r1, #524288
	str	r2, [r3, #1868]
	str	r1, [r3, #1288]
	str	r2, [r3, #1872]
	mov	r2, #1048576
	str	r2, [r3, #1288]
	movs	r1, #1
	ldr	r2, .L21
	ldr	r0, .L21+4
	bl	app_timer_create
	cbz	r0, .L19
	pop	{r3, lr}
	b	app_error_handler_bare
.L19:
	pop	{r3, pc}
.L22:
	.align	2
.L21:
	.word	led_timeout_handler
	.word	.LANCHOR3
	.size	hal_leds_init, .-hal_leds_init
	.section	.text.hal_buttons_init,"ax",%progbits
	.align	1
	.global	hal_buttons_init
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	hal_buttons_init, %function
hal_buttons_init:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cbz	r0, .L25
	ldr	r3, .L26
	ldr	r2, .L26+4
	str	r0, [r3]
	mov	r3, #1342177280
	movs	r0, #0
	str	r2, [r3, #1844]
	str	r2, [r3, #1848]
	str	r2, [r3, #1852]
	str	r2, [r3, #1856]
	ldr	r3, .L26+8
	mov	r2, #-2147483648
	str	r2, [r3, #772]
	str	r0, [r3, #380]
	ldr	r3, .L26+12
	movs	r2, #224
	strb	r2, [r3, #774]
	movs	r2, #64
	str	r2, [r3]
	bx	lr
.L25:
	movs	r0, #14
	bx	lr
.L27:
	.align	2
.L26:
	.word	.LANCHOR4
	.word	196620
	.word	1073766400
	.word	-536813312
	.size	hal_buttons_init, .-hal_buttons_init
	.section	.text.GPIOTE_IRQHandler,"ax",%progbits
	.align	1
	.global	GPIOTE_IRQHandler
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	GPIOTE_IRQHandler, %function
GPIOTE_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L36
	push	{r4, r5, r6, lr}
	movs	r4, #0
	ldr	r6, .L36+4
	ldr	r5, .L36+8
	str	r4, [r3, #380]
.L31:
	mov	r3, #1342177280
	ldrb	r1, [r4, r6]	@ zero_extendqisi2
	ldr	r2, [r3, #1296]
	movs	r3, #1
	lsls	r3, r3, r1
	bics	r3, r3, r2
	beq	.L29
	ldr	r1, .L36+12
	ldr	r3, [r5]
	ldr	r2, [r1, #1284]
	subs	r0, r3, r2
	it	mi
	submi	r0, r2, r3
	movw	r3, #13107
	cmp	r0, r3
	bls	.L29
	ldr	r3, [r1, #1284]
	str	r3, [r5]
	ldr	r3, .L36+16
	mov	r0, r4
	ldr	r3, [r3]
	blx	r3
.L29:
	adds	r4, r4, #1
	cmp	r4, #4
	bne	.L31
	pop	{r4, r5, r6, pc}
.L37:
	.align	2
.L36:
	.word	1073766400
	.word	.LANCHOR5
	.word	.LANCHOR6
	.word	1073786880
	.word	.LANCHOR4
	.size	GPIOTE_IRQHandler, .-GPIOTE_IRQHandler
	.section	.bss.m_blink_count,"aw",%nobits
	.align	2
	.set	.LANCHOR0,. + 0
	.type	m_blink_count, %object
	.size	m_blink_count, 4
m_blink_count:
	.space	4
	.section	.bss.m_blink_mask,"aw",%nobits
	.align	2
	.set	.LANCHOR1,. + 0
	.type	m_blink_mask, %object
	.size	m_blink_mask, 4
m_blink_mask:
	.space	4
	.section	.bss.m_blink_timer_data,"aw",%nobits
	.align	2
	.set	.LANCHOR2,. + 0
	.type	m_blink_timer_data, %object
	.size	m_blink_timer_data, 32
m_blink_timer_data:
	.space	32
	.section	.bss.m_button_handler_cb,"aw",%nobits
	.align	2
	.set	.LANCHOR4,. + 0
	.type	m_button_handler_cb, %object
	.size	m_button_handler_cb, 4
m_button_handler_cb:
	.space	4
	.section	.bss.m_last_button_press,"aw",%nobits
	.align	2
	.set	.LANCHOR6,. + 0
	.type	m_last_button_press, %object
	.size	m_last_button_press, 4
m_last_button_press:
	.space	4
	.section	.rodata.m_blink_timer,"a",%progbits
	.align	2
	.set	.LANCHOR3,. + 0
	.type	m_blink_timer, %object
	.size	m_blink_timer, 4
m_blink_timer:
	.word	m_blink_timer_data
	.section	.rodata.m_buttons_list,"a",%progbits
	.set	.LANCHOR5,. + 0
	.type	m_buttons_list, %object
	.size	m_buttons_list, 4
m_buttons_list:
	.byte	13
	.byte	14
	.byte	15
	.byte	16
	.ident	"GCC: (GNU) 7.3.1 20180622 (release) [ARM/embedded-7-branch revision 261907]"
