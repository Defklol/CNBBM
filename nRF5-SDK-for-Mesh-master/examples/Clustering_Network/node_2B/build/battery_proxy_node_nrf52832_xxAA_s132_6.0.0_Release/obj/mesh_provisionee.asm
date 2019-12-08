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
	.file	"mesh_provisionee.c"
	.text
	.section	.text.provisionee_start,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	provisionee_start, %function
provisionee_start:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r1, .L2
	ldr	r0, .L2+4
	ldr	r1, [r1, #4]
	movs	r3, #3
	movs	r2, #0
	b	nrf_mesh_prov_listen
.L3:
	.align	2
.L2:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.size	provisionee_start, .-provisionee_start
	.section	.text.sd_state_evt_handler,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	sd_state_evt_handler, %function
sd_state_evt_handler:
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	ldr	r4, .L37
	ldrb	r1, [r4]	@ zero_extendqisi2
	sub	sp, sp, #20
	cbz	r1, .L4
	cmp	r0, #1
	beq	.L7
	cmp	r0, #3
	beq	.L8
.L4:
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, pc}
.L7:
	add	r5, sp, #16
	movs	r3, #0
	str	r3, [r5, #-8]!
	mov	r1, r5
	bl	nrf_sdh_ble_default_cfg_set
	cbz	r0, .L9
	bl	app_error_handler_bare
.L9:
	mov	r0, r5
	bl	nrf_sdh_ble_enable
	cbz	r0, .L10
	bl	app_error_handler_bare
.L10:
	add	r1, sp, #16
	movs	r3, #1
	str	r3, [r1, #-4]!
	add	r0, sp, #6
	bl	dsm_subnet_get_all
	cbz	r0, .L11
	bl	app_error_handler_bare
.L11:
	ldrh	r0, [sp, #6]
	bl	dsm_net_key_index_to_subnet_handle
	add	r1, sp, #5
	bl	dsm_subnet_kr_phase_get
	cbz	r0, .L12
	bl	app_error_handler_bare
.L12:
	bl	proxy_init
	ldrb	r1, [sp, #5]	@ zero_extendqisi2
	movs	r0, #0
	bl	proxy_node_id_enable
	bl	nrf_mesh_enable
	cbz	r0, .L13
	bl	app_error_handler_bare
.L13:
	movs	r3, #0
	strb	r3, [r4]
	ldr	r3, .L37+4
	ldr	r3, [r3]
	cmp	r3, #0
	beq	.L4
	blx	r3
	b	.L4
.L8:
	bl	nrf_sdh_enable_request
	cmp	r0, #0
	beq	.L4
	bl	app_error_handler_bare
	b	.L4
.L38:
	.align	2
.L37:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.size	sd_state_evt_handler, .-sd_state_evt_handler
	.section	.text.prov_evt_handler,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	prov_evt_handler, %function
prov_evt_handler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #5
	beq	.L41
	cmp	r3, #8
	beq	.L42
	cmp	r3, #2
	bne	.L39
	ldr	r3, .L61
	ldrb	r3, [r3]	@ zero_extendqisi2
	cbnz	r3, .L44
	pop	{r3, lr}
	b	provisionee_start
.L44:
	ldr	r3, .L61+4
	movs	r2, #1
	strb	r2, [r3]
	bl	nrf_mesh_disable
	cbz	r0, .L45
	bl	app_error_handler_bare
.L45:
	bl	nrf_sdh_disable_request
.L60:
	cbz	r0, .L39
	pop	{r3, lr}
	b	app_error_handler_bare
.L41:
	ldr	r3, .L61+8
	ldr	r0, .L61+12
	ldr	r1, [r3, #8]
	movs	r2, #16
	bl	nrf_mesh_prov_auth_data_provide
	b	.L60
.L42:
	ldrd	r1, r0, [r0, #8]
	bl	mesh_stack_provisioning_data_store
	cbz	r0, .L48
	bl	app_error_handler_bare
.L48:
	ldr	r3, .L61
	movs	r2, #1
	strb	r2, [r3]
.L39:
	pop	{r3, pc}
.L62:
	.align	2
.L61:
	.word	.LANCHOR3
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1
	.size	prov_evt_handler, .-prov_evt_handler
	.section	.text.mesh_provisionee_prov_start,"ax",%progbits
	.align	1
	.global	mesh_provisionee_prov_start
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv4-sp-d16
	.type	mesh_provisionee_prov_start, %function
mesh_provisionee_prov_start:
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	ldr	r2, .L66
	mov	r4, r0
	ldr	r1, [r2, #4]	@ unaligned
	ldr	r0, [r2]	@ unaligned
	sub	sp, sp, #24
	add	r3, sp, #12
	stmia	r3!, {r0, r1}
	ldrh	r1, [r2, #8]	@ unaligned
	ldrb	r2, [r2, #10]	@ zero_extendqisi2
	strh	r1, [r3]	@ unaligned
	strb	r2, [r3, #2]
	ldm	r4, {r0, r1, r2}
	ldr	r3, .L66+4
	stm	r3, {r0, r1, r2}
	cbz	r2, .L65
	ldr	r1, .L66+8
	ldr	r0, .L66+12
	bl	nrf_mesh_prov_generate_keys
	cbnz	r0, .L63
	ldr	r3, .L66+16
	str	r3, [sp]
	ldr	r2, .L66+8
	ldr	r1, .L66+12
	ldr	r0, .L66+20
	add	r3, sp, #12
	bl	nrf_mesh_prov_init
	cbnz	r0, .L63
	ldr	r0, .L66+24
	bl	nrf_mesh_prov_bearer_adv_interface_get
	mov	r1, r0
	ldr	r0, .L66+20
	bl	nrf_mesh_prov_bearer_add
	cbnz	r0, .L63
	ldr	r0, .L66+28
	bl	nrf_mesh_prov_bearer_gatt_init
	cbnz	r0, .L63
	ldr	r0, .L66+28
	bl	nrf_mesh_prov_bearer_gatt_interface_get
	mov	r1, r0
	ldr	r0, .L66+20
	bl	nrf_mesh_prov_bearer_add
	cbnz	r0, .L63
	bl	provisionee_start
.L63:
	add	sp, sp, #24
	@ sp needed
	pop	{r4, pc}
.L65:
	movs	r0, #7
	b	.L63
.L67:
	.align	2
.L66:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LANCHOR5
	.word	.LANCHOR6
	.word	prov_evt_handler
	.word	.LANCHOR1
	.word	.LANCHOR7
	.word	.LANCHOR8
	.size	mesh_provisionee_prov_start, .-mesh_provisionee_prov_start
	.section .rodata
	.set	.LANCHOR4,. + 0
.LC0:
	.byte	3
	.2byte	1
	.byte	0
	.byte	1
	.byte	0
	.2byte	0
	.byte	0
	.2byte	0
	.section	.bss.m_device_provisioned,"aw",%nobits
	.set	.LANCHOR3,. + 0
	.type	m_device_provisioned, %object
	.size	m_device_provisioned, 1
m_device_provisioned:
	.space	1
	.section	.bss.m_doing_gatt_reset,"aw",%nobits
	.set	.LANCHOR2,. + 0
	.type	m_doing_gatt_reset, %object
	.size	m_doing_gatt_reset, 1
m_doing_gatt_reset:
	.space	1
	.section	.bss.m_params,"aw",%nobits
	.align	2
	.set	.LANCHOR0,. + 0
	.type	m_params, %object
	.size	m_params, 12
m_params:
	.space	12
	.section	.bss.m_private_key,"aw",%nobits
	.set	.LANCHOR5,. + 0
	.type	m_private_key, %object
	.size	m_private_key, 32
m_private_key:
	.space	32
	.section	.bss.m_prov_bearer_adv,"aw",%nobits
	.align	2
	.set	.LANCHOR7,. + 0
	.type	m_prov_bearer_adv, %object
	.size	m_prov_bearer_adv, 456
m_prov_bearer_adv:
	.space	456
	.section	.bss.m_prov_bearer_gatt,"aw",%nobits
	.align	2
	.set	.LANCHOR8,. + 0
	.type	m_prov_bearer_gatt, %object
	.size	m_prov_bearer_gatt, 80
m_prov_bearer_gatt:
	.space	80
	.section	.bss.m_prov_ctx,"aw",%nobits
	.align	2
	.set	.LANCHOR1,. + 0
	.type	m_prov_ctx, %object
	.size	m_prov_ctx, 308
m_prov_ctx:
	.space	308
	.section	.bss.m_public_key,"aw",%nobits
	.set	.LANCHOR6,. + 0
	.type	m_public_key, %object
	.size	m_public_key, 64
m_public_key:
	.space	64
	.section	.sdh_state_observers1,"a",%progbits
	.align	2
	.type	m_sdh_req_obs, %object
	.size	m_sdh_req_obs, 8
m_sdh_req_obs:
	.word	sd_state_evt_handler
	.word	0
	.ident	"GCC: (GNU) 7.3.1 20180622 (release) [ARM/embedded-7-branch revision 261907]"
