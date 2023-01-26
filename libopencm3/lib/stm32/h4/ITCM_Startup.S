.syntax unified
.cpu cortex-m7
.thumb

.word _siitcm
.word _sitcm
.word _eitcm
.word _sisdram
.word _ssdram
.word _esdram

.section .text.ITCM_Startup
.global ITCM_Startup
.func ITCM_Startup
.type ITCM_Startup, %function
ITCM_Startup:
	/* copy the code for ITCM */
	movs  r1, #0
	b  LoopCopyITCMInit
CopyITCMInit:
	ldr  r3, =_siitcm
	ldr  r3, [r3, r1]
	str  r3, [r0, r1]
	adds r1, r1, #4

LoopCopyITCMInit:
	ldr  r0, =_sitcm
	ldr  r3, =_eitcm
	adds r2, r0, r1
	cmp  r2, r3
	bcc  CopyITCMInit

	bx	 lr
.endfunc
.size ITCM_Startup, .-ITCM_Startup

.section .text.SDRAM_Startup
.global SDRAM_Startup
.func SDRAM_Startup
.type SDRAM_Startup, %function
SDRAM_Startup:
	/* copy the code and data for SDRAM */
	movs  r1, #0
	b  LoopCopySDRAMInit
CopySDRAMInit:
	ldr  r3, =_sisdram
	ldr  r3, [r3, r1]
	str  r3, [r0, r1]
	adds r1, r1, #4

LoopCopySDRAMInit:
	ldr  r0, =_ssdram
	ldr  r3, =_esdram
	adds r2, r0, r1
	cmp  r2, r3
	bcc  CopySDRAMInit

	dsb
	dmb
	isb
	bx	 lr
.endfunc
.size SDRAM_Startup, .-SDRAM_Startup

/*
  FLASH ROM BIOS vector table, forwarded from SRAM BIOS vector table
*/
.section .BIOS_vectors, "a", %progbits
.type SDRAM_Entry, %function
.global SDRAM_Entry
	.balign 4
SDRAM_Entry:
	ldr		r3, =0x60000001
	bx		r3
.size SDRAM_Entry, .-SDRAM_Entry

.type BIOSn1, %function
.global BIOSn1
	.balign 4
BIOSn1:
	b.w		UART_Send
.size BIOSn1, .-BIOSn1

.type BIOSn2, %function
.global BIOSn2
	.balign 4
BIOSn2:
	b.w		UART_SendStr
.size BIOSn2, .-BIOSn2

.type BIOSn3, %function
.global BIOSn3
	.balign 4
BIOSn3:
	b.w		UART_GetString
.size BIOSn3, .-BIOSn3

/*
   SDRAM BIOS vector table
*/
.section .sdramtext.vectors, "a", %progbits
.type SDRAM_main, %function
.global SDRAM_main
	.balign 4
SDRAM_main:
	b.w	SDRAM_Main
.size SDRAM_main, .-SDRAM_main

.type SDRAM_UARTSend, %function
.global SDRAM_UARTSend
	.balign 4
SDRAM_UARTSend:
	ldr		r12, =0x0804029D
	bx		r12
.size SDRAM_UARTSend, .-SDRAM_UARTSend

.type SDRAM_UARTSendString, %function
.global SDRAM_UARTSendString
	.balign 4
SDRAM_UARTSendString:
	ldr		r12, =0x080402A1
	bx		r12
.size SDRAM_UARTSendString, .-SDRAM_UARTSendString

.type SDRAM_UARTGetString, %function
.global SDRAM_UARTGetString
	.balign 4
SDRAM_UARTGetString:
	ldr		r12, =0x080402A5
	bx		r12
.size SDRAM_UARTGetString, .-SDRAM_UARTGetString

/*
   just for now: test the entry to SDRAM code, later C-code has to have
   this SDRAM_Main() function
*/
	.section .sdramtext, "a", %progbits
.type SDRAM_Main, %function
.global SDRAM_Main
	.balign 4
SDRAM_Main:
	add	r0, #1
	bx	lr
.size SDRAM_Main, .-SDRAM_Main
