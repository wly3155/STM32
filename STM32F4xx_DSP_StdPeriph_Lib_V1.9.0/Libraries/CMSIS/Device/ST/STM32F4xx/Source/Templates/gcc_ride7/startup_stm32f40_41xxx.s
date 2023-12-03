/**
  ******************************************************************************
  * @file      startup_stm32f40_41xxx.s
  * @author    MCD Application Team
  * @version   V1.8.1
  * @date      27-January-2022
  * @brief     STM32F40xxx/41xxx Devices vector table for RIDE7 toolchain.
  *            Same as startup_stm32f40xx.s and maintained for legacy purpose             
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Configure the clock system and the external SRAM mounted on 
  *                  STM324xG-EVAL board to be used as data memory (optional, 
  *                  to be enabled by user)
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
    
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

.section  .text.Reset_Handler
.weak  Reset_Handler
.type  Reset_Handler, %function
Reset_Handler:  

/* Copy the data segment initializers from flash to SRAM */  
	movs r1, #0
	b LoopCopyDataInit

CopyDataInit:
	ldr r3, =_sidata
	ldr r3, [r3, r1]
	str r3, [r0, r1]
	adds r1, r1, #4
    
LoopCopyDataInit:
	ldr r0, =_sdata
	ldr r3, =_edata
	adds r2, r0, r1
	cmp r2, r3
	bcc CopyDataInit
	ldr r2, =_sbss
	b LoopFillZerobss

/* Zero fill the bss segment. */  
FillZerobss:
	movs r3, #0
	str r3, [r2], #4
    
LoopFillZerobss:
	ldr r3, = _ebss
	cmp r2, r3
	bcc FillZerobss

/* Call the clock system intitialization function.*/
	bl SystemInit   
/* Call the application's entry point.*/
	bl main
	bx lr    
.size  Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an 
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None     
 * @retval None       
*/
.section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b Infinite_Loop
.size  Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
* 
*******************************************************************************/
.section  .isr_vector,"a",%progbits
.type  g_pfnVectors, %object
.size  g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
	.word _kstack_start
	.word Reset_Handler
	.word NMI_Handler
	.word HardFault_Handler
	.word MemManage_Handler
	.word BusFault_Handler
	.word UsageFault_Handler
	.word 0
	.word 0
	.word 0
	.word 0
	.word SVC_Handler
	.word DebugMon_Handler
	.word 0
	.word PendSV_Handler
	.word SysTick_Handler

	/* External Interrupts */
	.word irq_dispatch /* Window WatchDog              */
	.word irq_dispatch /* PVD through EXTI Line detection */
	.word irq_dispatch /* Tamper and TimeStamps through the EXTI line */
	.word irq_dispatch /* RTC Wakeup through the EXTI line */
	.word irq_dispatch /* FLASH                        */
	.word irq_dispatch /* RCC                          */ 
	.word irq_dispatch /* EXTI Line0                   */
	.word irq_dispatch /* EXTI Line1                   */
	.word irq_dispatch /* EXTI Line2                   */
	.word irq_dispatch /* EXTI Line3                   */
	.word irq_dispatch /* EXTI Line4                   */
	.word irq_dispatch /* DMA1 Stream 0                */
	.word irq_dispatch /* DMA1 Stream 1                */
	.word irq_dispatch /* DMA1 Stream 2                */
	.word irq_dispatch /* DMA1 Stream 3                */
	.word irq_dispatch /* DMA1 Stream 4                */
	.word irq_dispatch /* DMA1 Stream 5                */
	.word irq_dispatch /* DMA1 Stream 6                */
	.word irq_dispatch /* ADC1, ADC2 and ADC3s         */
	.word irq_dispatch /* CAN1 TX                      */
	.word irq_dispatch /* CAN1 RX0                     */
	.word irq_dispatch /* CAN1 RX1                     */
	.word irq_dispatch /* CAN1 SCE                     */
	.word irq_dispatch /* External Line[9:5]s          */
	.word irq_dispatch /* TIM1 Break and TIM9          */
	.word irq_dispatch /* TIM1 Update and TIM10        */
	.word irq_dispatch /* TIM1 Trigger and Commutation and TIM11 */
	.word irq_dispatch /* TIM1 Capture Compare         */
	.word irq_dispatch /* TIM2                         */
	.word irq_dispatch /* TIM3                         */
	.word irq_dispatch /* TIM4                         */
	.word irq_dispatch /* I2C1 Event                   */
	.word irq_dispatch /* I2C1 Error                   */
	.word irq_dispatch /* I2C2 Event                   */
	.word irq_dispatch /* I2C2 Error                   */
	.word irq_dispatch /* SPI1                         */
	.word irq_dispatch /* SPI2                         */
	.word irq_dispatch /* USART1                       */
	.word irq_dispatch /* USART2                       */
	.word irq_dispatch /* USART3                       */
	.word irq_dispatch /* External Line[15:10]s        */
	.word irq_dispatch /* RTC Alarm (A and B) through EXTI Line */
	.word irq_dispatch /* USB OTG FS Wakeup through EXTI line */
	.word irq_dispatch /* TIM8 Break and TIM12         */
	.word irq_dispatch /* TIM8 Update and TIM13        */
	.word irq_dispatch /* TIM8 Trigger and Commutation and TIM14 */
	.word irq_dispatch /* TIM8 Capture Compare         */
	.word irq_dispatch /* DMA1 Stream7                 */
	.word irq_dispatch /* FSMC                         */
	.word irq_dispatch /* SDIO                         */
	.word irq_dispatch /* TIM5                         */
	.word irq_dispatch /* SPI3                         */
	.word irq_dispatch /* UART4                        */
	.word irq_dispatch /* UART5                        */
	.word irq_dispatch /* TIM6 and DAC1&2 underrun errors */
	.word irq_dispatch /* TIM7                         */
	.word irq_dispatch /* DMA2 Stream 0                */
	.word irq_dispatch /* DMA2 Stream 1                */
	.word irq_dispatch /* DMA2 Stream 2                */
	.word irq_dispatch /* DMA2 Stream 3                */
	.word irq_dispatch /* DMA2 Stream 4                */
	.word irq_dispatch /* Ethernet                     */
	.word irq_dispatch /* Ethernet Wakeup through EXTI line */
	.word irq_dispatch /* CAN2 TX                      */
	.word irq_dispatch /* CAN2 RX0                     */
	.word irq_dispatch /* CAN2 RX1                     */
	.word irq_dispatch /* CAN2 SCE                     */
	.word irq_dispatch /* USB OTG FS                   */
	.word irq_dispatch /* DMA2 Stream 5                */
	.word irq_dispatch /* DMA2 Stream 6                */
	.word irq_dispatch /* DMA2 Stream 7                */
	.word irq_dispatch /* USART6                       */
	.word irq_dispatch /* I2C3 event                   */
	.word irq_dispatch /* I2C3 error                   */
	.word irq_dispatch /* USB OTG HS End Point 1 Out   */
	.word irq_dispatch /* USB OTG HS End Point 1 In    */
	.word irq_dispatch /* USB OTG HS Wakeup through EXTI */
	.word irq_dispatch /* USB OTG HS                   */
	.word irq_dispatch /* DCMI                         */
	.word irq_dispatch /* CRYP crypto                  */
	.word irq_dispatch /* Hash and Rng                 */
	.word irq_dispatch /* FPU                          */
