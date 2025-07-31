/*
 * exti.c
 *
 *  Created on: July 18, 2025
 *      Author: dobao
 */

#include "exti.h"


/*
 * @brief	Enable the interrupt in the NVIC
 *
 * @param	irqNumber	IRQ number based on the vector table (0-85)
 */
void NVIC_enableIRQ(IRQn_Pos_t irqNumber){
	if(irqNumber > 85) return;

	uint8_t regIdx = irqNumber / 32; //Determine which ISER reg
	uint8_t bitPos = irqNumber % 32; //Determine bit position in that ISER reg

	NVIC_REG -> _ISER[regIdx] |= (SET << bitPos); //Set corresponding bit to enable IRQ
}



/*
 * @brief	Disable the interrupt in the NVIC
 *
 * @param 	irqNumber IRQ number based on the vector table (0-85)
 */
void NVIC_disableIRQ(IRQn_Pos_t irqNumber){
	if(irqNumber > 85) return;

	uint8_t regIdx = irqNumber / 32; //Determine which ICER reg
	uint8_t bitPos = irqNumber % 32; //Determine bit position in that ISER reg

	NVIC_REG -> _ICER[regIdx] |= (SET << bitPos); // Set corresponding bit to disable IRQ
}



/*
 * @brief	Set interrupt priority in NVIC
 *
 * @param	irqNumber	IRQ number (0-85)
 * @param	priority	Priority level (only upper 4 bits of 8-bit field are used)
 */
void NVIC_setPriority(IRQn_Pos_t irqNumber, uint8_t priority){
	if(irqNumber > 85) return;
	NVIC_REG -> _IPR[irqNumber] = (priority & 0x0F) << 4; //Top 4 bits only are used
}



/*
 * @brief	Configure the EXTI line and enable IRQ
 *
 * @param	bitPosition		EXTI line number (0-22)
 * @param	triggerMode		Trigger the interrupt based on rising/falling/bot edges
 * @param	irqNumber		Corresponding NVIC IRQ number to enable
 */
void EXTI_init(char bitPosition,
			   EXTI_Trigger_t triggerMode,
			   IRQn_Pos_t irqNumber){

	if(triggerMode == my_EXTI_TRIGGER_RISING || triggerMode == my_EXTI_TRIGGER_BOTH){
		writeEXTI(bitPosition, RTSR, SET); //Rising Edge Reg Enable
	}

	if(triggerMode == my_EXTI_TRIGGER_FALLING || triggerMode == my_EXTI_TRIGGER_BOTH){
		writeEXTI(bitPosition, FTSR, SET); //Falling Edge Reg Enable
	}

	writeEXTI(bitPosition, IMR, SET); //(write 1 = Unmask Interrupt) -> Enable EXTI line
	NVIC_enableIRQ(irqNumber); //Enable IRQ in NVIC
}



/*
 * @brief	Relocates the vector table to a new memory address in RAM
 * 			Useful for enabling dynamic interrupt vector updates at runtime
 *
 * @param	vectorTableOffsetAddr	A pointer to the new RAM location where the vector table will be copied
 */
static volatile uint32_t* globalVectorTableOffsetAddr;

void vectorTableOffset(volatile uint32_t* vectorTableOffsetAddr){
	//Pointer to the current vector base address (usually in flash)
	volatile uint32_t* originVectorTableAddr = (volatile uint32_t*)(*(volatile uint32_t*) VTOR_BASE_ADDR);

	//Copy the existing vector table entries to the new location
	for(int i = 0; i < 0x198; i++){
		*(vectorTableOffsetAddr + i) = *(originVectorTableAddr + i);
	}

	//Update the VTOR to point to the new vector table in RAM
	(*(volatile uint32_t*)VTOR_BASE_ADDR) = (uint32_t)vectorTableOffsetAddr;

	globalVectorTableOffsetAddr = vectorTableOffsetAddr;
}



/*
 * @brief	Registers a custom interrupt handler function at a specific vector table offset
 *
 * @param	function_id		Pointer to the user-defined interrupt handler function
 * @param	interruptAddr	Address of the vector table entry (from STM32 VT table, e.g. 0x58 for EXTI0)
 */
void user_IRQHandler(void (*function_id)(void), uint32_t interruptAddr){
	volatile void(**funcPointer)(void) = (volatile void(**)(void)) (globalVectorTableOffsetAddr + interruptAddr/4);
	*funcPointer = function_id;
}



/*
 * @brief	Write a value to a specific EXTI register bit
 *
 * @param	bitPosition		EXTI line (0-22, depending on MCU and availability)
 * @param	mode			Specifies which EXTI register to write to (IMR, EMR, FTSR, etc.)
 * @param	state			SET to enable/set bit, RESET to disable/clear bit
 *
 * @Note	EXTI_PR: Only SET is valid. Writing 1 clears the pending flag.
 * 			Writing 0 to EXTI_PR has no effect.
 */
void writeEXTI(uint8_t bitPosition, EXTI_Mode_t mode, FlagStatus state){
	if(!isValidEXTIBits(bitPosition)) return;

	volatile uint32_t* reg = NULL; //A pointer to store the address and access to the memory space at that address

	switch(mode){
		case IMR:	reg = &EXTI_REG -> IMR; break;
		case EMR: 	reg = &EXTI_REG -> EMR; break;
		case RTSR: 	reg = &EXTI_REG -> RTSR; break;
		case FTSR:	reg = &EXTI_REG -> FTSR; break;
		case SWIER:	reg = &EXTI_REG -> SWIER; break;
		case PR: 	reg = &EXTI_REG -> PR; break;
		default: return; //Invalid mode
	}

	uint32_t mask = 1 << bitPosition;

	if(mode == PR && state == SET){
		*reg = mask; //Clear the pending interrupt
	}
	else{
		if(state == SET){
			*reg |= mask; //Set bit
		}
		else *reg &= ~mask; //Clear bit
	}
}






