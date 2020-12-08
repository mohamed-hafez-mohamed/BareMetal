/*******************************************************************************
* Title                 :   startup code 
* Filename              :   stm32f10x_startup.c
* Author                :   Mohamed Hafez
* Origin Date           :   11/11/2011
* Version               :   1.0.0
* Compiler              :   TODO: COMPILER GOES HERE
* Target                :   TODO: MCU GOES HERE
* Notes                 :   None 
*
*****************************************************************************/
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author          Description 
*  11/11/20   1.0.0   Mohamed Hafez   Initial Release.
*
*******************************************************************************/
/** @file DRIVER_program.c
 *  @brief This is the source file for TODO: WHAT DO I DO? 
 */
/******************************************************************************
* Includes
*******************************************************************************/	

#include "STD_TYPES.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define SRAM_START_ADDRESS            0x20000000U
#define SRAM_SIZE                     (20 * 1024)                             //20 KB
#define SRAM_END_ADDRESS              ((SRAM_START_ADDRESS) + (SRAM_SIZE))
#define STACK_START_ADDRESS           SRAM_END_ADDRESS                        //Full descending stack
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/                     
// Prototype of main
int main(void);
// Instruct compiler to make rhese
void Reset_Handler(void);
void NMI_Handler                       (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler                       (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler   				      (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler  				      (void) __attribute__ ((weak, alias("Default_Handler")));
void WWDG_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler 				         (void) __attribute__ ((weak, alias("Default_Handler")));             
void TAMP_STAMP_IRQHandler 			   (void) __attribute__ ((weak, alias("Default_Handler")));      
void RTC_WKUP_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));                               
void RCC_IRQHandler 				         (void) __attribute__ ((weak, alias("Default_Handler")));             
void EXTI0_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));           
void EXTI1_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));           
void EXTI2_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));           
void EXTI3_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));           
void EXTI4_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));           
void DMA1_Stream0_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void DMA1_Stream1_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void DMA1_Stream2_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void DMA1_Stream3_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void DMA1_Stream4_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void DMA1_Stream5_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void DMA1_Stream6_IRQHandler 		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void ADC_IRQHandler 				         (void) __attribute__ ((weak, alias("Default_Handler")));             
void CAN1_TX_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void CAN1_RX0_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));        
void CAN1_RX1_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));        
void CAN1_SCE_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));        
void EXTI9_5_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void TIM1_BRK_TIM9_IRQHandler 		   (void) __attribute__ ((weak, alias("Default_Handler")));   
void TIM1_UP_TIM10_IRQHandler 		   (void) __attribute__ ((weak, alias("Default_Handler")));   
void TIM1_TRG_COM_TIM11_IRQHandler 	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void TIM2_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));            
void TIM3_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));            
void TIM4_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));            
void I2C1_EV_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void I2C1_ER_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void I2C2_EV_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void I2C2_ER_IRQHandler 			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void SPI1_IRQHandler  				      (void) __attribute__ ((weak, alias("Default_Handler")));           
void SPI2_IRQHandler 				      (void) __attribute__ ((weak, alias("Default_Handler")));            
void USART1_IRQHandler  			      (void) __attribute__ ((weak, alias("Default_Handler")));         
void USART2_IRQHandler  			      (void) __attribute__ ((weak, alias("Default_Handler")));        
void USART3_IRQHandler   			      (void) __attribute__ ((weak, alias("Default_Handler")));        
void EXTI15_10_IRQHandler   		      (void) __attribute__ ((weak, alias("Default_Handler")));     
void RTC_Alarm_IRQHandler    		      (void) __attribute__ ((weak, alias("Default_Handler")));    
void OTG_FS_WKUP_IRQHandler     	      (void) __attribute__ ((weak, alias("Default_Handler"))); 
void TIM8_BRK_TIM12_IRQHandler   	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler    	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler 	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler          	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void FSMC_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void SDIO_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler            	   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler            	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler         	   (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_IRQHandler              	   (void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler         	   (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler          	   (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler         	   (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler         	   (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler         	   (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler           	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler     	   (void) __attribute__ ((weak, alias("Default_Handler")));
void USART6_IRQHandler           	   (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler          	   (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler          	   (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler   	   (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler    	   (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler      	   (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler           	   (void) __attribute__ ((weak, alias("Default_Handler")));
void DCMI_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void CRYP_IRQHandler             	   (void) __attribute__ ((weak, alias("Default_Handler")));
void HASH_RNG_IRQHandler         	   (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler              	   (void) __attribute__ ((weak, alias("Default_Handler"))); 
/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
// Access linker symbols which contain boundaries addresses
extern u32 _etext;
extern u32 _sdata;
extern u32 _edata;
extern u32 _sbss;
extern u32 _ebss;
// Instruct compiler to put this array in the (.isr_vector) user defined section
u32 vectors[] __attribute__((section(".isr_vector"))) = 
{
   STACK_START_ADDRESS,
   (u32)&Reset_Handler,
   (u32)&NMI_Handler,
   (u32)&HardFault_Handler,
   (u32)MemManage_Handler,
	(u32)BusFault_Handler,
	(u32)UsageFault_Handler,
	0,
	0,
	0,
	0,
	(u32)SVC_Handler,
	(u32)DebugMon_Handler,
	0,
	(u32)PendSV_Handler,
	(u32)SysTick_Handler,
	(u32)WWDG_IRQHandler,
	(u32)PVD_IRQHandler,         
	(u32)TAMP_STAMP_IRQHandler,  
	(u32)RTC_WKUP_IRQHandler,    
	0,                      
	(u32)RCC_IRQHandler,         
	(u32)EXTI0_IRQHandler,       
	(u32)EXTI1_IRQHandler,       
	(u32)EXTI2_IRQHandler,       
	(u32)EXTI3_IRQHandler,       
	(u32)EXTI4_IRQHandler,       
	(u32)DMA1_Stream0_IRQHandler,
	(u32)DMA1_Stream1_IRQHandler,
	(u32)DMA1_Stream2_IRQHandler,
	(u32)DMA1_Stream3_IRQHandler,
	(u32)DMA1_Stream4_IRQHandler,
	(u32)DMA1_Stream5_IRQHandler,
	(u32)DMA1_Stream6_IRQHandler,
	(u32)ADC_IRQHandler,         
	(u32)CAN1_TX_IRQHandler,     
	(u32)CAN1_RX0_IRQHandler,    
	(u32)CAN1_RX1_IRQHandler,    
	(u32)CAN1_SCE_IRQHandler,    
	(u32)EXTI9_5_IRQHandler,     
	(u32)TIM1_BRK_TIM9_IRQHandler,
	(u32)TIM1_UP_TIM10_IRQHandler,
	(u32)TIM1_TRG_COM_TIM11_IRQHandler,
	(u32)TIM1_CC_IRQHandler,     
	(u32)TIM2_IRQHandler,        
	(u32)TIM3_IRQHandler,        
	(u32)TIM4_IRQHandler,        
	(u32)I2C1_EV_IRQHandler,     
	(u32)I2C1_ER_IRQHandler,     
	(u32)I2C2_EV_IRQHandler,     
	(u32)I2C2_ER_IRQHandler,     
	(u32)SPI1_IRQHandler,        
	(u32)SPI2_IRQHandler,        
	(u32)USART1_IRQHandler,      
	(u32)USART2_IRQHandler,      
	(u32)USART3_IRQHandler,      
	(u32)EXTI15_10_IRQHandler,   
	(u32)RTC_Alarm_IRQHandler,   
	(u32)OTG_FS_WKUP_IRQHandler, 
	(u32)TIM8_BRK_TIM12_IRQHandler,
	(u32)TIM8_UP_TIM13_IRQHandler,
	(u32)TIM8_TRG_COM_TIM14_IRQHandler,
	(u32)TIM8_CC_IRQHandler,     
	(u32)DMA1_Stream7_IRQHandler,
	(u32)FSMC_IRQHandler,        
	(u32)SDIO_IRQHandler,        
	(u32)TIM5_IRQHandler,        
	(u32)SPI3_IRQHandler,        
	(u32)UART4_IRQHandler,       
	(u32)UART5_IRQHandler,       
	(u32)TIM6_DAC_IRQHandler,    
	(u32)TIM7_IRQHandler,        
	(u32)DMA2_Stream0_IRQHandler,
	(u32)DMA2_Stream1_IRQHandler,
	(u32)DMA2_Stream2_IRQHandler,
	(u32)DMA2_Stream3_IRQHandler,
	(u32)DMA2_Stream4_IRQHandler,
	(u32)ETH_IRQHandler,         
	(u32)ETH_WKUP_IRQHandler,    
	(u32)CAN2_TX_IRQHandler,     
	(u32)CAN2_RX0_IRQHandler,    
	(u32)CAN2_RX1_IRQHandler,    
	(u32)CAN2_SCE_IRQHandler,    
	(u32)OTG_FS_IRQHandler,      
	(u32)DMA2_Stream5_IRQHandler,
	(u32)DMA2_Stream6_IRQHandler,
	(u32)DMA2_Stream7_IRQHandler,
	(u32)USART6_IRQHandler,      
	(u32)I2C3_EV_IRQHandler,     
	(u32)I2C3_ER_IRQHandler,     
	(u32)OTG_HS_EP1_OUT_IRQHandler,
	(u32)OTG_HS_EP1_IN_IRQHandler,
	(u32)OTG_HS_WKUP_IRQHandler, 
	(u32)OTG_HS_IRQHandler,      
	(u32)DCMI_IRQHandler,        
	(u32)CRYP_IRQHandler,        
	(u32)HASH_RNG_IRQHandler,    
	(u32)FPU_IRQHandler,  
};

/******************************************************************************
* Function Definitions
*******************************************************************************/
void Default_Handler(void)
{
   // Do not do any thing
   while(1);
}

void Reset_Handler(void)
{
	// Calcaulate data used in data copy
	u32  Local_u32DataSectionSize = (u32)&_edata - (u32)&_sdata;
	u8 * Local_PtrDataSram  = (u8 *)&_sdata;
	u8 * Local_PtrDataFlash = (u8 *)&_etext;
	// Calcaulate data used in initalize bss
	u32  Local_u32BssSectionSize = (u32)&_ebss - (u32)&_sbss;
   u8 * Local_PtrBssSram  = (u8 *)&_sbss;
	// Copy .data section from flash to sram
	for(u32 Local_u32Counter = 0;Local_u32Counter < Local_u32DataSectionSize;Local_u32Counter++)
	{
		*Local_PtrDataSram++ = *Local_PtrDataFlash++;
	}
	// Initalize .bss section with zero
   for(u32 Local_u32Counter = 0;Local_u32Counter < Local_u32BssSectionSize;Local_u32Counter++)
	{
		*Local_PtrBssSram++ = 0;
	}
	// Call entry point (main function)
	main();
}

/*************** END OF FUNCTIONS ***************************************************************************/
