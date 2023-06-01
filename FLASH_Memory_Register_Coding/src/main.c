#include "stm32f4xx.h"

#define sector0					0x08000000				// sector1 address
#define sector1					0x08004000				// sector2 address
#define sector2					0x08008000				// sector3 address
#define sector3					0x0800C000				// sector4 address		and so on...
#define lock					(1 << 31)
#define key1					0x45670123
#define key2					0xCDEF89AB
#define flash_EOPIE				(1 << 24)				//End of operation interrupt enable
#define flash_busy_flag			0x00010000



void CLK_Config(void);
void Led_Config(void);
void Flash_Unlock(void);
void Flash_Lock(void);
void Flash_Erase_Sector(uint8_t number);
void Flash_Write(uint32_t address, uint32_t data);
uint32_t Flash_Read(uint32_t address);

uint32_t var;

int main(void)
{
	CLK_Config();
	Led_Config();
	Flash_Unlock();										// unlock to write
	Flash_Erase_Sector(2);								// sector2 erase
	Flash_Write(sector2,0xAB);							// to write the AB to the sector2
	Flash_Lock();										// lock
	var = Flash_Read(sector2);							// read the sector2
	Flash_Lock();										// lock

  while (1)
  {

  }
}


void CLK_Config() 											// Clock speed for 168MHz
{
	RCC->CR |= 0x00010000;                 					// HSEON ENABLE
	while(!(RCC->CR & 0x00020000));        					// HSEON Ready Flag wait
	RCC->CR |= 0x00080000;              					// CSS ENABLE
	RCC->CR |= 0x01000000;									// PLL ON
	RCC->PLLCFGR |= 0x00400000;        						// PLL SRC HSE is selected
	RCC->PLLCFGR |= 0x00000004;       						// PLL M 4
	RCC->PLLCFGR |= 0x00005A00;        						// PLL N 168
	RCC->PLLCFGR |= 0x00000000;       						// PLL P 2
	RCC->CFGR |= 0x00000000;          						// AHB PRESCALER 1
	RCC->CFGR |= 0x00080000;          						// APB2 PRESCALER 2
	RCC->CFGR |= 0x00001400;          						// APB1 PRESCALER 4
	RCC->CIR |= 0x00080000;             					// HSE READY FLAG CLEAR
	RCC->CIR |= 0x00800000;             					// CSS FLAG CLEAR
}

void Led_Config(void)  										// User led configuration
{
	RCC->AHB1ENR |= 0x1U << 3U; 							// D port clock enable

	GPIOD->MODER |= 0x55000000; 							// pins D12, D13, D14, D15 is selected output mode
	GPIOD->OSPEEDR |= 0xFF000000; 							// very high speed is selected
	GPIOD->PUPDR |= 0x00000000; 							// no pull up, pull down
}

void FLASH_IRQHandler()
{
	GPIOD->ODR = 0x0000F000;  						 		// pins set
	FLASH->SR |= (1 << 0);						     		// clear interrupt flag
}

void Flash_Unlock(void)
{
	while((FLASH->SR & flash_busy_flag) != 0);				// wait for busy flag
	FLASH->KEYR |= key1;
	FLASH->KEYR |= key2;
	FLASH->CR |= flash_EOPIE;								// End of operation interrupt enable
	NVIC_EnableIRQ(FLASH_IRQn);								// Enable nested vector interrupt for FLASH interrupt
}

void Flash_Lock(void)
{
	while((FLASH->SR & flash_busy_flag) != 0);				// wait for busy flag
	FLASH->CR |= lock;
}

void Flash_Erase_Sector(uint8_t sectorx)
{
	while((FLASH->SR & flash_busy_flag) != 0);				// wait for busy flag
	FLASH->CR |= (1 << 1);									// sector erase activated
	FLASH->CR |= (sectorx << 3);							// SNB reigister
	FLASH->CR |= (1 << 16);									// STRT bit This bit triggers an erase operation when set. It is set only by software and cleared when the BSY bit is cleared.
}

void Flash_Write(uint32_t address, uint32_t data)
{
	while((FLASH->SR & flash_busy_flag) != 0);				// wait for busy flag
	FLASH->CR |= (2 << 8);									// program size x32
	FLASH->CR |= (1 << 0);									// flash programming activated
	*(uint32_t*)address = data;								// writing the data to the address
}

uint32_t Flash_Read(uint32_t address)
{
	while((FLASH->SR & flash_busy_flag) != 0);				// wait for busy flag
	uint32_t read_data;
	read_data = *(uint32_t*)address;						// reading data from the address
	return read_data;
}


