/*
 * UART_Bootloader.c
 */ 


#include "sam.h"

#define PORTA 0
#define PORTB 1

/* Application starts from 1kB memory - Bootloader size is 1kB */
/* Change the address if higher boot size is needed */
#define APP_START	0x00000400

/* Target application size can be 15kB */
/* APP_SIZE is the application section size in kB */
/* Change as per APP_START */
#define APP_SIZE	15

/* Flash page size is 64 bytes */
#define PAGE_SIZE	64
/* Memory pointer for flash memory */
#define NVM_MEMORY        ((volatile uint16_t *)FLASH_ADDR)

/* Change the following if different SERCOM and boot pins are used */
#define BOOT_SERCOM			SERCOM2
#define BOOT_SERCOM_BAUD	115200
#define BOOT_PORT			PORTA
#define BOOT_PIN			25
#define LED_PORT            PORTA
#define LED_PIN             9

/* SERCOM USART GCLK Frequency */
#define SERCOM_GCLK		8000000UL
#define BAUD_VAL	(65536.0*(1.0-((float)(16.0*(float)BOOT_SERCOM_BAUD)/(float)SERCOM_GCLK)))

uint8_t data_8 = 1;
uint32_t file_size, i, dest_addr, app_start_address;
uint8_t page_buffer[PAGE_SIZE];
uint32_t *flash_ptr;

enum uart_pad_settings {
	UART_RX_PAD0_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(0) | SERCOM_USART_CTRLA_TXPO(1),
	UART_RX_PAD1_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_TXPO(1),
	UART_RX_PAD2_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(2),
	UART_RX_PAD3_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(3),
	UART_RX_PAD1_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(1),
	UART_RX_PAD3_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1),
};

void uart_init(uint16_t baud_val, enum uart_pad_settings pad_conf)
{
	/* Enable & configure alternate function D for pins PA10 & PA11 */
	/* Change following 3 lines if different SERCOM/SERCOM pins are used */
	PORT->Group[0].WRCONFIG.reg = 0x53010C00;
	PM->APBCMASK.reg |= (1u << 4);
	GCLK->CLKCTRL.reg = 0x4010;
	
	BOOT_SERCOM->USART.CTRLA.reg = pad_conf | SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD;
	BOOT_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
	while(BOOT_SERCOM->USART.SYNCBUSY.bit.CTRLB);
	BOOT_SERCOM->USART.BAUD.reg = baud_val;
	BOOT_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while(BOOT_SERCOM->USART.SYNCBUSY.bit.ENABLE);
}

void uart_write_byte(uint8_t data)
{
	while(!BOOT_SERCOM->USART.INTFLAG.bit.DRE);
	BOOT_SERCOM->USART.DATA.reg = (uint16_t)data;
}

uint8_t uart_read_byte(void)
{
	while(!BOOT_SERCOM->USART.INTFLAG.bit.RXC);
	return((uint8_t)(BOOT_SERCOM->USART.DATA.reg & 0x00FF));
}

void nvm_erase_row(const uint32_t row_address)
{
	/* Check if the module is busy */
	while(!NVMCTRL->INTFLAG.bit.READY);
	/* Clear error flags */
	NVMCTRL->STATUS.reg &= ~NVMCTRL_STATUS_MASK;
	/* Set address and command */
	NVMCTRL->ADDR.reg  = (uintptr_t)&NVM_MEMORY[row_address / 4];
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;
	while(!NVMCTRL->INTFLAG.bit.READY);
}

void nvm_write_buffer(const uint32_t destination_address, const uint8_t *buffer, uint16_t length)
{

	/* Check if the module is busy */
	while(!NVMCTRL->INTFLAG.bit.READY);

	/* Erase the page buffer before buffering new data */
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;

	/* Check if the module is busy */
	while(!NVMCTRL->INTFLAG.bit.READY);

	/* Clear error flags */
	NVMCTRL->STATUS.reg &= ~NVMCTRL_STATUS_MASK;

	uint32_t nvm_address = destination_address / 2;

	/* NVM _must_ be accessed as a series of 16-bit words, perform manual copy
	 * to ensure alignment */
	for (uint16_t k = 0; k < length; k += 2) 
	{
		uint16_t data;
		/* Copy first byte of the 16-bit chunk to the temporary buffer */
		data = buffer[k];
		/* If we are not at the end of a write request with an odd byte count,
		 * store the next byte of data as well */
		if (k < (length - 1)) {
			data |= (buffer[k + 1] << 8);
		}
		/* Store next 16-bit chunk to the NVM memory space */
		NVM_MEMORY[nvm_address++] = data;
	}
	while(!NVMCTRL->INTFLAG.bit.READY);
}

int main(void)
{ 
	PORT->Group[LED_PORT].DIRSET.reg = (1 << LED_PIN);

	/* Check if boot pin is held low - Jump to application if boot pin is high */
	PORT->Group[BOOT_PORT].OUTSET.reg = (1u << BOOT_PIN);
	PORT->Group[BOOT_PORT].PINCFG[BOOT_PIN].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	if ((PORT->Group[BOOT_PORT].IN.reg & (1u << BOOT_PIN)))
	{
		app_start_address = *(uint32_t *)(APP_START + 4);
		/* Rebase the Stack Pointer */
		__set_MSP(*(uint32_t *) APP_START);

		/* Rebase the vector table base address */
		SCB->VTOR = ((uint32_t) APP_START & SCB_VTOR_TBLOFF_Msk);

		/* Jump to application Reset Handler in the application */
		asm("bx %0"::"r"(app_start_address));
	}

	PORT->Group[LED_PORT].OUTSET.reg = (1 << LED_PIN);
	
	
	/* Make CPU to run at 8MHz by clearing prescalar bits */ 
    SYSCTRL->OSC8M.bit.PRESC = 0;
	NVMCTRL->CTRLB.bit.CACHEDIS = 1;
	/* Change pad_conf argument if different pad settings is used */
	uart_init(BAUD_VAL, UART_RX_PAD3_TX_PAD2);
	//uart_init(64278, UART_RX_PAD3_TX_PAD2);
    while (1) 
    {
        data_8 = uart_read_byte();
		if (data_8 == '#')
		{
			uart_write_byte('s');
			uart_write_byte((uint8_t)APP_SIZE);
		}
		else if (data_8 == 'e')
		{
			for(i = APP_START; i < FLASH_SIZE; i = i + 256)
			{
				nvm_erase_row(i);
			}
			dest_addr = APP_START;
			flash_ptr = APP_START;
			uart_write_byte('s');
		}
		else if (data_8 == 'p')
		{
			uart_write_byte('s');
			for (i = 0; i < PAGE_SIZE; i++)
			{
				page_buffer[i] = uart_read_byte();
			}
			nvm_write_buffer(dest_addr, page_buffer, PAGE_SIZE);
			dest_addr += PAGE_SIZE;
			uart_write_byte('s');
		}
		else if (data_8 == 'v')
		{
			uart_write_byte('s');
			for (i = 0; i < (PAGE_SIZE/4); i++)
			{
				app_start_address = *flash_ptr;
				uart_write_byte((uint8_t)app_start_address);
				uart_write_byte((uint8_t)(app_start_address >> 8));
				uart_write_byte((uint8_t)(app_start_address >> 16));
				uart_write_byte((uint8_t)(app_start_address >> 24));
				flash_ptr++;
			}
		}
    }
}

