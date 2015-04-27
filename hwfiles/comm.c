#include "comm.h"
#include "lpc17xx_libcfg_default.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "LED_Blink.h"
#include "config.h"

#ifdef UBLOX_GPS
#define GPS_BAUD 38400
#else
#ifdef PX4_FLOW
#define	GPS_BAUD 115200
#else
#define GPS_BAUD 115200
#endif
#endif

#define TEL_BAUD 57600


unsigned char *dst_adr_read_last;
unsigned char *dst_adr_read_lastgps;


typedef struct {
	uint32_t src_addr;   /**< source address */
	uint32_t dst_addr;   /**< destination address */
	uint32_t next;       /**< next LLI address, 0 if none */
	uint32_t ctrl;       /**< control register */
} dma_list_t;


dma_list_t rxdma;


uint8_t tx_buf[250];
uint8_t txcount=0;


dma_list_t rxdmagps;


uint8_t tx_bufgps[200];
uint8_t txcountgps=0;



// Receive buffer
__IO uint8_t rx_buf[RX_BUF_SIZE];

__IO uint8_t rx_bufgps[RX_BUF_SIZE];

// Terminal Counter flag for Channel 0
__IO uint32_t Channel0_TC;

// Error Counter flag for Channel 0
__IO uint32_t Channel0_Err;

// Terminal Counter flag for Channel 1
__IO uint32_t Channel1_TC;

// Error Counter flag for Channel 1
__IO uint32_t Channel1_Err;

// Terminal Counter flag for Channel 2
__IO uint32_t Channel2_TC;

// Error Counter flag for Channel 2
__IO uint32_t Channel2_Err;

// Terminal Counter flag for Channel 3
__IO uint32_t Channel3_TC;

// Error Counter flag for Channel 3
__IO uint32_t Channel3_Err;

void comm_init(){

	uart_gps_dma_init();
	uart_tel_dma_init();





}

void uart_gps_dma_init(){


	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

	// Pin configuration for UART1
	PINSEL_CFG_Type PinCfg;

	LPC_UART_TypeDef *UARTx = (LPC_UART_TypeDef *)GPS_UART;

	if((uint32_t)UARTx == (uint32_t)LPC_UART0) {
		/*
		 * Initialize UART0 pin connect
		 */
		PinCfg.Funcnum = 1;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Pinnum = 2;
		PinCfg.Portnum = 0;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 3;
		PINSEL_ConfigPin(&PinCfg);
	}
	else if ((uint32_t)UARTx == (uint32_t)LPC_UART1) {
		/*
		 * Initialize UART1 pin connect
		 */
		PinCfg.Funcnum = 2;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Pinnum = 0;
		PinCfg.Portnum = 2;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 1;
		PINSEL_ConfigPin(&PinCfg);
	}


	UART_ConfigStructInit(&UARTConfigStruct);

	/* Set Baudrate to 38400 */ //Default of Ublox GPS
	UARTConfigStruct.Baud_rate = GPS_BAUD;

	// Initialize UART1 peripheral with given to corresponding parameter
	UART_Init(UARTx, &UARTConfigStruct);


	/* Initialize FIFOConfigStruct to default state:
	 *                              - FIFO_DMAMode = DISABLE
	 *                              - FIFO_Level = UART_FIFO_TRGLEV0
	 *                              - FIFO_ResetRxBuf = ENABLE
	 *                              - FIFO_ResetTxBuf = ENABLE
	 *                              - FIFO_State = ENABLE
	 */
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Enable DMA mode in UART
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;

	// Initialize FIFO for UART0 peripheral
	UART_FIFOConfig(UARTx, &UARTFIFOConfigStruct);

	// Enable UART Transmit
	UART_TxCmd(UARTx, ENABLE);

}


int gps_receive(unsigned char buf[]){
	uint16_t rxcount=0,len=0;
	uint16_t index=0;

	if((rxcount=UART_GetReceiveCountGPS())!=0){
//		blink();
		len = rxcount;
		while(rxcount--){
			//			UART_Send(TEL_UART, (uint8_t *)dst_adr_read_lastgps, 1, BLOCKING);
			// mavlink receive buffer
			//			c = *dst_adr_read_lastgps;
			buf[index++] = *dst_adr_read_lastgps;
			dst_adr_read_lastgps = (dst_adr_read_lastgps + 1);
			if((uint32_t)dst_adr_read_lastgps>=((uint32_t)&rx_bufgps + RX_BUF_SIZE)){
				dst_adr_read_lastgps = (unsigned char *)&rx_bufgps;
			}
		}
	}
	return len;
}

uint16_t UART_GetReceiveCountGPS() {

	uint32_t dest = DMA_GetChDestAdr(LPC_GPDMACH3);
	//	uint32_t  lli = LPC_GPDMACH1->DMACCLLI;
	//	uint32_t  ctrl = LPC_GPDMACH1->DMACCControl;

	if(dest<(uint32_t)dst_adr_read_lastgps) {
		return (uint16_t)(RX_BUF_SIZE - (int16_t)((uint32_t)dst_adr_read_lastgps - dest));
	}
	else {
		return (uint16_t)((dest - (uint32_t)dst_adr_read_lastgps));
	}
}

/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************//**
 * @brief               GPDMA interrupt handler sub-routine
 * @param[in]   None
 * @return              None
 **********************************************************************/

void DMA_IRQHandler (void)
{
	uint32_t tmp;
	// Scan interrupt pending
	for (tmp = 0; tmp <= 7; tmp++) {
		if (GPDMA_IntGetStatus(GPDMA_STAT_INT, tmp)){
			// Check counter terminal status
			if(GPDMA_IntGetStatus(GPDMA_STAT_INTTC, tmp)){
				// Clear terminate counter Interrupt pending
				GPDMA_ClearIntPending (GPDMA_STATCLR_INTTC, tmp);

				switch (tmp){
				case 0:
					Channel0_TC++;		// TEL TX
					GPDMA_ChannelCmd(0, DISABLE);
					break;
				case 1:
					Channel1_TC++;		// TEL RX
					//					GPDMA_ChannelCmd(1, DISABLE);
					break;
				case 2:
					Channel2_TC++;		// GPS TX
					GPDMA_ChannelCmd(2, DISABLE);
					break;
				case 3:
					Channel3_TC++;		// GPS RX
					//					GPDMA_ChannelCmd(3, DISABLE);
					break;
				default:
					break;
				}

			}
			// Check error terminal status
			if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, tmp)){
				// Clear error counter Interrupt pending
				GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, tmp);
				switch (tmp){
				case 0:
					Channel0_Err++;
					GPDMA_ChannelCmd(0, DISABLE);
					break;
				case 1:
					Channel1_Err++;
					//					GPDMA_ChannelCmd(1, DISABLE);
					break;
				case 2:
					Channel2_Err++;
					GPDMA_ChannelCmd(2, DISABLE);
					break;
				case 3:
					Channel3_Err++;
					//					GPDMA_ChannelCmd(3, DISABLE);
					break;
				default:
					break;
				}
			}
		}
	}
}

void uart_tel_dma_init(){

	uint32_t idx;
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	GPDMA_Channel_CFG_Type GPDMACfg;
	// Pin configuration for UART0
	PINSEL_CFG_Type PinCfg;

	LPC_UART_TypeDef *UARTx = (LPC_UART_TypeDef *)TEL_UART;

	if((uint32_t)UARTx == (uint32_t)LPC_UART0) {
		/*
		 * Initialize UART0 pin connect
		 */
		PinCfg.Funcnum = 1;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Pinnum = 2;
		PinCfg.Portnum = 0;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 3;
		PINSEL_ConfigPin(&PinCfg);
	}
	else if ((uint32_t)UARTx == (uint32_t)LPC_UART1) {
		/*
		 * Initialize UART1 pin connect
		 */
		PinCfg.Funcnum = 2;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Pinnum = 0;
		PinCfg.Portnum = 2;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 1;
		PINSEL_ConfigPin(&PinCfg);
	}


	UART_ConfigStructInit(&UARTConfigStruct);

	/* Set Baudrate to 19200 */ //suits DMA
	UARTConfigStruct.Baud_rate = TEL_BAUD;

	// Initialize UART0 peripheral with given to corresponding parameter
	UART_Init(UARTx, &UARTConfigStruct);


	/* Initialize FIFOConfigStruct to default state:
	 *                              - FIFO_DMAMode = DISABLE
	 *                              - FIFO_Level = UART_FIFO_TRGLEV0
	 *                              - FIFO_ResetRxBuf = ENABLE
	 *                              - FIFO_ResetTxBuf = ENABLE
	 *                              - FIFO_State = ENABLE
	 */
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Enable DMA mode in UART
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;

	// Initialize FIFO for UART0 peripheral
	UART_FIFOConfig(UARTx, &UARTFIFOConfigStruct);

	// Enable UART Transmit
	UART_TxCmd(UARTx, ENABLE);


	/* GPDMA Interrupt configuration section ------------------------------------------------- */

	/* Initialize GPDMA controller */
	GPDMA_Init();


	/* Setting GPDMA interrupt */
	// Disable interrupt for DMA
	NVIC_DisableIRQ (DMA_IRQn);
	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));



	// Setup GPDMA channel --------------------------------
	// channel 1
	GPDMACfg.ChannelNum = 1;
	// Source memory - don't care
	GPDMACfg.SrcMemAddr = (uint32_t)&UART_ReceiveRegister(UARTx);
	// Destination memory
	GPDMACfg.DstMemAddr = (uint32_t) &rx_buf;

	dst_adr_read_last = (unsigned char *)&rx_buf;


	// Transfer size
	GPDMACfg.TransferSize = sizeof(rx_buf);
	// Transfer width - don't care
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
	// Source connection
	if((uint32_t)UARTx == (uint32_t)LPC_UART0) {
		GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;
	}
	else if ((uint32_t)UARTx == (uint32_t)LPC_UART1) {
		GPDMACfg.SrcConn = GPDMA_CONN_UART1_Rx;
	}
	// Destination connection - don't care
	GPDMACfg.DstConn = 0;

	rxdma.src_addr = GPDMACfg.SrcMemAddr;   /**< source address */
	rxdma.dst_addr = GPDMACfg.DstMemAddr;   /**< destination address */
	rxdma.next  = (uint32_t)&rxdma;       /**< next LLI address, 0 if none */
	//    rxdma.ctrl  =  RX_BUF_SIZE|DMA_CTL_CH_SRC_WIDTH_BYTE|DMA_CTL_CH_DST_WIDTH_BYTE|DMA_CTL_CH_DST_INC;//LPC_GPDMACH1->DMACCControl;    /**< control register */

	//    LPC_GPDMACH1->DMACCLLI = rxdma.next;//(uint32_t)&rxdma;

	// Linker List Item - unused
	GPDMACfg.DMALLI = (uint32_t)&rxdma;//GPDMACfg.SrcMemAddr; // ring buffer
	GPDMA_Setup(&GPDMACfg);

	/* Reset terminal counter */
	Channel0_TC = 1;
	/* Reset Error counter */
	Channel0_Err = 0;


	UARTx = (LPC_UART_TypeDef *)GPS_UART;

	// Setup GPDMA channel --------------------------------
	// channel 1
	GPDMACfg.ChannelNum = 3;
	// Source memory - don't care
	GPDMACfg.SrcMemAddr = (uint32_t)&UART_ReceiveRegister(UARTx);
	// Destination memory
	GPDMACfg.DstMemAddr = (uint32_t) &rx_bufgps;

	dst_adr_read_lastgps = (unsigned char *)&rx_bufgps;


	// Transfer size
	GPDMACfg.TransferSize = sizeof(rx_bufgps);
	// Transfer width - don't care
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
	// Source connection
	if((uint32_t)UARTx == (uint32_t)LPC_UART0) {
		GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;
	}
	else if ((uint32_t)UARTx == (uint32_t)LPC_UART1) {
		GPDMACfg.SrcConn = GPDMA_CONN_UART1_Rx;
	}
	// Destination connection - don't care
	GPDMACfg.DstConn = 0;

	rxdmagps.src_addr = GPDMACfg.SrcMemAddr;   /**< source address */
	rxdmagps.dst_addr = GPDMACfg.DstMemAddr;   /**< destination address */
	rxdmagps.next  = (uint32_t)&rxdmagps;       /**< next LLI address, 0 if none */
	//    rxdma.ctrl  =  RX_BUF_SIZE|DMA_CTL_CH_SRC_WIDTH_BYTE|DMA_CTL_CH_DST_WIDTH_BYTE|DMA_CTL_CH_DST_INC;//LPC_GPDMACH1->DMACCControl;    /**< control register */

	//    LPC_GPDMACH1->DMACCLLI = rxdma.next;//(uint32_t)&rxdma;

	// Linker List Item - unused
	GPDMACfg.DMALLI = (uint32_t)&rxdmagps;//GPDMACfg.SrcMemAddr; // ring buffer
	GPDMA_Setup(&GPDMACfg);

	/* Reset terminal counter */
	Channel2_TC = 1;
	/* Reset Error counter */
	Channel2_Err = 0;



	// Enable interrupt for DMA
	NVIC_EnableIRQ (DMA_IRQn);

	//	// Enable GPDMA channel 0
	//	GPDMA_ChannelCmd(0, ENABLE);
	// Make sure GPDMA channel 1 is disabled
	GPDMA_ChannelCmd(1, DISABLE);
	GPDMA_ChannelCmd(3, DISABLE);


	/* Reset terminal counter */
	Channel1_TC = 0;
	/* Reset Error counter */
	Channel1_Err = 0;

	/* Reset terminal counter */
	Channel3_TC = 0;
	/* Reset Error counter */
	Channel3_Err = 0;

	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);

	// Enable GPDMA channel 1
	GPDMA_ChannelCmd(1, ENABLE);
	GPDMA_ChannelCmd(3, ENABLE);

	rxdma.ctrl = (LPC_GPDMACH1->DMACCControl)^(GPDMA_DMACCxControl_I);  // reset 31st bit to disable TC interrupt
	rxdmagps.ctrl = (LPC_GPDMACH3->DMACCControl)^(GPDMA_DMACCxControl_I);  // reset 31st bit to disable TC interrupt

	// Clear Rx buffer using DMA
	for (idx = 0; idx < RX_BUF_SIZE; idx++){
		rx_buf[idx] = 0;
	}

	// Clear Rx buffer using DMA
	for (idx = 0; idx < RX_BUF_SIZE; idx++){
		rx_bufgps[idx] = 0;
	}
}

int tel_receive(unsigned char buf[]){
	uint16_t rxcount=0,len=0;
	uint16_t index=0;

	if((rxcount=UART_GetReceiveCount())!=0){
		len = rxcount;
		while(rxcount--){
			//			UART_Send(TEL_UART, (uint8_t *)dst_adr_read_last, 1, BLOCKING);
			// mavlink receive buffer
			//			c = *dst_adr_read_last;
			buf[index++] = *dst_adr_read_last;
			dst_adr_read_last = (dst_adr_read_last + 1);
			if((uint32_t)dst_adr_read_last>=((uint32_t)&rx_buf + RX_BUF_SIZE)){
				dst_adr_read_last = (unsigned char *)&rx_buf;
			}
		}
	}
	return len;
}



//}


uint16_t UART_GetReceiveCount() {

	uint32_t dest = DMA_GetChDestAdr(LPC_GPDMACH1);
	//	uint32_t  lli = LPC_GPDMACH1->DMACCLLI;
	//	uint32_t  ctrl = LPC_GPDMACH1->DMACCControl;

	if(dest<(uint32_t)dst_adr_read_last) {
		return (uint16_t)(RX_BUF_SIZE - (int16_t)((uint32_t)dst_adr_read_last - dest));
	}
	else {
		return (uint16_t)((dest - (uint32_t)dst_adr_read_last));
	}
}

void tel_putchar(char data){

	//	UART_Send(TEL_UART, (uint8_t *)&data, 1, BLOCKING);
	tx_buf[txcount++] = data;

	if(txcount>=240)tel_transmit();

}

void tel_transmit(){


	uint32_t idx;
	static uint8_t tx_buf_copy[250];
	LPC_UART_TypeDef *UARTx = (LPC_UART_TypeDef *)TEL_UART;
	GPDMA_Channel_CFG_Type GPDMACfg;

	/* GPDMA Interrupt configuration section ------------------------------------------------- */

	/* Initialize GPDMA controller */
	//	GPDMA_Init();


	/* Setting GPDMA interrupt */
	// Disable interrupt for DMA
	//	NVIC_DisableIRQ (DMA_IRQn);
	/* preemption = 1, sub-priority = 1 */
	//	NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));

	while ((Channel0_TC == 0) && (Channel0_Err == 0));

	Channel0_TC = 0; Channel0_Err = 0;

	for(idx=0; idx<txcount; idx++)tx_buf_copy[idx] = tx_buf[idx];

	// Setup GPDMA channel --------------------------------
	// channel 0
	GPDMACfg.ChannelNum = 0;
	// Source memory
	GPDMACfg.SrcMemAddr = (uint32_t) &tx_buf_copy;
	// Destination memory - don't care
	GPDMACfg.DstMemAddr = 0;
	// Transfer size
	GPDMACfg.TransferSize = txcount;
	// Transfer width - don't care
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	// Source connection - don't care
	GPDMACfg.SrcConn = 0;
	// Destination connection
	if((uint32_t)UARTx == (uint32_t)LPC_UART0) {
		GPDMACfg.DstConn = GPDMA_CONN_UART0_Tx;
	}
	else if ((uint32_t)UARTx == (uint32_t)LPC_UART1) {
		GPDMACfg.DstConn = GPDMA_CONN_UART1_Tx;
	}

	// Linker List Item - unused
	GPDMACfg.DMALLI = 0;
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);

	GPDMA_ChannelCmd(0, ENABLE);

	txcount = 0;
}

void gps_putchar(char data){

		UART_Send((LPC_UART_TypeDef *)GPS_UART, (uint8_t *)&data, 1, BLOCKING);
//	tx_bufgps[txcountgps++] = data;

//	if(txcountgps>=200)gps_transmit();

}

void gps_puts(uint8_t buf[], uint8_t len){
	uint8_t idx=0;
	while(len>0){
		gps_putchar(buf[idx++]);
		len--;
	}
//	gps_transmit();
}
void gps_transmit(){


	uint32_t idx;
	static uint8_t tx_buf_copy[200];
	LPC_UART_TypeDef *UARTx = (LPC_UART_TypeDef *)GPS_UART;
	GPDMA_Channel_CFG_Type GPDMACfg;

	/* GPDMA Interrupt configuration section ------------------------------------------------- */

	/* Initialize GPDMA controller */
	//	GPDMA_Init();


	/* Setting GPDMA interrupt */
	// Disable interrupt for DMA
	//	NVIC_DisableIRQ (DMA_IRQn);
	/* preemption = 1, sub-priority = 1 */
	//	NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));

	while ((Channel2_TC == 0) && (Channel2_Err == 0));

	Channel2_TC = 0; Channel2_Err = 0;

	for(idx=0; idx<txcountgps; idx++)tx_buf_copy[idx] = tx_bufgps[idx];

	// Setup GPDMA channel --------------------------------
	// channel 2
	GPDMACfg.ChannelNum = 2;
	// Source memory
	GPDMACfg.SrcMemAddr = (uint32_t) &tx_buf_copy;
	// Destination memory - don't care
	GPDMACfg.DstMemAddr = 0;
	// Transfer size
	GPDMACfg.TransferSize = txcountgps;
	// Transfer width - don't care
	GPDMACfg.TransferWidth = 0;
	// Transfer type
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	// Source connection - don't care
	GPDMACfg.SrcConn = 0;
	// Destination connection
	if((uint32_t)UARTx == (uint32_t)LPC_UART0) {
		GPDMACfg.DstConn = GPDMA_CONN_UART0_Tx;
	}
	else if ((uint32_t)UARTx == (uint32_t)LPC_UART1) {
		GPDMACfg.DstConn = GPDMA_CONN_UART1_Tx;
	}

	// Linker List Item - unused
	GPDMACfg.DMALLI = 0;
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg);

	GPDMA_ChannelCmd(2, ENABLE);

	txcountgps = 0;
}





//
//	/*-------------------------MAIN FUNCTION------------------------------*/
//	/*********************************************************************//**
//	 * @brief               c_entry: Main UART program body
//	 * @param[in]   None
//	 * @return              int
//	 **********************************************************************/
//	int c_entry(void)
//	{

//
//		// DeInitialize UART0 peripheral
//		UART_DeInit(LPC_UART0);
//
//		/* Loop forever */
//		while(1);
//		return 1;
//	}
//
//	/* With ARM and GHS toolsets, the entry point is main() - this will
//	   allow the linker to generate wrapper code to setup stacks, allocate
//	   heap area, and initialize and copy code and data segments. For GNU
//	   toolsets, the entry point is through __start() in the crt0_gnu.asm
//	   file, and that startup code will setup stacks and data */
//	int main(void)
//	{
//		return c_entry();
//	}
//
//
//
//}
