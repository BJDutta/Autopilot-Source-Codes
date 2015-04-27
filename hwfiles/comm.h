#ifndef COMM_H
#define COMM_H
#include "lpc17xx.h"

#define GPS_UART   LPC_UART1
#define TEL_UART   LPC_UART0

/* Receive buffer size */
#define RX_BUF_SIZE     250

/* source address */
#define DMA_SetChSrcAdr(x,y) {(x)->DMACCSrcAddr = (uint32_t)(y);}
#define DMA_GetChSrcAdr(x)   ((x)->DMACCSrcAddr)

/* destination address */
#define DMA_SetChDestAdr(x,y) {(x)->DMACCDestAddr = (uint32_t)(y);}
#define DMA_GetChDestAdr(x)   ((x)->DMACCDestAddr)

#define UART_ReceiveRegister(x)   ((x)->RBR)

#define DMA_CTL_CH_SRC_WIDTH_BYTE     0UL        /**< width = 1 byte */
#define DMA_CTL_CH_SRC_WIDTH_HALFWORD (1UL<<18)  /**< width = 2 bytes */
#define DMA_CTL_CH_SRC_WIDTH_WORD     (2UL<<18)  /**< width = 4 bytes */

#define DMA_CTL_CH_DST_WIDTH_BYTE     0UL        /**< width = 1 byte  */
#define DMA_CTL_CH_DST_WIDTH_HALFWORD (1UL<<21)  /**< width = 2 bytes */
#define DMA_CTL_CH_DST_WIDTH_WORD     (2UL<<21)  /**< width = 4 bytes */

#define DMA_CTL_CH_SRC_INC            (1UL<<26)  /**< source increment */
#define DMA_CTL_CH_DST_INC            (1UL<<27)  /**< destination increment */


/* clear  error interrupt requests bitmask 0..7 */
#define DMA_ClrIntErrReq(x)   {LPC_GPDMA->DMACIntErrClr = (x);}

/* clear terminal count  interrupt requests 0..7 */
#define DMA_ClrIntTCount(x)   {LPC_GPDMA->DMACIntTCClear = (x);}

/* Get (masked) terminal count requests */
#define DMA_GetIntTCountStat() (LPC_GPDMA->DMACIntTCStat)

void comm_init();
void uart_gps_dma_init();
int gps_receive(unsigned char buf[]);
uint16_t UART_GetReceiveCountGPS();
void uart_tel_dma_init();
int tel_receive(unsigned char buf[]);
uint16_t UART_GetReceiveCount();
void tel_putchar(char data);
void tel_transmit();
void gps_putchar(char data);
void gps_puts(uint8_t buf[], uint8_t len);
void gps_transmit();








#endif
