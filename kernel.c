#if !defined(__cplusplus)
#include <stdbool.h>
#endif
#include <stddef.h>
#include <stdint.h>

void ns_sleep();

#define CONFIG_TX_DESCR_NUM   16
#define CONFIG_RX_DESCR_NUM   16
#define CONFIG_ETH_BUFSIZE    2048

#define CONFIG_DW_GMAC_DEFAULT_DMA_PBL 8
#define DMA_PBL               (CONFIG_DW_GMAC_DEFAULT_DMA_PBL<<8)
#define TX_TOTAL_BUFSIZE      (CONFIG_ETH_BUFSIZE * CONFIG_TX_DESCR_NUM)
#define RX_TOTAL_BUFSIZE      (CONFIG_ETH_BUFSIZE * CONFIG_RX_DESCR_NUM)
#define MAC_MAX_FRAME_SZ      1600

#define DESC_TXSTS_OWNBYDMA   (1 << 31)
#define DESC_TXCTRL_SIZE1SHFT (0)
#define DESC_TXCTRL_SIZE1MASK (0x7FF << 0)
#define DESC_TXCTRL_TXLAST    (1 << 30)
#define DESC_TXCTRL_TXFIRST   (1 << 29)
#define DESC_TXCTRL_TXCHAIN   (1 << 24)
#define DESC_RXCTRL_RXCHAIN   (1 << 24)
#define DESC_RXCTRL_SIZE1MASK (0x7FF << 0)
#define DESC_RXSTS_OWNBYDMA   (1 << 31)
#define STOREFORWARD          (1 << 21)
#define FLUSHTXFIFO           (1 << 20)
#define FIXEDBURST            (1 << 16)
#define PRIORXTX_41           (3 << 14)
#define RXSTART               (1 << 1)
#define TXSTART               (1 << 13)
#define RXENABLE              (1 << 2)
#define TXENABLE              (1 << 3)
#define FULLDUPLEX            (1 << 11)
#define LINKUP                (1 << 8)

// The UART registers base address.
#define UART0_BASE 0x01C28000
// Macros to access UART registers.
#define UART0_RBR *(volatile uint32_t *)(UART0_BASE + 0x00)
#define UART0_THR *(volatile uint32_t *)(UART0_BASE + 0x00)
#define UART0_IER *(volatile uint32_t *)(UART0_BASE + 0x04)
#define UART0_FCR *(volatile uint32_t *)(UART0_BASE + 0x08)
#define UART0_LSR *(volatile uint32_t *)(UART0_BASE + 0x14)
#define UART0_USR *(volatile uint32_t *)(UART0_BASE + 0x7C)

// The GMAC registers base address.
#define GMAC_BASE         0x01C50000
// Macros to access GMAC registers.
#define GMAC_CONTROL      *(volatile uint32_t *)(GMAC_BASE + 0x00)
#define GMAC_FRAME_FILTER *(volatile uint32_t *)(GMAC_BASE + 0x04)
#define GMAC_GMII_ADDR    *(volatile uint32_t *)(GMAC_BASE + 0x10)
#define GMAC_ADDR0_HIGH   *(volatile uint32_t *)(GMAC_BASE + 0x40)
#define GMAC_ADDR0_LOW    *(volatile uint32_t *)(GMAC_BASE + 0x44)
#define GDMA_XMT_POLL     *(volatile uint32_t *)(GMAC_BASE + 0x1004)
#define GDMA_RCV_POLL     *(volatile uint32_t *)(GMAC_BASE + 0x1008)
#define GDMA_RCV_LIST     *(volatile uint32_t *)(GMAC_BASE + 0x100C)
#define GDMA_XMT_LIST     *(volatile uint32_t *)(GMAC_BASE + 0x1010)
#define GDMA_BUS_MODE     *(volatile uint32_t *)(GMAC_BASE + 0x1000)
#define GDMA_OPERATION    *(volatile uint32_t *)(GMAC_BASE + 0x1018)

// The DRAM base address.
#define DRAM_BASE 0x40000000

// The DRAM base address.
#define SRAM_BASE 0x00020000

static unsigned char ip_address[4]  = { 192, 168, 88, 5 };
static unsigned char mac_address[6] = { 2,3,4,5,6,7 };

// Structure representing an Ethernet frame header
struct ether_hdr {
  volatile char dst_mac[6];
  volatile char src_mac[6];
  volatile uint16_t ethertype;
};

// Structure representing an Ethernet frame header
struct ip_udp_hdr {
  volatile unsigned char ip_v:4, ip_hl:4;
  volatile unsigned char ip_tos;
  volatile unsigned short int ip_len;
  volatile unsigned short int ip_id;
  volatile unsigned short int ip_off;
  volatile unsigned char ip_ttl;
  volatile unsigned char ip_protocol;
  volatile unsigned short int ip_sum;
  volatile unsigned char ip_src[4];
  volatile unsigned char ip_dst[4];
  volatile unsigned short int udp_sport;
  volatile unsigned short int udp_dport;
  volatile unsigned short int udp_length;
  volatile unsigned short int udp_checksum;
};

struct arp_hdr {
  volatile uint16_t htype;
  volatile uint16_t ptype;
  volatile unsigned char hlen;
  volatile unsigned char plen;
  volatile uint16_t oper;
  // The following assumes IPv4 over Ethernet. This will always be the case.
  volatile char sha[6];
  volatile char spa[4];
  volatile char tha[6];
  volatile char tpa[4];
};

// Structure representing a DMA descriptor (TX or RX)
struct dmamacdescr {
  volatile uint32_t txrx_status;
  volatile uint32_t dmamac_cntl;
  volatile void *dmamac_addr;
  volatile struct dmamacdescr *dmamac_next;
};

// Set up some pointers to locations in DRAM for GMAC DMA
volatile char *txbuf = (void*)(DRAM_BASE + 0x0);
volatile char *rxbuf = (void*)(DRAM_BASE + 0x8000);
volatile struct dmamacdescr *tx_mac_descrtable = (void*)(DRAM_BASE + 0x10000);
volatile struct dmamacdescr *rx_mac_descrtable = (void*)(DRAM_BASE + 0x10100);

// Current CPU position in DMA pointers
unsigned char rx_idx = 0;
unsigned char tx_idx = 0;

// The length of a null terminated string
size_t strlen(const char* str)
{
  size_t ret = 0;
  while ( str[ret] != 0 )
    ret++;
  return ret;
}

// Set up the UART (serial port)
void uart_init()
{
  // Disable UART interrupts.
  UART0_IER = 0x00000000;

  // Configure UART (enable FIFO)
  UART0_FCR = 0x00000001;

}

// UART is ready to receive data to transmit?
unsigned char uart_tx_ready()
{
  return (UART0_USR & 2);
}

// UART has received data?
unsigned char uart_rx_ready()
{
  return (UART0_LSR & 1);
}

// Push one byte to the UART port (blocking until ready to transmit)
void uart_putc(unsigned char byte)
{
  // Wait for UART transmit FIFO to be not full.
  while ( ! uart_tx_ready() );
  UART0_THR = byte;
}

// Get one byte from the UART port (blocking until data available)
unsigned char uart_getc()
{
    // Wait for UART to have recieved something.
    while ( ! uart_rx_ready() );
    return UART0_RBR;
}

// Write some bytes from memory to the UART
void uart_write(const unsigned char* buffer, size_t size)
{
  for ( size_t i = 0; i < size; i++ )
    uart_putc(buffer[i]);
}

// Write a zero terminated string to the UART
void uart_print(const char* str)
{
  uart_write((const unsigned char*) str, strlen(str));
}

// Print a char to the UART as ASCII HEX
void uart_print_uint8(unsigned char number)
{
  unsigned char chars[] = "0123456789ABCDEF";
  uart_putc(chars[(number >> 4) & 0xF]);
  uart_putc(chars[(number >> 0) & 0xF]);
}

// Print a uint32 to the UART as ASCII HEX
void uart_print_uint32(uint32_t number)
{
  unsigned char chars[] = "0123456789ABCDEF";
  uart_putc(chars[(number >> 28) & 0xF]);
  uart_putc(chars[(number >> 24) & 0xF]);
  uart_putc(chars[(number >> 20) & 0xF]);
  uart_putc(chars[(number >> 16) & 0xF]);
  uart_putc(chars[(number >> 12) & 0xF]);
  uart_putc(chars[(number >> 8) & 0xF]);
  uart_putc(chars[(number >> 4) & 0xF]);
  uart_putc(chars[(number >> 0) & 0xF]);
}

// Fill some memory with a specified char
void memset(volatile void *s, unsigned char c, size_t n)
{
    volatile unsigned char* p=s;
    while(n--)
        *p++ = c;
}

// Set up and enable the GMAC (Ethernet)
void gmac_init()
{
  unsigned char idx;
  volatile struct dmamacdescr *desc_p;

  // Reset GMAC DMA
  uart_print("Resetting GMAC DMA... ");
  GDMA_BUS_MODE = (GDMA_BUS_MODE | 1);
  while (GDMA_BUS_MODE & 1);
  uart_print("[DONE]\r\n");

  // Set MAC address
  uart_print("Setting MAC address... ");
  uint32_t mac_low, mac_high;
  mac_low = mac_address[0] + (mac_address[1] << 8) + (mac_address[2] << 16) + (mac_address[3] << 24);
  mac_high = mac_address[4] + (mac_address[5] << 8);
  GMAC_ADDR0_LOW  = mac_low;
  GMAC_ADDR0_HIGH = mac_high;
  while (GDMA_BUS_MODE & 1);
  uart_print("[DONE]\r\n");

  // Zero-fill buffers
  memset(rxbuf, 0, RX_TOTAL_BUFSIZE);
  memset(txbuf, 0, TX_TOTAL_BUFSIZE);

  // Set up RX buffer descriptors
  uart_print("Configuring RX buffers... ");
  for (idx = 0; idx < CONFIG_RX_DESCR_NUM; idx++) {
    desc_p = rx_mac_descrtable + idx;

    desc_p->dmamac_addr = &rxbuf[idx * CONFIG_ETH_BUFSIZE];
    desc_p->dmamac_next = rx_mac_descrtable + idx + 1;

    desc_p->dmamac_cntl = (MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | DESC_RXCTRL_RXCHAIN;

    desc_p->txrx_status = DESC_RXSTS_OWNBYDMA;
  }
  desc_p->dmamac_next = rx_mac_descrtable;

  // Configure GMAC DMA with RX buffer descriptors
  GDMA_RCV_LIST = (uint32_t)rx_mac_descrtable;
  uart_print("[DONE]\r\n");

  // Set up TX buffer descriptors
  uart_print("Configuring TX buffers... ");
  for (idx = 0; idx < CONFIG_TX_DESCR_NUM; idx++) {
    desc_p = tx_mac_descrtable + idx;

    desc_p->dmamac_addr = &txbuf[idx * CONFIG_ETH_BUFSIZE];
    desc_p->dmamac_next = tx_mac_descrtable + idx + 1;

    desc_p->dmamac_cntl = DESC_TXCTRL_TXCHAIN;
    desc_p->txrx_status = 0;
  }
  desc_p->dmamac_next = tx_mac_descrtable;

  // Configure GMAC DMA with TX buffer descriptors
  GDMA_XMT_LIST = (uint32_t)tx_mac_descrtable;
  uart_print("[DONE]\r\n");

  GDMA_BUS_MODE  = FIXEDBURST | PRIORXTX_41 | DMA_PBL;
  GDMA_OPERATION = GDMA_OPERATION | FLUSHTXFIFO | STOREFORWARD;
  GDMA_OPERATION = GDMA_OPERATION | RXSTART | TXSTART;
  GMAC_CONTROL   = GMAC_CONTROL | RXENABLE | TXENABLE;
  GMAC_CONTROL   = GMAC_CONTROL | FULLDUPLEX | LINKUP;
}

void handle_ip_packet(volatile void * frame)
{
  unsigned int n;
  volatile struct ip_udp_hdr *header = frame + 14;
    if(header->ip_dst[0] == ip_address[0] && header->ip_dst[1] == ip_address[1] && header->ip_dst[2] == ip_address[2] && header->ip_dst[3] == ip_address[3])
    {
      switch(header->ip_protocol)
      {
        case(0x11):
          if(__builtin_bswap16(header->udp_dport) == 8888)
          {
            volatile unsigned char *data = frame + 14 + 20 + 8;
            uint32_t length = __builtin_bswap16(header->udp_length) - 8;
            if(length <=1200)
            {
              uart_print("Received data: ");
              unsigned int mask;
              for(n=0;n<length;n++)
              {
                for (mask = 0x80; mask != 0; mask >>= 1) {
                  if (data[n] & mask) {
                    // High
                    ns_sleep();
                    ns_sleep();
                    // One Low
                    ns_sleep();
                  } else {
                    // High
                    ns_sleep();
                    // Zero low
                    ns_sleep();
                    ns_sleep();
                  }
                }
              }
            } else {
              uart_print("Data too large.");
            }
          } else {
            uart_print("Ignoring UDP packet not on port 8888.");
          }
          break;
        default:
        uart_print("Ignoring non-UDP packet.");
      }
    } else {
      uart_print("Ignoring packet for non-local IP.");
    }
}

void handle_arp_frame(volatile void * frame)
{
  int n;
  volatile struct ether_hdr *frame_hdr = frame;
  volatile struct arp_hdr *arp_request = frame + 14;
  if (__builtin_bswap16(arp_request->oper) == 1)
  {
    if(arp_request->tpa[0] == ip_address[0] && arp_request->tpa[1] == ip_address[1] && arp_request->tpa[2] == ip_address[2] && arp_request->tpa[3] == ip_address[3])
    {
      uart_print("Responding!");
      volatile struct dmamacdescr *desc_tx = tx_mac_descrtable + tx_idx;
      volatile struct ether_hdr *reply_hdr = (void *)desc_tx->dmamac_addr;
      volatile struct arp_hdr *arp_response = (void *)desc_tx->dmamac_addr + 14;
      for(n=0;n<6;n++)
        reply_hdr->dst_mac[n] = frame_hdr->src_mac[n];
      for(n=0;n<6;n++)
        reply_hdr->src_mac[n] = mac_address[n];
      reply_hdr->ethertype = __builtin_bswap16(0x0806);
      arp_response->htype  = __builtin_bswap16(1);
      arp_response->ptype  = __builtin_bswap16(0x0800);
      arp_response->hlen   = 6;
      arp_response->plen   = 4;
      arp_response->oper   = __builtin_bswap16(2);
      for(n=0;n<6;n++)
        arp_response->sha[n] = mac_address[n];
      for(n=0;n<6;n++)
        arp_response->tha[n] = arp_request->sha[n];
      for(n=0;n<4;n++)
        arp_response->spa[n] = ip_address[n];
      for(n=0;n<4;n++)
        arp_response->tpa[n] = arp_request->spa[n];

      desc_tx->dmamac_cntl |= (((28+6+6+2) << DESC_TXCTRL_SIZE1SHFT) & DESC_TXCTRL_SIZE1MASK) | DESC_TXCTRL_TXLAST | DESC_TXCTRL_TXFIRST;
      desc_tx->txrx_status = DESC_TXSTS_OWNBYDMA;
      GDMA_XMT_POLL = 0xFFFFFFFF;
      tx_idx++;
      if(tx_idx >= CONFIG_TX_DESCR_NUM) tx_idx = 0;
    } else {
      uart_print("Ignoring request for non-local IP.");
    }
  } else {
    uart_print("Inoring not a request.");
  }

}

// Wait for an Ethernet frame to arrive and process it.
void receive_frame()
{
  // Get a pointer to the current RX descriptor and frame data
  volatile struct dmamacdescr *desc_p = rx_mac_descrtable + rx_idx;
  volatile struct ether_hdr *frame_hdr = desc_p->dmamac_addr;

  // Wait for the GMAC to release the descriptor
  while(desc_p->txrx_status & DESC_RXSTS_OWNBYDMA);

  // Network byte order inverted.
  switch(__builtin_bswap16(frame_hdr->ethertype))
  {
    case 0x0806:
      uart_print("ARP frame received... ");
      handle_arp_frame(desc_p->dmamac_addr);
      uart_print("\r\n");
      break;
    case 0x0800:
      uart_print("IP frame received... ");
      handle_ip_packet(desc_p->dmamac_addr);
      uart_print("\r\n");
      break;
    default:
      uart_print("Unknown ethertype received!\r\n");
  }
  // Release the descriptor back to DMA
  desc_p->txrx_status = DESC_RXSTS_OWNBYDMA;
  GDMA_RCV_POLL = 0xFFFFFFFF;

  // Increment DMA frame position
  rx_idx++;
  if(rx_idx >= CONFIG_RX_DESCR_NUM) rx_idx = 0;
}

#if defined(__cplusplus)
extern "C" /* Use C linkage for kernel_main. */
#endif

// Main program loop
void kernel_main(uint32_t r0, uint32_t r1, uint32_t atags)
{
  (void) r0;
  (void) r1;
  (void) atags;

  uart_init();
  uart_print("Booting...\r\n");
  gmac_init();

  while ( true )
  {
    receive_frame();
  }
}
