#ifndef __STM32_CAN_h_
#define __STM32_CAN_h_

// #define DEBUG_STM32_CAN
#define AF4   0x04

/* Symbolic names for bit rate of CAN message                                */
typedef enum {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

/* Symbolic names for formats of CAN message                                 */
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {DATA_FRAME = 0, REMOTE_FRAME}         CAN_FRAME;

typedef struct
{
  uint32_t id;        /* 29 bit identifier                               */
  uint8_t  data[8];   /* Data field                                      */
  uint8_t  len;       /* Length of data field in bytes                   */
  uint8_t  ch;        /* Object channel(Not use)                         */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef const struct
{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
} CAN_bit_timing_config_t;

typedef struct
{
  /**
   * Specified filter index. Valid values are in range [0, 13]
   */
  uint8_t index;

  /**
   * Select filter scale.
   *   0: Dual 16-bit scale configuration
   *   1: Single 32-bit scale configuration
   */
  uint8_t scale;

  /**
   * Select filter mode.
   *   0: Two 32-bit registers of filter bank x are in Identifier Mask mode
   *   1: Two 32-bit registers of filter bank x are in Identifier List mode
   */
  uint8_t mode;

  /**
   * Select FIFO assignment.
   *   0: Filter assigned to FIFO 0
   *   1: Filter assigned to FIFO 1
   */
  uint8_t fifo;

  /**
   * Filter bank register 1
   */
  uint32_t bank1;

  /**
   * Filter bank register 2
   */
  uint32_t bank2;
} CAN_filter_t;

#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU


class STM32_CAN {
  public:
    STM32_CAN(void) : _initialized(false), _filters(0), _filter_count(0), _serial(0) {};

    bool begin(HardwareSerial *serial, BITRATE bitrate);
    void setFilters(CAN_filter_t *filters, int count);
    bool receive(CAN_msg_t* CAN_rx_msg);
    bool send(CAN_msg_t* CAN_tx_msg);
    int available(void);
    void printRegister(const char * buf, uint32_t reg);
    static uint32_t makeFilter32(uint32_t id, uint8_t format, uint8_t type)
    {
      uint32_t out;
      if (format == EXTENDED_FORMAT) {      // Extended frame format
          out = ((id & CAN_EXT_ID_MASK) << 3) | STM32_CAN_TIR_IDE;
      } else {                              // Standard frame format
          out = ((id & CAN_STD_ID_MASK) << 21);
      }

      // Remote frame
      if (type == REMOTE_FRAME) {
          out |= STM32_CAN_TIR_RTR;
      }

      return out;
    };

    static uint16_t makeFilter16(uint32_t id, uint8_t format, uint8_t type)
    {
      uint16_t out = 0;
      if (format == EXTENDED_FORMAT) {      // Extended frame format
          out |= (uint16_t)((id & 0x0002800) >> 15) | 0x0008;
      }

      // Standard frame format (and Extended)
      out |= (uint16_t)((id & CAN_STD_ID_MASK) << 5);

      // Remote frame
      if (type == REMOTE_FRAME) {
          out |= 0x0010;
      }

      return out;
    };


  protected:
    void setFilter(CAN_filter_t &item);
    void updateFilters(void);
    void setGPIO(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3);
    int getTXMailbox(void);
    int getRXMailbox(void);
    inline uint32_t __bswap32(uint32_t x);

    bool _initialized;
    CAN_filter_t *_filters;
    int _filter_count;
    HardwareSerial *_serial;
};

#endif
