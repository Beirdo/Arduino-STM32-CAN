#include <Arduino.h>
#include "STM32-CAN.h"

CAN_bit_timing_config_t can_configs[6] = {
  {2, 13, 60}, 
  {2, 13, 30}, 
  {2, 13, 24}, 
  {2, 13, 12}, 
  {2, 13, 6}, 
  {2, 13, 3}
};

CAN_filter_t default_filters[] = {
  // Set filter 0
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  {0, 1, 0, 0, 0, 0},
};
int default_filter_count = sizeof(default_filters) / sizeof(default_filters[0]);


/**
 * Print registers.
*/ 
void STM32_CAN::printRegister(const char *buf, uint32_t reg)
{
#ifdef DEBUG_STM32_CAN
  Serial.print(buf);
  Serial.print("0x");
  Serial.print(reg, HEX);
  Serial.println();
#endif
}

/**
 * Initializes the CAN GPIO registers.
 *
 * @params: addr    - Specified GPIO register address.
 * @params: index   - Specified GPIO index.
 * @params: afry    - Specified Alternative function selection AF0-AF15.
 * @params: speed   - Specified OSPEEDR register value.(Optional)
 *
 */
void STM32_CAN::setGPIO(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed)
{
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7) {
      _index4 = (index - 8) * 4;
      ofs = 1;
    }

    uint32_t mask;
    printRegister("GPIO_AFR(b)=", addr->AFR[1]);
    mask = 0xF << _index4;
    addr->AFR[ofs]  &= ~mask;         // Reset alternate function
    //setting = 0x9;                    // AF9
    setting = afry;                   // Alternative function selection
    mask = setting << _index4;
    addr->AFR[ofs]  |= mask;          // Set alternate function
    printRegister("GPIO_AFR(a)=", addr->AFR[1]);

    printRegister("GPIO_MODER(b)=", addr->MODER);
    mask = 0x3 << _index2;
    addr->MODER   &= ~mask;           // Reset mode
    setting = 0x2;                    // Alternate function mode
    mask = setting << _index2;
    addr->MODER   |= mask;            // Set mode
    printRegister("GPIO_MODER(a)=", addr->MODER);

    printRegister("GPIO_OSPEEDR(b)=", addr->OSPEEDR);
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask;           // Reset speed
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask;            // Set speed
    printRegister("GPIO_OSPEEDR(a)=", addr->OSPEEDR);

    printRegister("GPIO_OTYPER(b)=", addr->OTYPER);
    mask = 0x1 << index;
    addr->OTYPER  &= ~mask;           // Reset Output push-pull
    printRegister("GPIO_OTYPER(a)=", addr->OTYPER);

    printRegister("GPIO_PUPDR(b)=", addr->PUPDR);
    mask = 0x3 << _index2;
    addr->PUPDR   &= ~mask;           // Reset port pull-up/pull-down
    printRegister("GPIO_PUPDR(a)=", addr->PUPDR);
}


/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: item    - CAN_filter definition for this filter
 *
 */
void STM32_CAN::setFilter(CAN_filter_t &item)
{
  if (item.index > 13) {
    return;
  }

  uint32_t bit = 1 << item.index;

  CAN->FA1R &= ~bit;                  // Deactivate filter

  if (item.scale == 0) {
    CAN->FS1R &= ~bit;                // Set filter to Dual 16-bit scale configuration
  } else {
    CAN->FS1R |= bit;                 // Set filter to single 32 bit configuration
  }

  if (item.mode == 0) {
    CAN->FM1R &= ~bit;                // Set filter to Mask mode
  } else {
    CAN->FM1R |= bit;                 // Set filter to List mode
  }

  if (item.fifo == 0) {
    CAN->FFA1R &= ~bit;               // Set filter assigned to FIFO 0
  } else {
    CAN->FFA1R |= bit;                // Set filter assigned to FIFO 1
  }

  CAN->sFilterRegister[item.index].FR1 = item.bank1;    // Set filter bank registers1
  CAN->sFilterRegister[item.index].FR2 = item.bank2;    // Set filter bank registers2

  CAN->FA1R |= bit;                   // Activate filter
}


void STM32_CAN::setFilters(CAN_filter_t *filters, int count)
{
  _filters = filters;
  _filter_count = count;

  // This allows us to setup the filters before we call begin()
  if (_initialized) {
    updateFilters();
  }
}

void STM32_CAN::updateFilters(void)
{
  // This will only run once the controller is configured to the point where it's
  // ready for filters.

  if (!_filters || !_filter_count) {
    _filters = default_filters;
    _filter_count = default_filter_count;
  }

  CAN->FMR |= 0x1;                    // Set to filter initialization mode
  CAN->FA1R = 0;                      // Disable any old filters

  CAN_filter_t *item = _filters;
  for (int i = 0; i < _filter_count && i < 14; i++, item++) {
    setFilter(*item);
  }

  CAN->FMR &= ~(0x1);                 // Deactivate initialization mode
}

    
/**
 * Initializes the CAN controller with specified bit rate.
 *
 * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
 *
 */
bool STM32_CAN::begin(BITRATE bitrate)
{
  // Reference manual
  // https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

  RCC->APB1ENR |= 0x2000000;           // Enable CAN clock 

  RCC->AHBENR |= 0x20000;              // Enable GPIOA clock 
  setGPIO(GPIOA, 11, AF4);             // Set PA11 to AF4
  setGPIO(GPIOA, 12, AF4);             // Set PA12 to AF4

  CAN->MCR |= 0x1;                     // Set CAN to Initialization mode 
  while (!(CAN->MSR & 0x1));           // Wait for Initialization mode

  //CAN->MCR = 0x51;                   // Hardware initialization (No automatic retransmission)
  CAN->MCR = 0x41;                     // Hardware initialization (With automatic retransmission)

  // Set bit rates 
  CAN->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF)); 
  CAN->BTR |=  (((can_configs[bitrate].TS2 - 1) & 0x07) << 20) | 
               (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16) | 
               ((can_configs[bitrate].BRP - 1) & 0x3FF);
  printRegister("CAN->BTR=", CAN->BTR);

  _initialized = true;

  // Configure Filters (use defaults if not already set)
  updateFilters();

  uint16_t TimeoutMilliseconds = 1000;
  bool can1 = false;
  CAN->MCR   &= ~(0x1);              // Require CAN1 to normal mode 

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if (!(CAN->MSR & 0x1)) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }

  if (can1) {
    Serial.println("CAN1 initialize ok");
  } else {
    Serial.println("CAN1 initialize fail!!");
    return false;
  }
  return true; 
}


int STM32_CAN::getRXMailbox(void)
{
  if (CAN->RF0R & 0x03) {
    return 0;
  }

  if (CAN->RF1R & 0x03) {
    return 1;
  }

  return -1;
}

/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
bool STM32_CAN::receive(CAN_msg_t* CAN_rx_msg)
{
  // we have 2 fifos with 3 messages each.  We want to service both
  int mailbox = getRXMailbox();
  if (mailbox == -1) {
    return false;
  }

  uint32_t id = CAN->sFIFOMailBox[mailbox].RIR;
  if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
      CAN_rx_msg->format = STANDARD_FORMAT;
      CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21));
  } else {                             // Extended frame format
      CAN_rx_msg->format = EXTENDED_FORMAT;
      CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3));
  }

  if ((id & STM32_CAN_RIR_RTR) == 0) { // Data frame
      CAN_rx_msg->type = DATA_FRAME;
  } else {                             // Remote frame
      CAN_rx_msg->type = REMOTE_FRAME;
  }

  
  CAN_rx_msg->len = (CAN->sFIFOMailBox[mailbox].RDTR) & 0x0F;
  
  uint32_t data;

  data = __bswap32(CAN->sFIFOMailBox[mailbox].RDLR);
  *(uint32_t *)&CAN_rx_msg->data[0] = data;

  data = __bswap32(CAN->sFIFOMailBox[mailbox].RDHR);
  *(uint32_t *)&CAN_rx_msg->data[4] = data;

  // Release FIFO output mailbox.
  // Make the next incoming message available, clear flags
  if (mailbox == 0) {
    CAN->RF0R |= 0x38;
  } else {
    CAN->RF1R |= 0x38;
  }

  return true;
}

inline uint32_t STM32_CAN::__bswap32(uint32_t x)
{
  return ((x & 0xFF000000) >> 24) | ((x && 0x00FF0000) >> 8) | ((x & 0x0000FF00) << 8) | ((x & 0x000000FF) << 24);
}
 
/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @params CAN_tx_msg - CAN message structure for transmission
 * 
 */
bool STM32_CAN::send(CAN_msg_t* CAN_tx_msg)
{
  uint32_t out = makeFilter32(CAN_tx_msg->id, CAN_tx_msg->format, CAN_tx_msg->type);

  // Wait until a mailbox is empty
  int mailbox;
  int count = 0;
  while ((mailbox = getTXMailbox()) == -1 && count++ < 1000000);
  
  if (mailbox == -1) {
    // Timeout.  Something's borked.
    Serial.println("Send Fail");
    Serial.println(CAN->ESR);
    Serial.println(CAN->MSR);
    Serial.println(CAN->TSR);
    return false;
  }

  CAN->sTxMailBox[mailbox].TDTR &= ~(0x0F);
  CAN->sTxMailBox[mailbox].TDTR |= CAN_tx_msg->len & 0x0F;
  
  uint32_t data;
  data = __bswap32(*(uint32_t *)&CAN_tx_msg->data[0]);
  CAN->sTxMailBox[mailbox].TDLR  = data;

  data = __bswap32(*(uint32_t *)&CAN_tx_msg->data[4]);
  CAN->sTxMailBox[mailbox].TDHR  = data;

  // Send Go
  CAN->sTxMailBox[mailbox].TIR = out | STM32_CAN_TIR_TXRQ;

  return true;
}


int STM32_CAN::getTXMailbox(void)
{
  // We have 3 mailboxes.  Find an empty one if possible
  for (int i = 0; i < 3; i++) {
    if (!(CAN->sTxMailBox[0].TIR & 0x01)) {
      return i;
    }
  }

  return -1;
}


/**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 *
 */
int STM32_CAN::available(void)
{
  // Check for pending FIFO 0 messages
  int count = 0;
  count += CAN->RF0R & 0x03;
  count += CAN->RF1R & 0x03;
  return count;
}



#if 0
void setup() {
  Serial.begin(115200);
  bool ret = CANInit(CAN_1000KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  //bool ret = CANInit(CAN_1000KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_1000KBPS, 3);  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
  if (!ret) while(true);
}

void loop() {
  CAN_msg_t CAN_TX_msg;
  CAN_msg_t CAN_RX_msg;
   
  CAN_TX_msg.data[0] = 0x00;
  CAN_TX_msg.data[1] = 0x01;
  CAN_TX_msg.data[2] = 0x02;
  CAN_TX_msg.data[3] = 0x03;
  CAN_TX_msg.data[4] = 0x04;
  CAN_TX_msg.data[5] = 0x05;
  CAN_TX_msg.data[6] = 0x06;
  CAN_TX_msg.data[7] = 0x07;
  CAN_TX_msg.len = frameLength;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if ( ( counter % 2) == 0) {
      CAN_TX_msg.type = DATA_FRAME;
      if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.id = 0x32F072;
    } else {
      CAN_TX_msg.type = DATA_FRAME;
      if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
      CAN_TX_msg.format = STANDARD_FORMAT;
      CAN_TX_msg.id = 0x072;
    }
    CANSend(&CAN_TX_msg);
    frameLength++;
    if (frameLength == 9) frameLength = 0;
    counter++;
  }
  
  if(CANMsgAvail()) {
    CANReceive(&CAN_RX_msg);

    if (CAN_RX_msg.format == EXTENDED_FORMAT) {
      Serial.print("Extended ID: 0x");
      if (CAN_RX_msg.id < 0x10000000) Serial.print("0");
      if (CAN_RX_msg.id < 0x1000000) Serial.print("00");
      if (CAN_RX_msg.id < 0x100000) Serial.print("000");
      if (CAN_RX_msg.id < 0x10000) Serial.print("0000");
      Serial.print(CAN_RX_msg.id, HEX);
    } else {
      Serial.print("Standard ID: 0x");
      if (CAN_RX_msg.id < 0x100) Serial.print("0");
      if (CAN_RX_msg.id < 0x10) Serial.print("00");
      Serial.print(CAN_RX_msg.id, HEX);
      Serial.print("     ");
    }

    Serial.print(" DLC: ");
    Serial.print(CAN_RX_msg.len);
    if (CAN_RX_msg.type == DATA_FRAME) {
      Serial.print(" Data: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        Serial.print("0x"); 
        Serial.print(CAN_RX_msg.data[i], HEX); 
        if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println(" Data: REMOTE REQUEST FRAME");
    }
  }
}
#endif
