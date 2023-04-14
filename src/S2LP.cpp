/**
 ******************************************************************************
 * @file    S2LP.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Implementation of a S2-LP sub-1GHz transceiver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "arduino.h"
#include "mux.h"

#include "S2LP.h"

/* Defines -------------------------------------------------------------------*/
#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define S2LP_CMD_SIZE   2

#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/

#define LINEAR_FIFO_ADDRESS		0xFF  /*!< Linear FIFO address*/

/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param spi object of the instance of the spi peripheral
 * @param csn the spi chip select pin
 * @param sdn the shutdown pin
 * @param irqn the pin to receive IRQ event
 * @param frequency the base carrier frequency in Hz
 * @param xtalFrequency the crystal oscillator frequency
 * @param paInfo information about external power amplifier
 * @param irq_gpio the S2-LP gpio used to receive the IRQ events
 * @param my_addr address of the device
 * @param multicast_addr multicast address
 * @param broadcast_addr broadcast address
 *
S2LP::S2LP(SPIClass *spi, int csn, int sdn, int irqn, uint32_t frequency, uint32_t xtalFrequency, PAInfo_t paInfo, S2LPGpioPin irq_gpio, uint8_t my_addr, uint8_t multicast_addr, uint8_t broadcast_addr)
 : dev_spi(spi), csn_pin(csn), sdn_pin(sdn), irq_pin(irqn), 
 lFrequencyBase(frequency), s_lXtalFrequency(xtalFrequency), 
 s_paInfo(paInfo), irq_gpio_selected(irq_gpio), my_address(my_addr), multicast_address(multicast_addr), broadcast_address(broadcast_addr)
{
  Callback<void()>::func = std::bind(&S2LP::S2LPIrqHandler, this);
  irq_handler = static_cast<S2LPEventHandler>(Callback<void()>::callback);
  memset((void *)&g_xStatus, 0, sizeof(S2LPStatus));
  s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;
  current_event_callback = NULL;
  nr_of_irq_disabled = 0;
  xTxDoneFlag = S_RESET;
  memset((void *)vectcRxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  memset((void *)vectcTxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  cRxData = 0;
  is_waiting_for_read = false;
  is_bypass_enabled = false;
}
*/
S2LP::S2LP(eS2lpId s2lpId):
  _s2lpId(s2lpId)

{

  dev_spi = &SPI;
  csn_pin= 6;
  sdn_pin = -1;
  irq_pin = 3;
  lFrequencyBase = 433920000; 
  s_lXtalFrequency = 50000000;
  
  s_paInfo.paRfRangeExtender=RANGE_EXT_NONE;
  s_paInfo.paSignalCSD_S2LP={S2LP_GPIO_0, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_TX_RX_MODE}; 
  s_paInfo.paSignalCPS_S2LP={S2LP_GPIO_1, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_RX_STATE}; 
  s_paInfo.paSignalCTX_S2LP={S2LP_GPIO_2, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_TX_STATE};
  s_paInfo.paSignalCSD_MCU=A0;
  s_paInfo.paSignalCPS_MCU=A2;
  s_paInfo.paSignalCTX_MCU=A3;
  s_paInfo.paLevelValue=0x25;
  
  irq_gpio_selected = S2LP_GPIO_3;
  my_address = 0x44;
  multicast_address = 0xEE;
  broadcast_address = 0xFF;
  
  Callback<void()>::func = NULL; //std::bind(&S2LP::S2LPIrqHandler, this);
  irq_handler = NULL; //static_cast<S2LPEventHandler>(Callback<void()>::callback);
  memset((void *)&g_xStatus, 0, sizeof(S2LPStatus));
  s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;
  current_event_callback = NULL;
  nr_of_irq_disabled = 0;
  xTxDoneFlag = S_RESET;
  memset((void *)vectcRxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  memset((void *)vectcTxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  cRxData = 0;
  is_waiting_for_read = false;
  is_bypass_enabled = false;
  
  
  
}
//void S2LP::setSPI( SPIClass* spiNew, int cs ){
//	dev_spi = (SPIClass*)spiNew;
//	csn_pin = cs;
//}
void S2LP::beginAri(uint32_t lFrequencyBase, uint32_t ldataRate, uint32_t lfreqDeviation, uint32_t lBandwidth, 
                    uint8_t lPreambleLenBit, uint8_t lPreambleType, uint8_t lSyncLenBit, uint32_t lSyncWord, int lRssiThreshdBm, int packetLen)
{
	uint8_t tmp = 0x00;
	//Reset uses PCA, so it is handled outside of this lib
	// GPIO are also set outside.

  /* S2-LP soft reset */
  S2LPCmdStrobeSres();

  delay(10); 

  SRadioInit xRadioInit = {
    lFrequencyBase, /* base carrier frequency */
    MOD_2FSK,       /* modulation type */
    ldataRate,      /* data rate */
    lfreqDeviation, /* frequency deviation */
    lBandwidth      /* bandwidth */
  };

  S2LPRadioInit(&xRadioInit);
  SerialUSB.print("Setting lFrequencyBase ");SerialUSB.println(lFrequencyBase);
  SerialUSB.print("Setting ldataRate ");SerialUSB.println(ldataRate);
  SerialUSB.print("Setting lfreqDeviation ");SerialUSB.println(lfreqDeviation);
  SerialUSB.print("Setting lBandwidth ");SerialUSB.println(lBandwidth);

  S2LPRadioSetMaxPALevel(S_DISABLE);

  if(s_paInfo.paRfRangeExtender == RANGE_EXT_NONE)
  {
    S2LPRadioSetPALeveldBm(7, 12);
  } else
  {
    /* in case we are using a board with external PA, the S2LPRadioSetPALeveldBm will be not functioning because
    the output power is affected by the amplification of this external component.
    Set the raw register. */
    /* For example, paLevelValue=0x25 will give about 19dBm on a STEVAL FKI-915V1 */
    S2LPSpiWriteRegisters(PA_POWER8_ADDR, 1, &s_paInfo.paLevelValue);

    if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SE2435L)
    {
      S2LPGpioInit(&s_paInfo.paSignalCSD_S2LP);
      S2LPGpioInit(&s_paInfo.paSignalCPS_S2LP);
      S2LPGpioInit(&s_paInfo.paSignalCTX_S2LP);
    } else if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SKY66420)
    {
      pinMode(s_paInfo.paSignalCSD_MCU, OUTPUT);
      pinMode(s_paInfo.paSignalCPS_MCU, OUTPUT);
      pinMode(s_paInfo.paSignalCTX_MCU, OUTPUT);
    }
  }

  S2LPRadioSetPALevelMaxIndex(7);

  PktBasicInit xBasicInit={
    lPreambleLenBit,    /* Preamble length */
    lSyncLenBit,        /* Sync length */
    lSyncWord,          /* Sync word */
    S_DISABLE,          /* Variable length */
    S_DISABLE,          /* Extended length field */
    PKT_NO_CRC,         /* CRC mode */
    S_DISABLE,          /* Enable address */
    S_DISABLE,          /* Enable FEC */
    S_DISABLE           /* Enable Whitening */
  };

  S2LPPktBasicInit(&xBasicInit);
  Serial.print("Setting lPreambleLenBit ");Serial.println(lPreambleLenBit);
  Serial.print("Setting lSyncLenBit ");Serial.println(lSyncLenBit);
  Serial.print("Setting lSyncWord ");Serial.println(lSyncWord,HEX);



  PktBasicAddressesInit xAddressInit={
    S_DISABLE,          /* Filtering my address */
    0x00,        /* My address */
    S_DISABLE,          /* Filtering multicast address */
    0x00, /* Multicast address */
    S_DISABLE,          /* Filtering broadcast address */
    0x00  /* broadcast address */
  };

  S2LPPktBasicAddressesInit(&xAddressInit);

  //S2LPPacketHandlerSetRxPersistentMode(S_DISABLE);

  SRssiInit xSRssiInit = {
    .cRssiFlt = 14,
    .xRssiMode = RSSI_DYNAMIC_6DB_STEP_MODE, //RSSI_STATIC_MODE,  //RSSI_DYNAMIC_6DB_STEP_MODE
    .cRssiThreshdBm = lRssiThreshdBm,
  };
  S2LPRadioRssiInit(&xSRssiInit);
  Serial.print("Setting Rssi Threshold dBm ");Serial.println(lRssiThreshdBm);

  S2LPManagementRcoCalibration();  //TODO LOOP 10 time untill success??? cannot find S2LPManagementRcoCalibration in Arduino project

  /* disable CSMA and its filtering*/
  tmp = 0x00;
  S2LPSpiWriteRegisters(0x3A, 1, &tmp);
  
  /* Enable PQI */
  //S2LPRadioSetPqiCheck(0x00);
  S2LPRadioSetPqiCheck(S_DISABLE); //S_ENABLE);

  /* S2LP IRQs enable */
  //S2LPGpioIrqDeInit(NULL);
  //S2LPGpioIrqConfig(RX_DATA_READY,S_DISABLE);
  //S2LPGpioIrqConfig(TX_DATA_SENT , S_DISABLE);
  
  //RX_TIMEOUT_AND_OR_SELECT CS_TIMEOUT_MASK SQI_TIMEOUT_MASK PQI_TIMEOUT_MASK 
  //1  (0x40=4)                       0              0                0
  //The RX timeout cannot be stopped. It starts at the RX state and at the end expires
  tmp = 0x04;
  S2LPSpiWriteRegisters(0x39, 1, &tmp);  

  /* clear FIFO if needed */
  S2LPCmdStrobeFlushRxFifo();

  /* Set infinite Timeout */
  S2LPTimerSetRxTimerCounter(0);
  S2LPTimerSetRxTimerStopCondition(ANY_ABOVE_THRESHOLD);

  /* IRQ registers blanking */
  S2LPGpioIrqClearStatus();

  tmp = 0x90;
  S2LPSpiWriteRegisters(0x76, 1, &tmp);


  // Force preamble type :
  if( lPreambleType < 4 ){
	  //PCKTCTRL3 2E 2 low bits
	  S2LPSpiReadRegisters(0x2E, 1, &tmp );
	  tmp = (tmp&0xFC) | (lPreambleType&0x03);
	  S2LPSpiWriteRegisters(0x2E, 1, &tmp );
	  Serial.print("Setting preamble ");Serial.println(tmp);
  }

  if( packetLen > 0 ){
    Serial.print("Setting FIXED packet length ");Serial.println(packetLen);
	  S2LPSpiReadRegisters(0x2F, 1, &tmp );
	  tmp = tmp & 0xFE;
    S2LPSpiWriteRegisters(0x2F, 1, &tmp ); // Fixed packet len
    tmp = 0;
    S2LPSpiWriteRegisters(0x31, 1, &tmp );
    tmp = packetLen;
    S2LPSpiWriteRegisters(0x32, 1, &tmp );
  }

  /* Go to RX state */
  //S2LPCmdStrobeCommand(CMD_RX);
  Serial.println("Configuration done at begin time");

  //attachInterrupt(irq_pin, irq_handler, FALLING);
}




/**
* @brief  DeInitialize the S2-LP library.
* @param  None.
* @retval None.
*/
void S2LP::end(void)
{
  /* Shutdown the S2-LP */
  digitalWrite(sdn_pin, HIGH);
  delay(1);
  digitalWrite(sdn_pin, LOW);
  delay(1);

  /* S2-LP soft reset */
  S2LPCmdStrobeSres();

  /* Detach S2-LP IRQ */
  detachInterrupt(irq_pin);

  /* Reset CSN pin */
  pinMode(csn_pin, INPUT);

  /* Reset SDN pin */
  pinMode(sdn_pin, INPUT);

  /* Reset CSD, CPS and CTX if it is needed */
  if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SKY66420)
  {
    pinMode(s_paInfo.paSignalCSD_MCU, INPUT);
    pinMode(s_paInfo.paSignalCPS_MCU, INPUT);
    pinMode(s_paInfo.paSignalCTX_MCU, INPUT);
  }

  /* Reset all internal variables */  
  memset((void *)&g_xStatus, 0, sizeof(S2LPStatus));
  s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;
  current_event_callback = NULL;
  nr_of_irq_disabled = 0;
  xTxDoneFlag = S_RESET;
  memset((void *)vectcRxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  memset((void *)vectcTxBuff, 0, FIFO_SIZE*sizeof(uint8_t));
  cRxData = 0;
  is_waiting_for_read = false;
  is_bypass_enabled = false;
}

/**
* @brief  Attach the callback for Receive event.
* @param  func the Receive callback.
* @retval None.
*/
void S2LP::attachS2LPReceive(S2LPEventHandler func)
{
  current_event_callback = func;
}

/**
* @brief  Send the payload packet.
* @param  payload pointer to the data to be sent.
* @param  payload_len length in bytes of the payload data.
* @param  dest_addr destination address.
* @param  use_csma_ca should CSMA/CA be enabled for transmission.
* @retval zero in case of success, non-zero error code otherwise.
* @note  the maximum payload size allowed is 128 bytes
*/
uint8_t S2LP::send(uint8_t *payload, uint8_t payload_len, uint8_t dest_addr, bool use_csma_ca)
{
  uint32_t start_time;
  uint32_t current_time;

  if(payload_len > FIFO_SIZE)
  {
    return 1;
  }

  disableS2LPIrq();

  /* Go to Ready mode */
  if(S2LPSetReadyState())
  {
    enableS2LPIrq();
    return 1;
  }

  S2LPPktBasicSetPayloadLength(payload_len);

  S2LPSetRxSourceReferenceAddress(dest_addr);

  if(use_csma_ca)
  {
    S2LPCsma(S_ENABLE);
  }

  /* Flush TX FIFO */
  S2LPCmdStrobeCommand(CMD_FLUSHTXFIFO);

  memcpy(vectcTxBuff, payload, payload_len);

  S2LPSpiWriteFifo(payload_len, vectcTxBuff);

  /* IRQ registers blanking */
  S2LPGpioIrqClearStatus();

  enableS2LPIrq();

  uint8_t tmp=0x9C;
  S2LPSpiWriteRegisters(0x76,1,&tmp);

  if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SKY66420)
  {
    FEM_Operation_SKY66420(FEM_TX);
  }

  S2LPCmdStrobeCommand(CMD_TX);

  start_time = millis();

  /* wait for TX done */
  do
  {
    current_time = millis();
  } while(!xTxDoneFlag && (current_time - start_time) <= 1000);

  if(use_csma_ca)
  {
    S2LPCsma(S_DISABLE);
  }

  if(!is_waiting_for_read)
  {
    uint8_t tmp = 0x90;
    S2LPSpiWriteRegisters(0x76, 1, &tmp);

    if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SKY66420)
    {
      FEM_Operation_SKY66420(FEM_RX);
    }

    /* Return to RX state */
    S2LPCmdStrobeCommand(CMD_RX);
  }

  disableS2LPIrq();

  /* Timeout case */
  if(!xTxDoneFlag)
  {
    enableS2LPIrq();
    return 1;
  }

  xTxDoneFlag = S_RESET;
  enableS2LPIrq();

  return 0;
}

/**
* @brief  Get the payload size of the received packet.
* @param  None.
* @retval the payload size of the received packet.
*/
uint8_t S2LP::getRecvPayloadLen(void)
{
  return cRxData;
}

/**
* @brief  Read the payload packet.
* @param  payload pointer to the data to be sent.
* @param  payload_len length in bytes of the payload data.
* @retval the number of read bytes.
* @note  the maximum payload size allowed is 128 bytes
*/
uint8_t S2LP::read(uint8_t *payload, uint8_t payload_len)
{
  uint8_t ret_val = cRxData;

  disableS2LPIrq();

  if(payload_len < cRxData)
  {
    enableS2LPIrq();
    return 0;
  }

  memcpy(payload, vectcRxBuff, cRxData);

  cRxData = 0;

  is_waiting_for_read = false;

  uint8_t tmp = 0x90;
  S2LPSpiWriteRegisters(0x76, 1, &tmp);

  if(s_paInfo.paRfRangeExtender == RANGE_EXT_SKYWORKS_SKY66420)
  {
    FEM_Operation_SKY66420(FEM_RX);
  }

  /* Return to RX state */
  S2LPCmdStrobeCommand(CMD_RX);

  enableS2LPIrq();

  return ret_val;
}

/**
* @brief  Sets the channel number.
* @param  cChannel the channel number.
* @retval None.
*/
void S2LP::setRadioChannel(uint8_t cChannel)
{
  return S2LPRadioSetChannel(cChannel);
}

/**
* @brief  Returns the actual channel number.
* @param  None.
* @retval uint8_t Actual channel number.
*/
uint8_t S2LP::getRadioChannel(void)
{
  return S2LPRadioGetChannel();
}

/**
* @brief  Set the channel space factor in channel space register.
*         The channel spacing step is computed as F_Xo/32768.
* @param  fChannelSpace the channel space expressed in Hz.
* @retval None.
*/
void S2LP::setRadioChannelSpace(uint32_t lChannelSpace)
{
  return S2LPRadioSetChannelSpace(lChannelSpace);
}

/**
* @brief  Return the channel space register.
* @param  None.
* @retval uint32_t Channel space. The channel space is: CS = channel_space_factor x XtalFrequency/2^15
*         where channel_space_factor is the CHSPACE register value.
*/
uint32_t S2LP::getRadioChannelSpace(void)
{
  return S2LPRadioGetChannelSpace();
}

/**
* @brief  Set the Ready state.
* @param  None.
* @retval zero in case of success, non-zero error code otherwise.
*/
uint8_t S2LP::S2LPSetReadyState(void)
{
  uint8_t ret_val = 0;
  uint32_t start_time;
  uint32_t current_time;

  S2LPCmdStrobeCommand(CMD_SABORT);

  start_time = millis();

  do
  {
    S2LPRefreshStatus();

    current_time = millis();
  } while(g_xStatus.MC_STATE != MC_STATE_READY && (current_time - start_time) <= 1000);

  if((current_time - start_time) > 1000)
  {
    ret_val = 1;
  }

  return ret_val;
}

S2LPCutType S2LP::S2LPManagementGetCut(void)
{
  uint8_t tmp;

  /* Read Cut version from S2LP register */
  S2LPSpiReadRegisters(0xF1, 1, &tmp);
  return (S2LPCutType)tmp;
}

/**
* @brief  IRQ Handler.
* @param  None.
* @retval None.
*/
void S2LP::S2LPIrqHandler(void)
{
	bool res = false;
  S2LPIrqs xIrqStatus;

  /* Get the IRQ status */
  S2LPGpioIrqGetStatus(&xIrqStatus);

  /* Check the SPIRIT TX_DATA_SENT IRQ flag */
  if(xIrqStatus.IRQ_TX_DATA_SENT)
  {
	  Serial.println("Found Data Tx IRQ");
    /* set the tx_done_flag to manage the event in the send() */
    xTxDoneFlag = S_SET;
  }

  /* Check the S2LP RX_DATA_READY IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_READY)
  {
	S2LPSpiReadRegisters(0xA2, 1, &lastestRssi ); // RSSI_LEVEL
	  
    /* Get the RX FIFO size */
    cRxData = S2LPFifoReadNumberBytesRxFifo();

	//Serial.print("Found Data Rx IRQ : bytes ");
    //Serial.println(cRxData);
    /* Read the RX FIFO */
    S2LPSpiReadFifo(cRxData, vectcRxBuff);

    /* Flush the RX FIFO */
    S2LPCmdStrobeFlushRxFifo();

    is_waiting_for_read = true;

    /* Call application callback */
    if(current_event_callback)
    {
      current_event_callback();
    }
  }
}

/**
* @brief  Disable the S2LP interrupts.
* @param  None.
* @retval None.
*/
void S2LP::disableS2LPIrq(void)
{
  if(nr_of_irq_disabled == 0)
  {
    detachInterrupt(irq_pin);
  }
  nr_of_irq_disabled++;
}

/**
* @brief  Enable the S2LP interrupts.
* @param  None.
* @retval None.
*/
void S2LP::enableS2LPIrq(void)
{
  //ARI: no interrupt allowed !
  return;
//  if(nr_of_irq_disabled > 0)
//  {
//    nr_of_irq_disabled--;
//    if(nr_of_irq_disabled == 0)
//    {
//      attachInterrupt(irq_pin, irq_handler, FALLING);
//    }
//  }
}

/**
* @brief  Commands for external PA of type SKY66420.
* @param  operation the command to be executed.
* @retval None.
*/
void S2LP::FEM_Operation_SKY66420(FEM_OperationType operation)
{
  switch (operation)
  {
    case FEM_SHUTDOWN:
    {
      /* Puts CSD high to turn on PA */
      digitalWrite(s_paInfo.paSignalCSD_MCU, LOW);

      /* Puts CTX high to go in TX state DON'T CARE */
      digitalWrite(s_paInfo.paSignalCTX_MCU, HIGH);

      /* No Bypass mode select DON'T CARE */
      digitalWrite(s_paInfo.paSignalCPS_MCU, HIGH);

      break;
    }
    case FEM_TX_BYPASS:
    {
      /* Puts CSD high to turn on PA */
      digitalWrite(s_paInfo.paSignalCSD_MCU, HIGH);

      /* Puts CTX high to go in TX state */
      digitalWrite(s_paInfo.paSignalCTX_MCU, HIGH);

      /* Bypass mode select */
      digitalWrite(s_paInfo.paSignalCPS_MCU, LOW);

      break;
    }
    case FEM_TX:
    {
      /* Puts CSD high to turn on PA */
      digitalWrite(s_paInfo.paSignalCSD_MCU, HIGH);

      /* Puts CTX high to go in TX state */
      digitalWrite(s_paInfo.paSignalCTX_MCU, HIGH);

      /* No Bypass mode select DON'T CARE */
      digitalWrite(s_paInfo.paSignalCPS_MCU, HIGH);

      break;
    }
    case FEM_RX:
    {
      /* Puts CSD high to turn on PA */
      digitalWrite(s_paInfo.paSignalCSD_MCU, HIGH);

      /* Puts CTX low */
      digitalWrite(s_paInfo.paSignalCTX_MCU, LOW);

      /* Check Bypass mode */
      if (is_bypass_enabled)
      {
        digitalWrite(s_paInfo.paSignalCPS_MCU, LOW);
      } else
      {
        digitalWrite(s_paInfo.paSignalCPS_MCU, HIGH);
      }

      break;
    }
    default:
      /* Error */
      break;
  }
}

/**
* @brief  Write single or multiple registers.
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval Device status
*/ 
S2LPStatus S2LP::S2LPSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer )
{
  uint8_t header[S2LP_CMD_SIZE]={WRITE_HEADER,cRegAddress};
  S2LPStatus status;
  
  SpiSendRecv( header, pcBuffer, cNbBytes );

  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1]; 
  
  return status;
}

/**
* @brief  Read single or multiple registers.
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer )
{
  uint8_t header[S2LP_CMD_SIZE]={READ_HEADER,cRegAddress};
  S2LPStatus status;

  SpiSendRecv( header, pcBuffer, cNbBytes );

  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1]; 

  return status;
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t header[S2LP_CMD_SIZE]={COMMAND_HEADER,cCommandCode};
  S2LPStatus status;

  SpiSendRecv( header, NULL, 0 );
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}

/**
* @brief  Write data into TX FIFO.
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[S2LP_CMD_SIZE]={WRITE_HEADER,LINEAR_FIFO_ADDRESS};
  S2LPStatus status;

  SpiSendRecv( header, pcBuffer, cNbBytes );
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}


/**
* @brief  Read data from RX FIFO.
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval Device status
*/
S2LPStatus S2LP::S2LPSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[S2LP_CMD_SIZE]={READ_HEADER,LINEAR_FIFO_ADDRESS};
  S2LPStatus status;

  SpiSendRecv( header, pcBuffer, cNbBytes );
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}

void S2LP::csSelect() {
  switch (_s2lpId) {
    case S2LP::eS2lpId::eS2lp1:
      _Mux.s2lp1Cs();
      break;
    case S2LP::eS2lpId::eS2lp2:
      _Mux.s2lp2Cs();
      break;
     
    default:
      break;
  }
}

void S2LP::csUnselect() {
  switch (_s2lpId) {
    case S2LP::eS2lpId::eS2lp1:
      _Mux.s2lp1UnCs();
      break;
    case S2LP::eS2lpId::eS2lp2:
      _Mux.s2lp2UnCs();
      break;
     
    default:
      break;
  }
}

/**
* @brief  Basic SPI function to send/receive data.
* @param  pcHeader: pointer to header to be sent
* @param  pcBuffer: pointer to data to be sent/received
* @param  cNbBytes: number of bytes to send/receive
* @retval Device status
*/
void S2LP::SpiSendRecv(uint8_t *pcHeader, uint8_t *pcBuffer, uint16_t cNbBytes)
{
  disableS2LPIrq();
  //ARI :Reduced speed because bus is electricaly heavily loaded
  dev_spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  csSelect();

  dev_spi->transfer(pcHeader, S2LP_CMD_SIZE);

  if(cNbBytes)
  {
    dev_spi->transfer(pcBuffer, cNbBytes);
  }

  csUnselect();

  enableS2LPIrq();
}

/**
* @brief  Management of RCO calibration.
* @param  None.
* @retval None.
*/
void S2LP::S2LPManagementRcoCalibration(void)
{
  uint8_t tmp[2],tmp2;
  SerialUSB.print("S2LP Starting VCO Calibration...");
  uint32_t start = millis();
  
  S2LPSpiReadRegisters(0x6D, 1, &tmp2);
  tmp2 |= 0x01;
  S2LPSpiWriteRegisters(0x6D, 1, &tmp2);

  S2LPSpiCommandStrobes(0x63);
  delay(100);
  S2LPSpiCommandStrobes(0x62);

  do
  {
    S2LPSpiReadRegisters(0x8D, 1, tmp);
  }
  while((tmp[0]&0x10)==0   &&  ( millis()-start < 10000));
  if( (tmp[0]&0x10)==0 ){
	  SerialUSB.println("Failed to calibrate RCO !");
  }

  S2LPSpiReadRegisters(0x94, 2, tmp);
  S2LPSpiReadRegisters(0x6F, 1, &tmp2);
  tmp[1]=(tmp[1]&0x80)|(tmp2&0x7F);

  S2LPSpiWriteRegisters(0x6E, 2, tmp);
  S2LPSpiReadRegisters(0x6D, 1, &tmp2);
  tmp2 &= 0xFE;

  S2LPSpiWriteRegisters(0x6D, 1, &tmp2);
  SerialUSB.print("S2LP VCO Calibration Done");
}

void S2LP::S2LPDumpConfig() {
  uint8_t c;
  if (1) {
    for (uint8_t i = 0; i < 0xFF; i++) {
      S2LPSpiReadRegisters(i,1,&c);
      if (i < 16) { SerialUSB.print("0"); }
      SerialUSB.print(i,HEX);
      if (c < 16) { SerialUSB.print("0"); }
      SerialUSB.print(c,HEX);
      SerialUSB.print(",");
    }
    SerialUSB.println();
  }
}
