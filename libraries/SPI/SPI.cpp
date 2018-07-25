/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "SPI.h"
#include <Arduino.h>
#include <wiring_private.h>
#include <assert.h>

#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI,
		SercomSpiTXPad PadTx, SercomRXPad PadRx, int8_t dmaChannelRx, int8_t dmaChannelTx)
{
  initialized = false;
  assert(p_sercom != NULL);
  _p_sercom = p_sercom;

  // pins
  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;

  _dmaChannelRx = dmaChannelRx;
  _dmaChannelTx = dmaChannelTx;

  // SERCOM pads
  _padTx=PadTx;
  _padRx=PadRx;
}

void SPIClass::begin()
{
  init();

  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

  if(_dmaChannelRx > -1 && _dmaChannelTx > -1){
	descrx = &_descriptor[_dmaChannelRx];
	desctx = &_descriptor[_dmaChannelTx];
	descrx->BTCTRL.bit.VALID    = true;
	descrx->BTCTRL.bit.EVOSEL   = DMA_EVENT_OUTPUT_DISABLE;
	descrx->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_NOACT;
	descrx->BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_BYTE;
	descrx->BTCTRL.bit.SRCINC   = false;
	descrx->BTCTRL.bit.DSTINC   = true;
	descrx->BTCTRL.bit.STEPSEL  = DMA_STEPSEL_DST;
	descrx->BTCTRL.bit.STEPSIZE = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
	descrx->BTCNT.reg           = 0;
	descrx->SRCADDR.reg         = (uint32_t)0;

	desctx->BTCTRL.bit.VALID    = true;
	desctx->BTCTRL.bit.EVOSEL   = DMA_EVENT_OUTPUT_DISABLE;
	desctx->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_NOACT;
	desctx->BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_BYTE;
	desctx->BTCTRL.bit.SRCINC   = true;
	desctx->BTCTRL.bit.DSTINC   = false;
	desctx->BTCTRL.bit.STEPSEL  = DMA_STEPSEL_DST;
	desctx->BTCTRL.bit.STEPSIZE = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
	desctx->BTCNT.reg           = 0;
	desctx->SRCADDR.reg         = (uint32_t)0;

#ifdef __SAMD51__
	DMAC->Channel[_dmaChannelRx].CHCTRLA.bit.ENABLE  = 0;
	DMAC->Channel[_dmaChannelRx].CHCTRLA.bit.SWRST   = 1;
	DMAC->Channel[_dmaChannelTx].CHCTRLA.bit.ENABLE  = 0;
	DMAC->Channel[_dmaChannelTx].CHCTRLA.bit.SWRST   = 1;

	DMAC->Channel[_dmaChannelRx].CHPRILVL.bit.PRILVL = 0;
	DMAC->Channel[_dmaChannelRx].CHCTRLA.bit.TRIGACT = DMA_TRIGGER_ACTON_BEAT;
	DMAC->Channel[_dmaChannelRx].CHCTRLA.bit.BURSTLEN = DMAC_CHCTRLA_BURSTLEN_SINGLE_Val; // Single-beat burst length

	DMAC->Channel[_dmaChannelTx].CHPRILVL.bit.PRILVL = 0;
	DMAC->Channel[_dmaChannelTx].CHCTRLA.bit.TRIGACT = DMA_TRIGGER_ACTON_BEAT;
	DMAC->Channel[_dmaChannelTx].CHCTRLA.bit.BURSTLEN = DMAC_CHCTRLA_BURSTLEN_SINGLE_Val; // Single-beat burst length

	DMAC->SWTRIGCTRL.reg     &= ~(1UL << _dmaChannelTx);
	DMAC->SWTRIGCTRL.reg     &= ~(1UL << _dmaChannelRx);
#else
	DMAC->CHID.bit.ID         = _dmaChannelRx;
	DMAC->CHCTRLA.bit.ENABLE  = 0;
	DMAC->CHCTRLA.bit.SWRST   = 1;
	DMAC->CHCTRLB.bit.LVL     = 0;
	DMAC->CHCTRLB.bit.TRIGACT = DMA_TRIGGER_ACTON_BEAT;

	DMAC->CHID.bit.ID         = _dmaChannelTx;
	DMAC->CHCTRLA.bit.ENABLE  = 0;
	DMAC->CHCTRLA.bit.SWRST   = 1;
	DMAC->CHCTRLB.bit.LVL     = 0;
	DMAC->CHCTRLB.bit.TRIGACT = DMA_TRIGGER_ACTON_BEAT;
#endif

	_p_sercom->initSPIDMA(_dmaChannelRx, _dmaChannelTx, descrx, desctx);
  }

  config(DEFAULT_SPI_SETTINGS);
}

void SPIClass::init()
{
  if (initialized)
    return;
  interruptMode = SPI_IMODE_NONE;
  interruptSave = 0;
  interruptMask = 0;
  initialized = true;
}

void SPIClass::config(SPISettings settings)
{
  _p_sercom->disableSPI();

  _p_sercom->initSPI(_padTx, _padRx, SPI_CHAR_SIZE_8_BITS, settings.bitOrder);
  _p_sercom->initSPIClock(settings.dataMode, settings.clockFreq);

  _p_sercom->enableSPI();
}

void SPIClass::end()
{
  _p_sercom->resetSPI();
  initialized = false;
}

#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __interruptsStatus(void)
{
  // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/CHDBIBGJ.html
  return (__get_PRIMASK() ? 0 : 1);
}
#endif

void SPIClass::usingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EXTERNAL_INT_NMI))
    return;

  uint8_t irestore = interruptsStatus();
  noInterrupts();

  if (interruptNumber >= EXTERNAL_NUM_INTERRUPTS)
    interruptMode = SPI_IMODE_GLOBAL;
  else
  {
    interruptMode |= SPI_IMODE_EXTINT;
    interruptMask |= (1 << interruptNumber);
  }

  if (irestore)
    interrupts();
}

void SPIClass::beginTransaction(SPISettings settings)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      interruptSave = interruptsStatus();
      noInterrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
  }

  config(settings);
}

void SPIClass::endTransaction(void)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      if (interruptSave)
        interrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
  }
}

void SPIClass::setBitOrder(BitOrder order)
{
  if (order == LSBFIRST) {
    _p_sercom->setDataOrderSPI(LSB_FIRST);
  } else {
    _p_sercom->setDataOrderSPI(MSB_FIRST);
  }
}

void SPIClass::setDataMode(uint8_t mode)
{
  switch (mode)
  {
    case SPI_MODE0:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
      break;

    case SPI_MODE1:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
      break;

    case SPI_MODE2:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
      break;

    case SPI_MODE3:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
      break;

    default:
      break;
  }
}

void SPIClass::setClockDivider(uint8_t div)
{
  if (div < SPI_MIN_CLOCK_DIVIDER) {
    _p_sercom->setBaudrateSPI(SPI_MIN_CLOCK_DIVIDER);
  } else {
    _p_sercom->setBaudrateSPI(div);
  }
}

byte SPIClass::transfer(uint8_t data)
{
  return _p_sercom->transferDataSPI(data);
}

uint16_t SPIClass::transfer16(uint16_t data) {
  union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

  t.val = data;

  if (_p_sercom->getDataOrderSPI() == LSB_FIRST) {
    t.lsb = transfer(t.lsb);
    t.msb = transfer(t.msb);
  } else {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
  }

  return t.val;
}

void SPIClass::transfer(void *rx, void *tx, size_t count)
{
	if(count > 2 && _dmaChannelRx > -1 && _dmaChannelTx > -1){
		//use a synchronous DMA transfer
		descrx->BTCTRL.bit.VALID    = true;
		descrx->DSTADDR.reg         = (uint32_t)rx + count;
		descrx->BTCNT.reg           = count;

		desctx->BTCTRL.bit.VALID    = true;
		desctx->SRCADDR.reg         = (uint32_t)tx + count;
		desctx->BTCNT.reg           = count;

		DMAC->Channel[_dmaChannelTx].CHCTRLA.bit.ENABLE = 1;
		if(rx != NULL)
			DMAC->Channel[_dmaChannelRx].CHCTRLA.bit.ENABLE = 1;

		//wait for the transfer to finish
		while(_writeback[_dmaChannelTx].BTCTRL.bit.VALID || !_p_sercom->isDataRegisterEmptySPI());
		while(!_p_sercom->isTransmitCompleteSPI());
	}
	else{
	  //use a normal transfer
	  uint8_t *rxBuffer = reinterpret_cast<uint8_t *>(rx);
	  uint8_t *txBuffer = reinterpret_cast<uint8_t *>(tx);
	  for (size_t i=0; i<count; i++) {
		*rxBuffer++ = transfer(*txBuffer++);
	  }
	}
}

void SPIClass::transfer(void *buf, size_t count){
	transfer(buf, buf, count);
}

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

#if SPI_INTERFACES_COUNT > 0
  /* In case new variant doesn't define these macros,
   * we put here the ones for Arduino Zero.
   *
   * These values should be different on some variants!
   *
   * The SPI PAD values can be found in cores/arduino/SERCOM.h:
   *   - SercomSpiTXPad
   *   - SercomRXPad
   */
#ifndef PERIPH_SPI
    #define PERIPH_SPI           sercom4
    #define PAD_SPI_TX           SPI_PAD_2_SCK_3
    #define PAD_SPI_RX           SERCOM_RX_PAD_0
#endif // PERIPH_SPI

#ifdef SPI_HAS_DMA
  SPIClass SPI (&PERIPH_SPI,  PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI,  PAD_SPI_TX,  PAD_SPI_RX, SPI_DMA_CHANNEL_RX, SPI_DMA_CHANNEL_TX);
#else
  SPIClass SPI (&PERIPH_SPI,  PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI,  PAD_SPI_TX,  PAD_SPI_RX);
#endif
#endif
#if SPI_INTERFACES_COUNT > 1
#ifdef SPI1_HAS_DMA
  SPIClass SPI1(&PERIPH_SPI1, PIN_SPI1_MISO,  PIN_SPI1_SCK,  PIN_SPI1_MOSI,  PAD_SPI1_TX,  PAD_SPI1_RX, SPI1_DMA_CHANNEL_RX, SPI1_DMA_CHANNEL_TX);
#else
  SPIClass SPI1(&PERIPH_SPI1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI, PAD_SPI1_TX, PAD_SPI1_RX);
#endif
#endif
#if SPI_INTERFACES_COUNT > 2
#ifdef SPI2_HAS_DMA
  SPIClass SPI2(&PERIPH_SPI2, PIN_SPI2_MISO,  PIN_SPI2_SCK,  PIN_SPI2_MOSI,  PAD_SPI2_TX,  PAD_SPI2_RX, SPI2_DMA_CHANNEL_RX, SPI2_DMA_CHANNEL_TX);
#else
  SPIClass SPI2(&PERIPH_SPI2, PIN_SPI1_MISO, PIN_SPI2_SCK, PIN_SPI2_MOSI, PAD_SPI2_TX, PAD_SPI2_RX);
#endif
#endif
#if SPI_INTERFACES_COUNT > 3
#ifdef SPI3_HAS_DMA
  SPIClass SPI3(&PERIPH_SPI3, PIN_SPI3_MISO,  PIN_SPI3_SCK,  PIN_SPI3_MOSI,  PAD_SPI3_TX,  PAD_SPI3_RX, SPI3_DMA_CHANNEL_RX, SPI3_DMA_CHANNEL_TX);
#else
  SPIClass SPI3(&PERIPH_SPI3, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_MOSI, PAD_SPI3_TX, PAD_SPI3_RX);
#endif
#endif
#if SPI_INTERFACES_COUNT > 4
#ifdef SPI4_HAS_DMA
  SPIClass SPI4(&PERIPH_SPI4,  PIN_SPI4_MISO,  PIN_SPI4_SCK,  PIN_SPI4_MOSI,  PAD_SPI4_TX,  PAD_SPI4_RX, SPI4_DMA_CHANNEL_RX, SPI4_DMA_CHANNEL_TX);
#else
  SPIClass SPI4(&PERIPH_SPI4, PIN_SPI4_MISO, PIN_SPI4_SCK, PIN_SPI4_MOSI, PAD_SPI4_TX, PAD_SPI4_RX);
#endif
#endif
#if SPI_INTERFACES_COUNT > 5
#ifdef SPI5_HAS_DMA
  SPIClass SPI5(&PERIPH_SPI5,  PIN_SPI5_MISO,  PIN_SPI5_SCK,  PIN_SPI5_MOSI,  PAD_SPI5_TX,  PAD_SPI5_RX, SPI5_DMA_CHANNEL_RX, SPI5_DMA_CHANNEL_TX);
#else
  SPIClass SPI5(&PERIPH_SPI5, PIN_SPI5_MISO, PIN_SPI5_SCK, PIN_SPI5_MOSI, PAD_SPI5_TX, PAD_SPI5_RX);
#endif
#endif

