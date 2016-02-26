/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * AEGController
 * Copyright (C) Mathew Winters 2010 <mathew@winters.org.nz>
 *
 * AEGController is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * AEGController is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//#include <avr/io.h>
//#include <util/delay.h>
//#include <string.h>
//#include <stdio.h>
//#include <avr/pgmspace.h>
//#include <util/atomic.h>

#include <Arduino.h>

#include "serial.h"

using namespace utils;

MSerial *MSerial::m_Instance;
char MSerial::send_buffer[SERIAL_BUFFER];
char MSerial::send_buffer2[SERIAL_BUFFER];
char MSerial::data[SERIAL_BUFFER];
    char MSerial::receive_buffer[SERIAL_BUFFER];
    unsigned char MSerial::receive_buffer_position;



MSerial *MSerial::Instance()
{
  if(m_Instance == NULL)
  {
    m_Instance = (MSerial*) malloc(sizeof(Serial));
    m_Instance->Init();
  }
  return m_Instance;
}


void MSerial::WaitForSendingToFinish()
{
 // while(!MWAVR::SerialComms::sendBufferEmpty());
}


void MSerial::Send(const char *data)
{
  Instance()->internalSend((uint8_t*)data,strlen(data));

}

void MSerial::Send(const uint8_t *data, uint8_t datalen)
{
  Instance()->internalSend((uint8_t*)data,datalen);
}

void MSerial::internalSend(const uint8_t *data, uint8_t datalen)
{
  memcpy(send_buffer2,(char*)data,datalen);
 Serial.write(send_buffer2,datalen);
 Serial.flush();
}


void MSerial::Init()
{
  m_Reading = false;
 Serial.begin(57600);
}

void MSerial::StartRead()
{
// // MWAVR::SerialComms::receiveRing(receive_buffer, sizeof(receive_buffer));
  m_Reading = true;
//  m_MsgBufPos = 0;
//  memset(m_MsgBuf,0,sizeof(m_MsgBuf));
}

void MSerial::StopRead()
{
  m_Reading = false;
 // MWAVR::SerialComms::cancelReceive();
}

void MSerial::SendOk()
{
//  SendData(S_OK,NULL,0);
}

void MSerial::ProcessByte(uint8_t byte)
{
  m_MsgBuf[m_MsgBufPos++] = byte;

  if(m_MsgBufPos == sizeof(m_MsgBuf))
  {
    m_MsgBufPos = 0;
    return;
  }

  if(m_MsgBuf[0] == '0' && m_MsgBuf[1] == ' ')
  {
    // boot loader required..
    noInterrupts();
    PCICR = 0;      // disable pin-change interrupts
    PCMSK0 = 0;
    PCMSK1 = 0;
    PCMSK2 = 0;

    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A  = 0;  //  20MHz clock / 20000 = 1mS
    TIMSK1 = 0;

    goto *0x7800; // bootloader.
  }

//  if(m_MsgBuf[0] == m_MsgBufPos - 1)
//  {
//    ATOMIC_BLOCK(ATOMIC_FORCEON)
//    {
//      gIdleCount = 0;
//    }
//    uint8_t *datastart = &m_MsgBuf[2];
//    uint8_t datalen = m_MsgBufPos - 2;
//    switch(m_MsgBuf[1])
//    {
//      case S_PGM:
//        MSerial::printf_P(c_REC_PGM);
//        Programming::Instance()->SerialProgram(datastart,datalen,pwCurrent);
//        SendOk();
//        break;
//      case S_PSM:
//        MSerial::printf_P(c_REC_PSM);
//        Programming::Instance()->SerialProgram(datastart,datalen,pwStored);
//        SendOk();
//      case S_GPM:
//        MSerial::printf_P(c_REC_GPM);
//        Programming::Instance()->SendProgram(pwCurrent);
//        break;
//      case S_GSM:
//        MSerial::printf_P(c_REC_GSM);
//        Programming::Instance()->SendProgram(pwStored);
//        break;
//      case S_DTA:
//        MSerial::printf_P(c_REC_DTA);
//        Storage::Instance()->SendOnSerial(datalen == 1 ? datastart[0] : 0xff);
//        break;
//      case S_BAT:
//        MSerial::printf_P(c_REC_BAT);
//        VoltageSensor::Instance()->SendOnSerial();
//        break;
//      case S_SOT:
//        MSerial::printf_P(c_REC_SOT);
//        Motor::Instance()->SingleShot();
//        SendOk();
//        break;
//      case S_CLR:
//        MSerial::printf_P(c_REC_CLR);
//        Storage::Instance()->Clear();
//        SendOk();
//        break;
//      case S_RST:
//        MSerial::printf_P(c_REC_RST);
//        Programming::Instance()->Reset();
//        SendOk();
//        break;
//      case S_VER:
//        MSerial::printf_P(c_REC_VER);
//        {
//          char str[20];
//          sprintf_P(str,c_VersionFormat,AutoVersion::FULLVERSION_STRING,AutoVersion::SVN_REVISION,AutoVersion::DATE,AutoVersion::MONTH,AutoVersion::YEAR);
//          SendData(S_VER,str,strlen(str));
//        }
//        break;
//      case S_TOT:
//        MSerial::printf_P(c_REC_TOT);
//        Storage::Instance()->SendTotal();
//        break;
//      case S_CLI:
//        Storage::Instance()->ClearDataBlock(datastart[0]);
//        SendOk();
//        break;
//    }
//    memset(m_MsgBuf,0,100);
//    m_MsgBufPos = 0;
//  }
}

bool MSerial::Read()
{
  bool rv = false;
	if(Serial.available()>0)
	{
    //SWLion::LED::Toggle(SWLion::LED::GREEN);
	  rv = true;
	  receive_buffer[receive_buffer_position] = Serial.read();
		// Process the new byte that has just been received.
		ProcessByte(receive_buffer[receive_buffer_position]);

		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer.
		if (receive_buffer_position == sizeof(receive_buffer)-1)
		{
			receive_buffer_position = 0;
		}
		else
		{
			receive_buffer_position++;
		}
	}
	return rv;
}


// wrap a printf for serial and PROGMEM output.
void MSerial::printf_P(const char *__fmt, ...)
{
  va_list ap;
  va_start (ap, __fmt);
  vsprintf_P (data, __fmt, ap);
  va_end (ap);
  Send(data);
}

void MSerial::printf(const char *__fmt, ...)
{
  va_list ap;
  va_start (ap, __fmt);
  vsprintf(data, __fmt, ap);
  va_end (ap);
  Send(data);
}

void MSerial::SendData(const uint8_t msg, const void *data, const size_t datalen)
{
  MSerial::Instance()->internalSendData(msg, data,datalen);
}

void MSerial::internalSendData(const uint8_t msg, const void *data, const size_t datalen)
{
  WaitForSendingToFinish();
  send_buffer[0] = (uint8_t)datalen + 1;
  send_buffer[1] = msg;
  if(datalen > 0)
  {
    memcpy(&send_buffer[2],data,datalen);
  }
 // SWLion::LED::RedOn();
 // MWAVR::SerialComms::send(send_buffer, datalen+2);
//  SWLion::LED::RedGreenOff();
  Serial.write(send_buffer,datalen+1);
}

