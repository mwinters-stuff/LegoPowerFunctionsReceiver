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
#ifndef _SERIAL_H
#define _SERIAL_H
/* Serial output for debug purposes */
#define SERIAL_BUFFER 100
namespace utils{
class MSerial
{
  private:
    bool m_Reading;

    uint8_t m_MsgBufPos;
    uint8_t m_MsgBuf[100];

    void SendOk();
  protected:
    void Init();

    void ProcessByte(uint8_t byte);
    static MSerial *m_Instance;

    static char data[SERIAL_BUFFER];
    static char receive_buffer[SERIAL_BUFFER];
    static unsigned char receive_buffer_position;
    static char send_buffer[SERIAL_BUFFER];
    static char send_buffer2[SERIAL_BUFFER];

  public:
    static MSerial *Instance();

    void StartRead();
    void StopRead();
    bool Read();
    void WaitForSendingToFinish();

    void internalSend(const uint8_t *data, uint8_t datalen);

    static void Send(const char *data);
    static void printf(const char *__fmt, ...);
    static void printf_P(const char *__fmt, ...);
    static void Send(const uint8_t *data, uint8_t datalen);
    static void SendData(const uint8_t msg, const void *data, const size_t datalen);
    void internalSendData(const uint8_t msg, const void *data, const size_t datalen);

};
};
#endif
