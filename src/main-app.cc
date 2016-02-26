/*
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <Arduino.h>
#include <IRremote.h>

#define SOFTPWM_OUTPUT_DELAY
#include <PalatisSoftPWM.h>


#include "openpf.h"
#include "serial.h"

#define RECV_PIN 2
#define LED_PIN 7

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_D 3

#define MOTOR_A_F 0
#define MOTOR_A_R 1

#define MOTOR_B_F 2
#define MOTOR_B_R 3

#define MOTOR_C_F 4
#define MOTOR_C_R 5

#define MOTOR_D_F 6
#define MOTOR_D_R 7

SOFTPWM_DEFINE_CHANNEL(MOTOR_A_F, DDRD, PORTD, PORTD5);
SOFTPWM_DEFINE_CHANNEL(MOTOR_A_R, DDRD, PORTD, PORTD6);

SOFTPWM_DEFINE_CHANNEL(MOTOR_B_F, DDRD, PORTD, PORTD3);
SOFTPWM_DEFINE_CHANNEL(MOTOR_B_R, DDRB, PORTB, PORTB3);

SOFTPWM_DEFINE_CHANNEL(MOTOR_C_F, DDRC, PORTC, PORTC0);
SOFTPWM_DEFINE_CHANNEL(MOTOR_C_R, DDRC, PORTC, PORTC1);

SOFTPWM_DEFINE_CHANNEL(MOTOR_D_F, DDRC, PORTC, PORTC2);
SOFTPWM_DEFINE_CHANNEL(MOTOR_D_R, DDRC, PORTC, PORTC3);

SOFTPWM_DEFINE_OBJECT(8);
IRrecv irrecv(RECV_PIN,LED_PIN);

decode_results results;
#define NUMBER_CHANNELS 2
OpenPfRx_channel channelData[NUMBER_CHANNELS];
uint16_t legochannel[8] = {0,0,0,0,0,0,0,0}; //can be used to store retrieved data

struct motorChannelStruct{
  uint8_t forwardChannelA;
  uint8_t reverseChannelA;
  uint8_t forwardChannelB;
  uint8_t reverseChannelB;
  int16_t speedA;
  int16_t speedB;
};

motorChannelStruct motorChannels[NUMBER_CHANNELS] = {
  {MOTOR_A_F, MOTOR_A_R,MOTOR_B_F, MOTOR_B_R,0,0},
  {MOTOR_C_F, MOTOR_C_R,MOTOR_D_F, MOTOR_D_R,0,0}
};


utils::MSerial *serial;

PROGMEM const char c_HELLO[]              = "Lego Power Functions Receiver\n";

void setup()
{
  serial = utils::MSerial::Instance();
  serial->printf_P(c_HELLO);

  OpenPfRx_channel_init(&channelData[0],0);
  OpenPfRx_channel_init(&channelData[1],1);
  serial->StartRead();
  irrecv.enableIRIn(); // Start the receiver
  PalatisSoftPWM.begin(500);
  serial->printf_P(PSTR("Ready\n"));
}


void setMotorASpeed(int channel, int speed)
{
 // serial->printf_P(PSTR("A\n"));

  if(motorChannels[channel].speedA != speed){
    motorChannels[channel].speedA = speed;
    unsigned char reverse = 0;

    if (speed < 0)
    {
      speed = -speed;	// make speed a positive quantity
      reverse = 1;	// preserve the direction
    }
    if (speed > 0xFF)	// 0xFF = 255
      speed = 0xFF;

    if (reverse)
    {
      serial->printf_P(PSTR("Set Motor A Channel %d Reverse %d\n"),channel, speed);
      PalatisSoftPWM.set(motorChannels[channel].forwardChannelA , 0);
      PalatisSoftPWM.set(motorChannels[channel].reverseChannelA, speed);

    }
    else	// forward
    {
      serial->printf_P(PSTR("Set Motor A Channel %d Forward %d\n"),channel, speed);
      PalatisSoftPWM.set(motorChannels[channel].reverseChannelA, 0);
      PalatisSoftPWM.set(motorChannels[channel].forwardChannelA , speed);
    }
	}
}

void setMotorBSpeed(int channel, int speed)
{
  if(motorChannels[channel].speedB != speed){
    motorChannels[channel].speedB = speed;
    unsigned char reverse = 0;

    if (speed < 0)
    {
      speed = -speed;	// make speed a positive quantity
      reverse = 1;	// preserve the direction
    }
    if (speed > 0xFF)	// 0xFF = 255
      speed = 0xFF;

    if (reverse)
    {
      serial->printf_P(PSTR("Set Motor B Channel %d Reverse %d\n"),channel, speed);
      PalatisSoftPWM.set(motorChannels[channel].forwardChannelB , 0);
      PalatisSoftPWM.set(motorChannels[channel].reverseChannelB, speed);
    }
    else	// forward
    {
      serial->printf_P(PSTR("Set Motor B Channel %d Forward %d\n"),channel, speed);
      PalatisSoftPWM.set(motorChannels[channel].reverseChannelB, 0);
      PalatisSoftPWM.set(motorChannels[channel].forwardChannelB , speed);
    }
	}
}


void printOutput(OpenPfRx_output &data)
{
  serial->printf_P(PSTR("PWM Value:      %d\n"),data.pwmvalue);
  serial->printf_P(PSTR("Brake FloatCnt: %d\n"),data.brakethenfloatcount);
  serial->printf_P(PSTR("C1:             %d\n"),data.C1);
  serial->printf_P(PSTR("C2:             %d\n"),data.C2);
  serial->printf_P(PSTR("Output Mode:    "));
  switch(data.output_mode)
  {
  case OM_FWD:
    serial->printf_P(PSTR("Forward\n"));
    break;
  case OM_BWD:
    serial->printf_P(PSTR("Backward\n"));
    break;
  case OM_FLOAT:
    serial->printf_P(PSTR("Float\n"));
    break;
  case OM_BRAKE:
    serial->printf_P(PSTR("Brake\n"));
    break;
  case OM_BRAKE_THEN_FLOAT:
    serial->printf_P(PSTR("Brake then Float\n"));
    break;
  case OM_INDEPENDENT:
    serial->printf_P(PSTR("Independant\n"));
    break;
  }

}

void printData(OpenPfRx_channel &data)
{
  serial->printf_P(PSTR("Channel:        %d\n"),data.channel_number);
  serial->printf_P(PSTR("toggle:         %d\n"),data.timeout_limit);
  serial->printf_P(PSTR("timeout:        %d\n"),data.timeout);
  serial->printf_P(PSTR("timeout limit:  %d\n"),data.timeout_limit);
  serial->printf_P(PSTR("timeout action: %d\n"),data.timeout_action);

  serial->printf_P(PSTR("Output A\n"));
  printOutput(data.A);
  serial->printf_P(PSTR("Output B\n"));
  printOutput(data.B);
  serial->printf_P(PSTR("\n\n"));
}

void channelDo(int channelNumber)
{

  int pwmSpeedA = 0; // stop
  if(channelData[channelNumber].A.output_mode == OM_BWD)
  {
    channelData[channelNumber].timeout = channelData[channelNumber].timeout_limit;
    pwmSpeedA = -channelData[channelNumber].A.pwmvalue;
  }
  else if(channelData[channelNumber].A.output_mode == OM_FWD)
  {
    channelData[channelNumber].timeout = channelData[channelNumber].timeout_limit;
    pwmSpeedA = channelData[channelNumber].A.pwmvalue;
  }

  int pwmSpeedB = 0; // stop
  if(channelData[channelNumber].B.output_mode == OM_BWD)
  {
    channelData[channelNumber].timeout = channelData[channelNumber].timeout_limit;
    pwmSpeedB = -channelData[channelNumber].B.pwmvalue;
  }
  else if(channelData[channelNumber].B.output_mode == OM_FWD)
  {
    channelData[channelNumber].timeout = channelData[channelNumber].timeout_limit;
    pwmSpeedB = channelData[channelNumber].B.pwmvalue;
  }

  setMotorASpeed(channelNumber,pwmSpeedA);
  setMotorBSpeed(channelNumber,pwmSpeedB);

}

void loop()
{

  if (irrecv.decode(&results))
  {
    uint16_t pfdata = results.value;
    DBG_PRINTLN(pfdata, BIN);
    if(results.decode_type == LEGO_POWERFUNCTIONS &&  OpenPfRxVerifyChecksum(pfdata))                         //if Checksum is OK continue processing data
    {
      uint8_t channelNumber = OpenPfRxGetChannelNumber(pfdata);
      legochannel[channelNumber] = pfdata;
      OpenPfRxInterpreter(&legochannel[channelNumber] , &channelData[channelNumber]);
     // printData(channelData[channelNumber]);

      channelDo(channelNumber);
    }
    irrecv.resume();

  }else{

    for(int channelNumber = 0; channelNumber < NUMBER_CHANNELS; channelNumber++){
      channelData[channelNumber].A.brakethenfloatcount++;
      channelData[channelNumber].B.brakethenfloatcount++;

      if(channelData[channelNumber].timeout){
        --channelData[channelNumber].timeout;
      }

      if(channelData[channelNumber].timeout == 0 && channelData[channelNumber].timeout_action){
        setMotorASpeed(channelNumber,0);
        setMotorBSpeed(channelNumber,0);
        channelData[channelNumber].timeout_action = 0;
      }
    }
  }

  serial->Read();
  delay(10);
}

