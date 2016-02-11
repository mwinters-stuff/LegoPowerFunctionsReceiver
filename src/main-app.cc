/*
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>
//#include <avr/interrupt.h>
//#include <avr/io.h>
#include "openpf.h"

#define RECV_PIN 2
#define LED_PIN 7

#define PIN_C2A_1 A0
#define PIN_C2A_2 A1
#define PIN_C2B_1 A2
#define PIN_C2B_2 A3

//#define PWM0A	IO_D6
//#define	PWM0B	IO_D5
//#define PWM2A	IO_B3
//#define PWM2B	IO_D3

#define PWM0A	6
#define	PWM0B	5
#define PWM2A	11
#define PWM2B	3

IRrecv irrecv(RECV_PIN,LED_PIN);

decode_results results;
#define NUMBER_CHANNELS 4
OpenPfRx_channel channelData[NUMBER_CHANNELS];
uint16_t legochannel[8] = {0,0,0,0,0,0,0,0}; //can be used to store retrieved data


void setup()
{
  OpenPfRx_channel_init(&channelData[0],0);
  OpenPfRx_channel_init(&channelData[1],1);
  OpenPfRx_channel_init(&channelData[2],2);
  OpenPfRx_channel_init(&channelData[3],3);

  Serial.begin(57600);

  pinMode(PIN_C2A_1,OUTPUT);
  pinMode(PIN_C2A_2,OUTPUT);
  digitalWrite(PIN_C2A_1,LOW);
  digitalWrite(PIN_C2A_2,LOW);

  pinMode(PIN_C2B_1,OUTPUT);
  pinMode(PIN_C2B_2,OUTPUT);
  digitalWrite(PIN_C2B_1,LOW);
  digitalWrite(PIN_C2B_2,LOW);

//  pinMode(PWM0A,OUTPUT);
//  pinMode(PWM0B,OUTPUT);
//  pinMode(PWM2A,OUTPUT);
//  pinMode(PWM2B,OUTPUT);

  TCCR0A = TCCR2A = 0xF3;
  TCCR0B = TCCR2B = 0x02;
  OCR0A = OCR0B = OCR2A = OCR2B = 0;

//	analogWrite(PWM0A, 0);
//	analogWrite(PWM0B, 0);
//	analogWrite(PWM2A, 0);
//	analogWrite(PWM2B, 0);

  irrecv.enableIRIn(); // Start the receiver
  Serial.println("Ready");

}

void setOrangutanM1Speed(int speed)
{
//if(speed > 0){
//    digitalWrite(PWM0A,HIGH);
//    digitalWrite(PWM0B,LOW);
//
//  }else if(speed < 0){
//    digitalWrite(PWM0A,LOW);
//    digitalWrite(PWM0B,HIGH);
//
//  }else{
//    digitalWrite(PWM0A,LOW);
//    digitalWrite(PWM0B,LOW);
//  }
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
		OCR0B = 0;		// hold one driver input high
		OCR0A = speed;
    //analogWrite(PWM0B, 0);
    //analogWrite(PWM0A, speed);
	}
	else	// forward
	{
    OCR0B = speed;	// pwm one driver input
		OCR0A = 0;		// hold the other driver input high
//    analogWrite(PWM0A, 0);
//    analogWrite(PWM0B, speed);
	}
}

void setOrangutanM2Speed(int speed)
{

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
    analogWrite(PWM2B, 0);
		analogWrite(PWM2A, speed);
	}
	else	// forward
	{
		analogWrite(PWM2A, 0);
    analogWrite(PWM2B, speed);
	}

}

void setM3Speed(int speed){
  if(speed > 0){
    digitalWrite(PIN_C2A_1,HIGH);
    digitalWrite(PIN_C2A_2,LOW);

  }else if(speed < 0){
    digitalWrite(PIN_C2A_1,LOW);
    digitalWrite(PIN_C2A_2,HIGH);

  }else{
    digitalWrite(PIN_C2A_1,LOW);
    digitalWrite(PIN_C2A_2,LOW);
  }
}

void setM4Speed(int speed){
  if(speed > 0){
    digitalWrite(PIN_C2B_1,HIGH);
    digitalWrite(PIN_C2B_2,LOW);

  }else if(speed < 0){
    digitalWrite(PIN_C2B_1,LOW);
    digitalWrite(PIN_C2B_2,HIGH);

  }else{
    digitalWrite(PIN_C2B_1,LOW);
    digitalWrite(PIN_C2B_2,LOW);
  }
}

void printOutput(OpenPfRx_output &data)
{
  Serial.print("PWM Value:      ");
  Serial.println(data.pwmvalue);
  Serial.print("Brake FloatCnt: ");
  Serial.println(data.brakethenfloatcount);
  Serial.print("C1:             ");
  Serial.println(data.C1);
  Serial.print("C2:             ");
  Serial.println(data.C2);
  Serial.print("Output Mode:    ");
  switch(data.output_mode)
  {
  case OM_FWD:
    Serial.println("Forward         ");
    break;
  case OM_BWD:
    Serial.println("Backward        ");
    break;
  case OM_FLOAT:
    Serial.println("Float           ");
    break;
  case OM_BRAKE:
    Serial.println("Brake           ");
    break;
  case OM_BRAKE_THEN_FLOAT:
    Serial.println("Brake then Float");
    break;
  case OM_INDEPENDENT:
    Serial.println("Independant     ");
    break;
  }

}

void printData(OpenPfRx_channel &data)
{
  Serial.print("Channel:        ");
  Serial.println(data.channel_number);
  Serial.print("toggle:         ");
  Serial.println(data.timeout_limit);
  Serial.print("timeout:        ");
  Serial.println(data.timeout);
  Serial.print("timeout limit:  ");
  Serial.println(data.timeout_limit);
  Serial.print("timeout action: ");
  Serial.println(data.timeout_action);

  Serial.println("Output A");
  printOutput(data.A);
  Serial.println("Output B");
  printOutput(data.B);
  Serial.println();
  Serial.println();

}

void setChannelPWM(uint8_t channelNumber,int aPWMSpeed, int bPWMSpeed)
{
  Serial.print("setChannelPWM ");
  Serial.print(channelNumber);
  Serial.print(" A ");
  Serial.print(aPWMSpeed);
  Serial.print(" B ");
  Serial.println(bPWMSpeed);
  switch(channelNumber){
    case 0: // orangutan motors PWM
      setOrangutanM1Speed(aPWMSpeed);
      setOrangutanM2Speed(bPWMSpeed);
    break;

    case 1: // secondary motoros, no pwm?
      setM3Speed(aPWMSpeed);
      setM4Speed(bPWMSpeed);
    break;

  }
  if(aPWMSpeed == 0 && bPWMSpeed == 0){
    channelData[channelNumber].timeout_action = 0;
  }
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
//  Serial.print("PWM Values Channel ");
//  Serial.print(channelNumber);
//  Serial.print(" A ");
//  Serial.print(pwmSpeedA);
//  Serial.print(" B ");
//  Serial.println(pwmSpeedB);

  setChannelPWM(channelNumber,pwmSpeedA,pwmSpeedB);

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
      //printData(channelData[channelNumber]);

      channelDo(channelNumber);

    }


    irrecv.resume();
  }else{

    for(int channelNumber = 0; channelNumber < NUMBER_CHANNELS; channelNumber++){
      channelData[channelNumber].A.brakethenfloatcount++;
      channelData[channelNumber].B.brakethenfloatcount++;

      if(channelData[channelNumber].timeout){
        --channelData[channelNumber].timeout;
//        Serial.print("Timeout Down Channel ");
//        Serial.print(channelNumber);
//        Serial.print(" ");
//        Serial.println(channelData[channelNumber].timeout);
      }

      if(channelData[channelNumber].timeout == 0 && channelData[channelNumber].timeout_action){
        Serial.print("Timeout Channel ");
        Serial.println(channelNumber);
        setChannelPWM(channelNumber,0,0);
        channelData[channelNumber].timeout_action = 0;
      }
    }
  }


  delay(100);
}
