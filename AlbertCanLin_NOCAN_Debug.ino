#include <Arduino.h>

#include <SPI.h>
#include "mcp_can.h"
//#include <SoftwareSerial.h>

//SoftwareSerial Serial1(2, 3); // RX, TX

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

#define EPS_Serial Serial

#define PB_LeftPin 4
#define PB_RightPin 5

unsigned char canLen = 0;
unsigned char canBuf[8];

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned short lastLkasSend = 2000;

unsigned short lastCanReceived = 0;

unsigned short lastLEDtime = 0;

uint8_t LkasActive = 0;

uint8_t smallSteer = 0;
uint8_t bigSteer = 0;

uint8_t flipFlopBit = 0;

uint8_t lkas_off_array[][4] =  { {0x00, 0x80, 0xc0, 0xc0}, {0x20, 0x80, 0xc0, 0xa0} };

uint8_t EPStoLKASBuffer[5];
uint8_t LKAStoEPSBuffer[4];

uint8_t EPStoLKASBufferCounter = 0;

uint8_t OPCanCounter = 0;

uint8_t sendCounter = 0;
uint8_t ledCounter = 0;

uint8_t PB_RightActive = 0;
uint8_t PB_LeftActive = 0;
uint8_t PB_ButtonActive = 0;

int8_t LkasCountdown = 5;

uint8_t honda_compute_checksum(uint8_t *steerTorqueAndMotorTorque, uint8_t len, unsigned int addr);
uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte);

void sendSteerStatus();

void buildSendAllLinDataCanMsg();

void sentToEPS_Serial(uint8_t *a) {
  EPS_Serial.write(*a);
  EPS_Serial.write(*(a + 1));
  EPS_Serial.write(*(a + 2));
  EPS_Serial.write(*(a + 3));
}

void sendLKAS() {
  uint8_t msg[4];
  if (flipFlopBit) {
    msg[0] = flipFlopBit;
    flipFlopBit = 0;
  } else {
    msg[0] = flipFlopBit;
    flipFlopBit = 32;
  }

  if (PB_ButtonActive) {
    if (PB_LeftActive) {
      bigSteer = 10;
      smallSteer = 20;
    } else if (PB_RightActive) {
      bigSteer = 5;
      smallSteer = 10;
    }
    if (flipFlopBit) smallSteer += 1;
    if (LkasCountdown > 0) {
      LkasCountdown--;
      bigSteer = 0;
      smallSteer = 0;
    }
    LkasActive = 1;
  } else {
    LkasActive = 0;
    LkasCountdown = 5;
  }

  if (LkasActive) {
    msg[0] |= bigSteer | flipFlopBit;
    msg[1] = smallSteer + 128;
    msg[2] = 0xC0;
    msg[3] = chksm(msg[0], msg[1], msg[2]);
    sentToEPS_Serial(&msg[0]);
    //Serial1.println("Lon");//same us any button pressed
  } else {
    if (flipFlopBit) sentToEPS_Serial(&lkas_off_array[1][0]);
    else sentToEPS_Serial(&lkas_off_array[0][0]);
  }

  //   if ( (millis() - lastCanReceived) > 100) LkasActive = 0;
}


uint8_t chksm(uint8_t firstByte, uint8_t secondByte, uint8_t thirdByte) {
  uint8_t tot = firstByte + secondByte + thirdByte ;
  tot = 256 - tot;
  tot %= 128;
  tot += 128;
  return tot;
}

void readCan() {
  CAN.readMsgBuf(&canLen, canBuf);    // read data,  len: data length, buf: data buf

  unsigned int canId = CAN.getCanId();
  if (canId != 228) return;

  //  uint8_t lclBigSteer = 0;
  //  uint8_t lclLittleSteer = 0;

  LkasActive = canBuf[2] >> 7;

  bigSteer = ( canBuf[0] >> 4 ) & 8;
  bigSteer |= ( canBuf[1] >> 5 ) & 7;

  smallSteer = canBuf[1] & 31 ;
}


void readSerial() {

  if (!EPS_Serial.available()) return;

  uint8_t data = EPS_Serial.read();
  if (data < 32) EPStoLKASBufferCounter = 0;
  EPStoLKASBuffer[EPStoLKASBufferCounter] = data;

  if (data > 32) {  //debug
    Serial1.print("Data = "); Serial1.println(data);
  }

  if (EPStoLKASBufferCounter > B00000011) {
    sendSteerStatus();
    buildSendAllLinDataCanMsg();
    EPStoLKASBufferCounter = 0;
    OPCanCounter++;
    OPCanCounter &= 3;
    Serial1.print("E");
    Serial1.print((EPStoLKASBuffer[2] & 2) >> 1, DEC);
    Serial1.println(EPStoLKASBuffer[2] & 1, DEC);
  }

}


void sendSteerStatus() {


  // outputSerial.print("\nsending Steer Status Cna MSg");
  // CAN_message_t msg; // move this to a global so you dont have to re assign the id and len
  // CAN_msg_t msg;

  canBuf[0]  = EPStoLKASBuffer[0] << 5;   // 3 LSB of BigSteerTorque (4bit)
  canBuf[0] |= EPStoLKASBuffer[1] & B00011111; // all of smallSteerTorque
  canBuf[0]  = ~canBuf[0]; // invert the whole message to make negative positive, positive negative.  OP wants left positive (why??)
  canBuf[1]  =  ( ~(  EPStoLKASBuffer[0] >> 3 ) )   & B00000001; // 1st MSB of bigSteerTorque (4bit) ... added NOT (~) to invert the sign


  //add other data from Teensy so OP can record it

  canBuf[1] |= (EPStoLKASBuffer[2] >> 4) & 2;              // CAN B1 O1


  canBuf[2]  =  EPStoLKASBuffer[2]       & B00000111; //EPS B2 O0-2 into CAN B2 O0-2
  canBuf[2] |= (EPStoLKASBuffer[0] >> 1) & B00001000; //EPS B0 O4 into CAN B2 O3

  canBuf[3]  = (OPCanCounter << 4 );           // put in the counter

  canBuf[3] |= honda_compute_checksum(&canBuf[0], 8, (unsigned int) 399);

  // sendCanMsg(&canBuf);
  //  CAN.sendMsgBuf(399, 0, 8, canBuf);

}

uint8_t honda_compute_checksum(uint8_t *steerTorqueAndMotorTorque, uint8_t len, unsigned int addr) {

  uint8_t checksum = 0U;
  while (addr > 0U) {
    checksum += (addr & 0xFU); addr >>= 4;
  }
  for (int j = 0; j < len; j++) {
    uint8_t byte = *(steerTorqueAndMotorTorque + j);
    checksum += (byte & 0xFU) + (byte >> 4U);
    if (j == (len - 1)) {
      checksum -= (byte & 0xFU);  // remove checksum in message
    }
  }
  return (8U - checksum) & 0xFU;
}



void buildSendAllLinDataCanMsg() {

  canBuf[0] =  LKAStoEPSBuffer[0];
  canBuf[1] =  LKAStoEPSBuffer[1];
  canBuf[2] =  LKAStoEPSBuffer[2];
  canBuf[3] =  EPStoLKASBuffer[0];
  canBuf[4] =  EPStoLKASBuffer[1];
  canBuf[5] =  EPStoLKASBuffer[2];
  canBuf[6] =  EPStoLKASBuffer[3];

  canBuf[7] = (OPCanCounter << 4 ); // put in the counter
  canBuf[7] |= honda_compute_checksum(&canBuf[0], 8, (unsigned int) 521);

  //     CAN.sendMsgBuf(521, 0, 8, canBuf);
}




void handleTimedFunc() {
  //   Serial1.println("tst");
  if (!digitalRead(PB_LeftPin)) {
    PB_ButtonActive = 1;
    PB_RightActive = 0;
    PB_LeftActive = 1;
    Serial1.println("Left Button");
  } else if (!digitalRead(PB_RightPin)) {
    PB_ButtonActive = 1;
    PB_RightActive = 1;
    PB_LeftActive = 0;
    Serial1.println("Right Button");
  } else {
    PB_ButtonActive = 0;
    PB_RightActive = 0;
    PB_LeftActive = 0;
    //Serial1.println("No Button");
  }





  //   if(EPStoLKASBuffer[2] & 1) digitalWrite(13,HIGH);
  //   else{
  //
  //      if(LkasActive){
  //            digitalWrite(13,digitalRead(13));
  //      } else{
  //         if(ledCounter > 2) {
  //            digitalWrite(13,digitalRead(13));
  //            ledCounter = 0;
  //         }
  //      }
  //      ledCounter++;
  //   }

  sendCounter = 0;
}



/*********************************************************************************************************
                                    SETUP    and    LOOP
*********************************************************************************************************/



void setup()
{

  //    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  //    {
  //        delay(100);
  //    }
  //not sure if needed
  //    CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
  //    CAN.init_Mask(1, 0, 0x3ff);

  //    CAN.init_Filt(0, 0, 0xe4);                          // there are 6 filter in mcp2515

  //   pinMode(13,OUTPUT); //pin 13 is used in SPI
  pinMode(PB_LeftPin, INPUT_PULLUP);
  pinMode(PB_RightPin, INPUT_PULLUP);
  EPS_Serial.begin(9600, SERIAL_8E1);
  Serial1.begin(115200);

  Serial1.println("Setup Complete!");
  delay(500);
}



void loop()
{
  if ( (millis() - lastLkasSend) > 10) {
    sendLKAS();
    lastLkasSend = millis();
    sendCounter++;
  }

  //   if(CAN_MSGAVAIL == CAN.checkReceive()) readCan();            // check if data coming
  readSerial();
  if (sendCounter > 25) {
    //Serial1.println("handleTimedFunc");
    handleTimedFunc();
  }



}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
