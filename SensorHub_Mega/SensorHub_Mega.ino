
#include <MemoryFree.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
#include <RCSwitch.h>

#include <IRLib.h>
//#include <IRremote.h>
//decode_results results;

//  while(Serial.read() >= 0) ; // flush the receive buffer


// MEGA 2560 PINS 
const int LED1_PIN = 6;
const int PWR1_PIN = 13;

const int IRRX_PIN = 5;
const int RCRX_PIN = 2; 
const int RHRX_PIN = 2; 

const int RHTX_PIN = 12; 
const int RCTX_PIN = 12; 


// UNO PINS 
//const byte LED1_PIN = 11;
//const byte PWR1_PIN = 7;
//
//const byte IRRX_PIN = 8;
//const byte RCRX_PIN = 2; 
//const byte RHRX_PIN = 2; 
//
//const byte RHTX_PIN = 5; 
//const byte RCTX_PIN = 5; 

RCSwitch RCsend = RCSwitch();
RCSwitch RCrecv = RCSwitch();
RH_ASK driver(2000,RHRX_PIN,RHTX_PIN);  //(speed, RXpin, TXpin, ptt;IN, inverted)


// Storage of Etek Codes
unsigned long Etek_code;
unsigned int Etek_length;
const unsigned long bellDec = 11097496;  
const unsigned long rc5onDec = 4551939;  // etek 5 on
char* bb;
unsigned long currentMillis = millis();
const int RCblockperiod = 3000; // blocking and debouce RCremote clicks
unsigned long RClastEvent;
unsigned long RCcurrEvent;
unsigned long RCinterval = 1000UL;
int RCreadyflag = 1;



int test=0;
#define FLAG_INTERRUPT 0x01
char buff[20]; 

long interval = 9000; 
long previousMillis = 0; 


// *****************************************
// ETEK CODES
// *****************************************

const int NUMBER_OF_etekELEMENTS = 20;
typedef struct {
   char etekCodeArr[16];
   byte etekStateArr;
   byte etekIDArr;
} etekCODES;

const etekCODES etekArr [NUMBER_OF_etekELEMENTS] PROGMEM = {
  {"F0FF0FFF0101",1,0}, {"F0FF0FFF1001",1,1}, {"F0FF0FF10001",1,2}, {"F0FFFF1F0001",1,3}, {"F0FFF1FF0001",1,4},
  {"F0FFFFFF0101",1,0}, {"F0FFFFFF1001",1,1}, {"F0FFFFF10001",1,2}, {"F0FF0F1F0001",1,3}, {"F0FF01FF0001",1,4},
  {"F0FF0FFF0110",0,0}, {"F0FF0FFF1010",0,1}, {"F0FF0FF10010",0,2}, {"F0FFFF1F0010",0,3}, {"F0FFF1FF0010",0,4},
  {"F0FFFFFF0110",0,0}, {"F0FFFFFF1010",0,1}, {"F0FFFFF10010",0,2}, {"F0FFFF1F0010",0,3}, {"F0FF01FF0010",0,4},
};  


// *****************************************
// IR REMOTE CODES
// *****************************************
const int MAX_CHAR_ID_remote = 4;
const int MAX_CHAR_TXT_remote = 12;
const int NUMBER_OF_remoteELEMENTS = 45;
typedef struct {
   long int remoteValArr; 
   char remoteIDArr[MAX_CHAR_ID_remote];
   char remoteTxtArr[4];
} remoteCODES;

const remoteCODES remoteArr [NUMBER_OF_remoteELEMENTS] PROGMEM = {
  { 0x807F807F,"WH","1" }, { 0x807F40BF,"WH","2" }, { 0x807FC03F,"WH","3" }, { 0x807F20DF,"WH","4" }, { 0x807FA05F,"WH","5" }, 
  { 0x807F609F,"WH","6" }, { 0x807FE01F,"WH","7" }, { 0x807F10EF,"WH","8" }, { 0x807F906F,"WH","9" }, { 0x807F00FF,"WH","0" }, 
  { 0x1CE3807F,"SY","1" }, { 0x1CE340BF,"SY","2" }, { 0x1CE3C03F,"SY","3" }, { 0x1CE320DF,"SY","4" }, { 0x1CE3A05F,"SY","5" }, 
  { 0x1CE3609F,"SY","6" }, { 0x1CE3E01F,"SY","7" }, { 0x1CE310EF,"SY","8" }, { 0x1CE3906F,"SY","9" }, { 0x1CE300FF,"SY","0" }, 
  { 0x36113D,"CC","1" }, { 0x37111D,"CC","2" }, { 0x36912D,"CC","3" }, { 0x37910D,"CC","4" }, { 0x365135,"CC","5" }, 
  { 0x375115,"CC","6" }, { 0x36D125,"CC","7" }, { 0x37D105,"CC","8" }, { 0x363139,"CC","9" }, { 0x373119,"CC","0" }, 
};


// *****************************************
// LED RGB LIGHT BULB
// *****************************************
const int NUMBER_OF_bulbELEMENTS = 24;
typedef struct {
   long int bulbValArr;
   char bulbTxtArr[12];
} bulbCODES;

const bulbCODES bulbArr [NUMBER_OF_bulbELEMENTS] PROGMEM = {
  { 0xF720DF, "bulbRED1" }, { 0xF710EF, "bulbRED2" }, { 0xF730CF, "bulbRED3" }, { 0xF708F7, "bulbRED4" }, { 0xF728D7, "bulbRED5" },
  { 0xF7A05F, "bulbGRN1" }, { 0xF7906F, "bulbGRN2" }, { 0xF7B04F, "bulbGRN3" }, { 0xF78877, "bulbGRN4" }, { 0xF7A857, "bulbGRN5" }, 
  { 0xF7609F, "bulbBLU1" }, { 0xF750AF, "bulbBLU2" }, { 0xF7708F, "bulbBLU3" }, { 0xF748B7, "bulbBLU4" }, { 0xF76897, "bulbBLU5" }, 
  { 0xF7E01F, "bulbWHITE"  }, { 0xF7D02F, "bulbFLASH" }, { 0xF7F00F, "bulbSTROBE" }, { 0xF7C837, "bulbFADE" }, { 0xF7E817, "bulbSMOOTH" },
  { 0xF700FF, "bulbBRITER" }, { 0xF7807F, "bulbDIMMER" }, { 0xF740BF, "bulbOFF" }, { 0xF7C03F, "bulbON" },
};



 
// *****************************************
// IR REMOTE CODES FUNCTIONS
// *****************************************

const int NUMBER_OF_remote2ELEMENTS = 80;
typedef struct {
   long int remoteValArr; 
   char remoteIDArr[MAX_CHAR_ID_remote];
   char remoteTxtArr[MAX_CHAR_TXT_remote];
} remote2CODES;


const remote2CODES remote2Arr [NUMBER_OF_remote2ELEMENTS] PROGMEM = {
    { 0x36E123,   "CC", "BACK" }, 
    { 0x36F121,   "CC", "CHNDWN" }, 
    { 0xBDF57432, "CC", "CHNUP" }, 
    { 0x807F22DD, "CC", "DOWN" }, 
    { 0x807F629D, "CC", "EXIT" }, 
    { 0x20DFD827, "CC", "INFO" }, 
    { 0x807F827D, "CC", "LEFT" }, 
    { 0x807F08F7, "CC", "MUTE" }, 
    { 0x20DF22DD, "CC", "OK" }, 
    { 0x807FC23D, "CC", "RIGHT" }, 
    { 0x807F02FD, "CC", "UP" }, 
    { 0x20DFC03F, "CC", "VOLDWN" }, 
    { 0x20DF40BF, "CC", "VOLUP" }, 
    { 0x57438679, "RK", "*" }, 
    { 0x574308F7, "RK", "AMAZON" }, 
    { 0x57436699, "RK", "BACK" }, 
    { 0x5743CC33, "RK", "DOWN" }, 
    { 0x5743AA55, "RK", "FWD" }, 
    { 0x5743C03F, "RK", "HOME" }, 
    { 0x57431EE1, "RK", "LAST" }, 
    { 0x57437887, "RK", "LEFT" }, 
    { 0x5743F00F, "RK", "MGO" }, 
    { 0x5743D22D, "RK", "NETFLIX" }, 
    { 0x574354AB, "RK", "OK" }, 
    { 0x574332CD, "RK", "PLAY" }, 
    { 0x5743B44B, "RK", "RIGHT" }, 
    { 0x57432CD3, "RK", "RWD" }, 
    { 0x57439867, "RK", "UP" }, 
    { 0x574310EF, "RK", "VUDU" }, 
    { 0x1CE358A7, "SY", "AUDIO" }, 
    { 0x1CE38877, "SY", "CAUTION" }, 
    { 0x1CE3D02F, "SY", "CHNDWN" }, 
    { 0x1CE350AF, "SY", "CHNUP" }, 
    { 0x1CE3F20D, "SY", "DOWN" }, 
    { 0x1CE3CA35, "SY", "EXIT" }, 
    { 0x1CE3EA15, "SY", "FIX SHAPE" }, 
    { 0x1CE3C837, "SY", "INPUT" }, 
    { 0x1CE3F807, "SY", "LEFT" }, 
    { 0x1CE3E817, "SY", "MENU" }, 
    { 0x1CE318E7, "SY", "MUTE" }, 
    { 0x1CE32AD5, "SY", "OK" }, 
    { 0x1CE348B7, "SY", "POWER" }, 
    { 0x1CE39867, "SY", "RECALL" }, 
    { 0x1CE338C7, "SY", "RESET" }, 
    { 0x1CE37887, "SY", "RIGHT" }, 
    { 0x1CE3B04F, "SY", "SLEEP" }, 
    { 0x1CE330CF, "SY", "UBUT" }, 
    { 0x1CE3728D, "SY", "UP" }, 
    { 0x1CE3F00F, "SY", "VOLDWN" }, 
    { 0x1CE3708F, "SY", "VOLUP" }, 
    { 0x807F50AF, "WH", "-" }, 
    { 0x807F42BD, "WH", "ASPECT" }, 
    { 0x807FDA25, "WH", "BACK" }, 
    { 0x807FAA55, "WH", "BLU" }, 
    { 0x807F5AA5, "WH", "BLU2" }, 
    { 0x807FB04F, "WH", "CHNDWN" }, 
    { 0x807F30CF, "WH", "CHNUP" }, 
    { 0x807FC837, "WH", "DISPLAY" }, 
    { 0x807F58A7, "WH", "DOWN" }, 
    { 0x807F6897, "WH", "EXIT" }, 
    { 0x807F32CD, "WH", "GRN" }, 
    { 0x807F1AE5, "WH", "GRN2" }, 
    { 0x807FD827, "WH", "LEFT" }, 
    { 0x807FA857, "WH", "MENU" }, 
    { 0x807F8877, "WH", "MUTE" }, 
    { 0x807FB847, "WH", "OK" }, 
    { 0x20DF10EF, "WH", "POWER" }, 
    { 0x807F2AD5, "WH", "RED" }, 
    { 0x807FEA15, "WH", "RED2" }, 
    { 0x807F38C7, "WH", "RIGHT" }, 
    { 0x807FE817, "WH", "SLEEP" }, 
    { 0x807F48B7, "WH", "SOURCE" }, 
    { 0x807F9867, "WH", "UP" }, 
    { 0x807FF00F, "WH", "VOLDWN" }, 
    { 0x807F708F, "WH", "VOLUP" }, 
    { 0x807F6A95, "WH", "YLW" }, 
    { 0x807F9A65, "WH", "YLW2" }, 
};


// *****************************************
// WESTINGHOUSE REPEATING
// *****************************************

const int NUMBER_OF_whremoteELEMENTS = 30;
typedef struct {
   long int remoteValArr; 
   char remoteIDArr[MAX_CHAR_ID_remote];
   char remoteTxtArr[MAX_CHAR_TXT_remote];
} whremoteCODES;


const whremoteCODES whremoteArr [NUMBER_OF_whremoteELEMENTS] PROGMEM = {
    { 0x807F50AF, "WH", "-" }, 
    { 0x807F42BD, "WH", "ASPECT" }, 
    { 0x807FDA25, "WH", "BACK" }, 
    { 0x807FAA55, "WH", "BLU" }, 
    { 0x807F5AA5, "WH", "BLU2" }, 
    { 0x807FB04F, "WH", "CHNDWN" }, 
    { 0x807F30CF, "WH", "CHNUP" }, 
    { 0x807FC837, "WH", "DISPLAY" }, 
    { 0x807F58A7, "WH", "DOWN" }, 
    { 0x807F6897, "WH", "EXIT" }, 
    { 0x807F32CD, "WH", "GRN" }, 
    { 0x807F1AE5, "WH", "GRN2" }, 
    { 0x807FD827, "WH", "LEFT" }, 
    { 0x807FA857, "WH", "MENU" }, 
    { 0x807F8877, "WH", "MUTE" }, 
    { 0x807FB847, "WH", "OK" }, 
    { 0x20DF10EF, "WH", "POWER" }, 
    { 0x807F2AD5, "WH", "RED" }, 
    { 0x807FEA15, "WH", "RED2" }, 
    { 0x807F38C7, "WH", "RIGHT" }, 
    { 0x807FE817, "WH", "SLEEP" }, 
    { 0x807F48B7, "WH", "SOURCE" }, 
    { 0x807F9867, "WH", "UP" }, 
    { 0x807FF00F, "WH", "VOLDWN" }, 
    { 0x807F708F, "WH", "VOLUP" }, 
    { 0x807F6A95, "WH", "YLW" }, 
    { 0x807F9A65, "WH", "YLW2" }, 
};



IRsend irsend; // UNO PWM PIN 3
IRrecv irrecv(IRRX_PIN);
IRsendNEC irsendNEC;   // irlib
IRdecode My_Decoder;   // irlib
unsigned int Buffer[RAWBUF];
//IRTYPES irType;
IRTYPES codeType;

//long int irType;
unsigned long irValue; 
int irLen;
int irREDV_flag=0;

// Storage of RADIOHEAD codes
char RHMsg[20];
int pulsetime = 140;  // RCremote pulse out

// #######################################################
// SETUP
// #######################################################
void setup() {
  
  Serial.begin(9600);	// Debugging only
//  if (!driver.init())  {Serial.println(F("init failed"));}  // screwed up ir library
  pinMode(PWR1_PIN, OUTPUT); digitalWrite(PWR1_PIN, HIGH);
  RCsend.enableTransmit(RCTX_PIN);
  RCsend.setProtocol(1);
  RCsend.setPulseLength(pulsetime);
  RCsend.setRepeatTransmit(6); 
  RCrecv.enableReceive(0);   // ARDUINO PIN 2 - interrupt 0 
  
  //irrecv.enableIRIn(); // Start  IR receiver
  irrecv.enableIRIn(); // Start the receiver
  My_Decoder.UseExtnBuf(Buffer);
  
  Serial.println ();
  Serial.print (F("Free memory = "));
  Serial.println (freeMemory());
  Serial.println(F("Setupt complete")); 
  
}

// #######################################################
// MAIN LOOP
// #######################################################
void loop() {
  
  int etekState=-1;
  int etekSwitch=-1;
  
  //***************************************  
  //****** RADIOHEAD RX CODE 
  //***************************************
  uint8_t RHbuf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t RHbuflen = sizeof(RHbuf);
  if (driver.recv(RHbuf, &RHbuflen)) // Non-blocking
  {
    RH_RECEIVE((uint8_t *)RHbuf, RHbuflen);
  } 
  
  //***************************************
  // RCremote RX code 
  //***************************************
  if (RCrecv.available()) {
    delay(1);
    RC_RECEIVE();  
  }
 
  //-------------------
  // IR REMOTE RX code 
  //-------------------

  if (irrecv.GetResults(&My_Decoder)) {  
   
    
    if(My_Decoder.decode()) {
      //GotOne=true;
        irValue = My_Decoder.value;
        irLen = My_Decoder.bits;
        codeType = My_Decoder.decode_type;
            //My_Decoder.decode();
    Serial.print(F("Received "));
    Serial.print(Pnames(codeType));
    Serial.print(F(" Value:0x"));
    Serial.println(My_Decoder.value, HEX);
    delay(500);
    }
    

    //Serial.flush();
    irrecv.resume(); 
  }
  else{}
  
  if(irValue!=0){
    codeCheck4(irValue);
    //IR_SEND(0x807F708F);  
    //irrecv.enableIRIn();
  }
  irValue=0;

  
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
      // save the last time you blinked the LED 
      previousMillis = currentMillis; 
//      IR_SEND(0x807FE817);
//      IR_SEND(0xF720DF); delay(100); IR_SEND(0xF7609F); delay(100);
//      IR_SEND(0xF720DF); delay(100); IR_SEND(0xF7609F); delay(100);
//      IR_SEND(0xF720DF); delay(100); IR_SEND(0xF7609F); delay(100);
      Serial.print(F("Free memory: "));
      Serial.println (freeMemory());
      
 
  }
  

}



// #######################################################
// FUNCTIONS
// #######################################################


//-------------------
// CODECHECK FUNCTION 
//-------------------

void codeCheck4(long int code)
{
  int arrCNT = 0;
  int ID=99;
  //char irbutton[MAX_CHAR_TXT_remote2];

  while(arrCNT<NUMBER_OF_remote2ELEMENTS ){
    bulbCODES bulbItem;
    remoteCODES remoteItem;
    remote2CODES remote2Item;
    memcpy_P (&bulbItem, &bulbArr[arrCNT], sizeof bulbItem);
    memcpy_P (&remoteItem, &remoteArr[arrCNT], sizeof remoteItem);
    memcpy_P (&remote2Item, &remote2Arr[arrCNT], sizeof remote2Item);
    


    if(code==remoteItem.remoteValArr && arrCNT<NUMBER_OF_remoteELEMENTS)
    {
        int myint = atoi(remoteItem.remoteTxtArr);
        if(myint>0 && myint<6)
        {
          ID = myint-1;   
          Serial.print(ID);  Serial.println(F(":ON"));
          delay(100);  
          RC_SEND(ID, 1); 
          //RC_SEND(ID, 1); 
          //IR_SEND(0x807FE817); 
          break;
        } 
        else if((myint>5 && myint<10) || myint==0)
        {
          if(myint==0) ID = 4; 
            else ID = myint-6;
            
          Serial.print(ID); Serial.println(F(":OFF"));  
          delay(100); 
          RC_SEND(ID, 0);  
          break; 
        }
        break;
    } 
    else if(code==bulbItem.bulbValArr && arrCNT<NUMBER_OF_bulbELEMENTS) 
    {
        Serial.print(F("RGB BULB: ")); Serial.println(bulbItem.bulbTxtArr);  
        break;
    } 
    else if(code==remote2Item.remoteValArr) 
    {
        char* irbutton = remote2Item.remoteTxtArr;
        Serial.print(F("IR BUTTON[")); Serial.print(remote2Item.remoteIDArr); 
        Serial.print(F("]: ")); Serial.println(irbutton);  
        
        int whloop=0;
        while(whloop<NUMBER_OF_whremoteELEMENTS)
        {
          whremoteCODES whremoteItem;
          memcpy_P (&whremoteItem, &whremoteArr[whloop], sizeof whremoteItem);
          if(strcmp(irbutton,whremoteItem.remoteTxtArr)==0)
          {
            Serial.println(F("TESTING XXXXXXXXX"));
            IR_SEND(whremoteItem.remoteValArr);
            irrecv.enableIRIn();
            break;
          }        
        whloop++;
        }
        break;
    } 
    else{}

    arrCNT++; 
    
  } // END WHILE LOOP	 
}  





//-------------------
// RADIOHEAD RX CODE 
//-------------------
void RH_RECEIVE(uint8_t *RHbuf, uint8_t RHbuflen)
{
    memset(RHMsg, '\0', sizeof(RHMsg));
    Serial.print("Got["); Serial.print(RHbuflen); Serial.print("]:");
    for (int i = 0; i < RHbuflen; i++){
      RHMsg[i] = char(RHbuf[i]); //Serial.print((char)RHbuf[i]);
    }
    Serial.print(RHMsg); Serial.println();
} 


//-------------------
// RCremote RX code 
//-------------------
void RC_RECEIVE()
{
  RCreadyflag==0;
  int value = RCrecv.getReceivedValue();
  Etek_code = RCrecv.getReceivedValue();
  Etek_length = RCrecv.getReceivedBitlength();
  char* b = dec2binWzerofill(Etek_code, Etek_length);
  bb = dec2binWzerofill(Etek_code, Etek_length);
  
  if (value == 0) 
  {
    Serial.print("Unknown encoding");
  } 
  else {
    Serial.print("Received rcREMOTE:");
    Serial.println( Etek_code );
    if(Etek_code==11097496 | Etek_code==4551939 ){
      Serial.println("BELL RUNG");
      //RH_BELLRUNG();}
  }
  
  delay(200);
  RCrecv.resetAvailable();
  
}}




//-------------------
// IR REMOTE SEND 
//-------------------
void IR_SEND(unsigned long ircode)
{
  
  Serial.println("IR_SEND"); delay(10);
  Serial.flush();
  RCrecv.disableReceive();
  RCsend.disableTransmit();
  driver.setModeIdle();
  delay(25);
    
    unsigned long int code_in = ircode;
    unsigned long int red = 0xF720DF;
    unsigned long int blue = 0xF7609F;
    unsigned long int down = 0x5743CC33;
    unsigned long int sleeps = 0x807FE817;
    
    for(int h=0; h<8; h++){
    irsendNEC.send(code_in);   // WESTINGHOUSE SLEEP
    delay(100);}
    delay(10); 
    
  //irrecv.enableIRIn();
  RCrecv.enableReceive(0);
  RCsend.enableTransmit(RCTX_PIN);
  RCsend.setProtocol(1);
  RCsend.setPulseLength(pulsetime);
  driver.setModeRx();
  delay(5);
  
}






//-------------------
// RC REMOTE SEND code 
//-------------------
void RC_SEND(int etekID, int etekState)
{
  RCrecv.disableReceive();
  RCsend.enableTransmit(RCTX_PIN);
  RCsend.setProtocol(1);
  RCsend.setPulseLength(pulsetime);
  
  if(etekState==1){
    txRemoteOn(etekID);
  }
  else if(etekState==0){
    txRemoteOff(etekID);
  }
  
  RCsend.disableTransmit();
  RCrecv.enableReceive(0);
  
}

void txRemoteAllOn()
{
  for(int w=0; w<5; w++)
  {
    etekCODES etekItem;
    memcpy_P (&etekItem, &etekArr[w], sizeof etekItem);
    for(int k = 0; k<2; k++)
    {
      RCsend.sendTriState(etekItem.etekCodeArr); 
      delay(2);
    } 
    delay(20);
  }
}

void txRemoteAllOff()
{
  for(int w=10; w<20; w++)
  {
    etekCODES etekItem;
    memcpy_P (&etekItem, &etekArr[w], sizeof etekItem);
    for(int k = 0; k<2; k++)
    {
      RCsend.sendTriState(etekItem.etekCodeArr); 
      delay(2);
    } 
    delay(20);
  }
}


void txRemoteOn(int p)
{
  etekCODES etekItem1;
  etekCODES etekItem2;
  memcpy_P (&etekItem1, &etekArr[p], sizeof etekItem1);
  memcpy_P (&etekItem2, &etekArr[p+5], sizeof etekItem2);
  for(int k = 0; k<2; k++)
  {
    RCsend.sendTriState(etekItem1.etekCodeArr); 
    RCsend.sendTriState(etekItem2.etekCodeArr); 
    delay(4);
  }	
  //Serial.print(F("EtekON:")); Serial.println(etekItem1.etekIDArr);
  delay(1);
}


void txRemoteOff(int j)
{ 
  etekCODES etekItem1;
  etekCODES etekItem2;
  memcpy_P (&etekItem1, &etekArr[j+10], sizeof etekItem1);
  memcpy_P (&etekItem2, &etekArr[j+15], sizeof etekItem2);
 for(int k = 0; k<2; k++)
 {
    RCsend.sendTriState(etekItem1.etekCodeArr); 
    RCsend.sendTriState(etekItem2.etekCodeArr); 
    delay(4);
  }
  //Serial.print(F("EtekOFF:")); Serial.println(etekItem1.etekIDArr);
  delay(1);
}



//-------------------
// HELPER FUNCTION 
//-------------------
static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength)
{
  static char bin[64]; 
  unsigned int i=0;

  while (Dec > 0) {
    bin[32+i++] = (Dec & 1 > 0) ? '1' : '0';
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j< bitLength; j++) {
    if (j >= bitLength - i) {
      bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
    }else {
      bin[j] = '0';
    }
  }
  bin[bitLength] = '\0';
  
  return bin;
}


//    irsendNEC.send(0x57437887);   delay(100);
//    irsendNEC.send(0x5743CC33);   delay(100);
//    irsendNEC.send(0x5743CC33);   delay(100);
//    irsendNEC.send(0x5743CC33);   delay(100);
//    irsendNEC.send(0x5743CC33);   delay(100);
//    irsendNEC.send(0x5743CC33);   delay(100);
//    irsendNEC.send(0x5743CC33);   delay(100);
 
//    for(int h=0; h<9; h++){
//    irsendNEC.send(sleeps);   // WESTINGHOUSE SLEEP
//    delay(200);}
//   
//    for(int h=0; h<3; h++){
//    irsendNEC.send(red);   // WESTINGHOUSE SLEEP
//    delay(200);}
//
//    for(int h=0; h<3; h++){
//    irsendNEC.send(blue);   // WESTINGHOUSE SLEEP
//    delay(200);}
//
//    for(int h=0; h<5; h++){
//    irsendNEC.send(down);   // WESTINGHOUSE SLEEP
//    delay(200);}
//    
//        for(int h=0; h<9; h++){
//    irsendNEC.send(sleeps);   // WESTINGHOUSE SLEEP
//    delay(50);}
//    delay(100);
//    
//    irsendNEC.send(0xF7609F);   delay(200);
//    irsendNEC.send(0xF720DF);   delay(200);
//    irsendNEC.send(0xF7609F);   delay(200);
//    irsendNEC.send(0xF720DF);   delay(200);
//    irsendNEC.send(0xF7609F);   delay(200);
//    irsendNEC.send(0xF720DF);   delay(200);
//    irsendNEC.send(0xF7609F);   delay(200);
//    irsendNEC.send(0xF720DF);   delay(200);  
    
//    for(int h=0; h<7; h++){
//    irsend.sendNEC(0xF720DF, 32);   // WESTINGHOUSE SLEEP
//    delay(50);}
//    delay(1000);
//        
//    for(int h=0; h<7; h++){
//    irsend.sendNEC(0xF7A05F, 32);   // WESTINGHOUSE SLEEP
//    delay(50);}
//    delay(1000);
//    
//    for(int h=0; h<7; h++){
//    irsend.sendNEC(0xF7609F, 32);   // WESTINGHOUSE SLEEP
//    delay(50);} 
//    delay(1000);
//    
//    for(int h=0; h<7; h++){
//    irsend.sendNEC(0xF7E01F, 32);   // WESTINGHOUSE SLEEP
//    delay(50);}
//    delay(1000);
//    
//
//  for(int h=0; h<7; h++){
//    irsend.sendNEC(0x807F708F, 32);   // WESTINGHOUSE SLEEP
//    delay(50);
//  }
//  
//    for(int h=0; h<7; h++){
//    irsend.sendNEC(0x807FF00F, 32);   // WESTINGHOUSE SLEEP
//    delay(50);
//  }


//-------------------
// IR REMOTE SEND 
//-------------------
//void IR_SEND3(){
//  //int rowsize = sizeof(bulbValArr)/sizeof(bulbValArr[0]);
//  Serial.println("IR_SEND"); 
//  delay(10);
//  RCrecv.disableReceive();
//  RCsend.disableTransmit();
//  driver.setModeIdle();
//  delay(5);
//  for(int b=0; b<17; b=b+5){
//    
//    bulbCODES bulbItem;
//    memcpy_P (&bulbItem, &bulbArr[b], sizeof bulbItem);
//    
//    for(int h=0; h<3; h++){
//      irsend.sendNEC(bulbItem.bulbValArr, 32);   // WESTINGHOUSE SLEEP
//      delay(50);
//    }
//    delay(100);
//  }
//  
//  irrecv.enableIRIn();
//  RCrecv.enableReceive(0);
//  RCsend.enableTransmit(RCTX_PIN);
//  RCsend.setProtocol(1);
//  RCsend.setPulseLength(pulsetime);
//  driver.setModeRx();
//  delay(5);
//}
