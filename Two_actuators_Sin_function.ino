#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_DRV2605.h>

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
QWIICMUX myMux;
Adafruit_DRV2605 drv[8];
unsigned long channelStartTime;
unsigned long currentTime;
unsigned long channelInterval = 100;
unsigned long channelDuration = 140;
unsigned long startTimes[4] ={0};
unsigned long endTimes[4] ={0};
unsigned long previousTime[4] = {0};
int channelFlag[4];
byte i = 0;
byte j = 0;
byte k = 0;
byte l = 0;
int currentChannel = 0;

int previousChannel = 0;
boolean channelInProgress = false;
char receivedChar;
int count = 0; 
int flag =0 ;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic Mux Shield Read Example");
  delay(200);
  Wire.begin();
  if (myMux.begin() == false)
  {
    //Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  }
  byte currentPortNumber = myMux.getPort();
  for (int i=0;i<8;i++){
    performI2CChannelOperation(i);
  }
  channelStartTime = millis();
  currentTime = millis();  // put your setup code here, to run once:
}

void performI2CChannelOperation(uint8_t i){
  singleActuator(i);
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(1<<i);
  Wire.endTransmission();
  drv[i].begin();
  drv[i].setMode(DRV2605_MODE_REALTIME);
  drv[i].writeRegister8(DRV2605_REG_OVERDRIVE,0xFF);
  drv[i].writeRegister8(DRV2605_REG_FEEDBACK, 0x86);
  drv[i].useLRA();
  drv[i].selectLibrary(6);
}

void singleActuator(uint8_t i){
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(1<<i);
  Wire.endTransmission();
}

void disablesingle(uint8_t i){
  endTimes[i] = millis();
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(0<<i);
  Wire.endTransmission();
}
void loop() {
    currentTime = millis();
    if (!channelInProgress){
      if(currentTime-channelStartTime>=channelInterval){
        //Serial.println("************");
        singleActuator(currentChannel);
        drv[currentChannel].setRealtimeValue(0);
        channelFlag[currentChannel]=1;
        previousTime[currentChannel]= currentTime;
        previousChannel = currentChannel ;
        currentChannel = currentChannel+2 ;
        channelStartTime = currentTime; 
        if (previousChannel==2){
          channelInProgress = true; 
        }
      }
    }
    if (channelFlag[0]==1){
      if (currentTime-previousTime[0]<channelDuration*0.5)
      {
        singleActuator(0);
        Serial.println(currentTime);
        drv[0].setRealtimeValue(map((currentTime-previousTime[0]),0,70,0,127));
      }
      else if (currentTime-previousTime[0]<channelDuration){
        singleActuator(0);
        drv[0].setRealtimeValue(map((currentTime-previousTime[0]),70,140,127,0));
      }

      else{
        singleActuator(0);
        drv[0].setRealtimeValue(0);
        disablesingle(0);
        drv[0].stop();
        channelFlag[0]==0;
      }
    }
    if (channelFlag[2]==1){
      if (currentTime-previousTime[2]<channelDuration*0.5)
      {
        singleActuator(2);
        Serial.println(currentTime);
        drv[2].setRealtimeValue(map((currentTime-previousTime[2]),0,70,0,127));
      }
      else if (currentTime-previousTime[2]<channelDuration{
        singleActuator(2);
        Serial.println(currentTime);
        drv[2].setRealtimeValue(map((currentTime-previousTime[2]),70,140,127,0));
      }
      else{
        singleActuator(2);
        drv[2].setRealtimeValue(0);
        disablesingle(2);
        drv[2].stop();
        channelFlag[2]==0;
      }
    }

     if(channelFlag[0]==0&&channelFlag[2]==0){
      channelInProgress = false;
    }
}


void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
    }
}

int convert(){
  if (receivedChar == 'a'){
    flag= 1;
    count =0;
  }
}