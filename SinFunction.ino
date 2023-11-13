#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_DRV2605.h>

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
QWIICMUX myMux;
Adafruit_DRV2605 drv[8];
unsigned long channelStartTime;
unsigned long currentTime;
unsigned long channelInterval = 40;
unsigned long channelDuration = 140;
unsigned long startTimes[4] ={0};
unsigned long endTimes[4] ={0};
unsigned long previousTime[4] = {0};
int channelFlag[4];
int m;
byte i = 0;
byte j = 0;
byte k = 0;
byte l = 0;
int currentChannel = 0;
int previousChannel = 0;
boolean channelInProgress = false;
bool i_Going_up = true;


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
  if (i>7) return;
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
  if (i>7) return;
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(1<<i);
  Wire.endTransmission();
}

void disablesingle(uint8_t i){
  if (i>7) return;
  endTimes[i] = millis();
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(0<<i);
  Wire.endTransmission();
}

void pwmModulation(uint8_t i){
  bool i_Going_up =true;
  unsigned long startTimes[i]= {millis()};
  Serial.println(i);
  if (startTimes[i]-channelStartTime<=70){
    long val = map((startTimes[i]-channelStartTime),0,70,0,127);
  }


}
void loop() {
  currentTime = millis();
  if (!channelInProgress){
    if(currentTime-channelStartTime>=channelInterval){
      singleActuator(currentChannel);
      drv[currentChannel].setRealtimeValue(0);
      channelFlag[currentChannel]=1;
      previousTime[currentChannel]= currentTime;
      previousChannel = currentChannel ;
      currentChannel = (currentChannel+1) % 4 ;
      channelStartTime = currentTime; 
      if (previousChannel==3){
        channelInProgress = true; 
      }
    }
  }
  if (channelFlag[0]==1){
    if (currentTime-previousTime[0]<= channelDuration*0.5)
    {
      singleActuator(0);
      drv[0].setRealtimeValue(map((currentTime-previousTime[0]),0,70,0,127));
    }
    else if (currentTime-previousTime[0]<channelDuration)
    {
      singleActuator(0);
      drv[0].setRealtimeValue(map((currentTime-previousTime[0]),70,140,127,0));
      Serial.println("hi");
      Serial.println(currentTime-previousTime[0]);
     } //
    else{  disablesingle(0);
      drv[0].stop();
      channelFlag[0]==0;
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<= channelDuration*0.5)
    {
      singleActuator(1);
      drv[1].setRealtimeValue(map((currentTime-previousTime[1]),0,70,0,127));
    }
    else if (currentTime-previousTime[1]<channelDuration)
    {
      singleActuator(1);
      drv[1].setRealtimeValue(map((currentTime-previousTime[1]),70,140,127,0));
      Serial.println("hi");
      Serial.println(currentTime-previousTime[1]);
     } //
    else{  disablesingle(1);
      drv[1].stop();
      channelFlag[1]==0;
    }
  }
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<= channelDuration*0.5)
    {
      singleActuator(2);
      drv[2].setRealtimeValue(map((currentTime-previousTime[0]),0,70,0,127));
    }
    else if (currentTime-previousTime[2]<channelDuration)
    {
      singleActuator(2);
      drv[2].setRealtimeValue(map((currentTime-previousTime[2]),70,140,127,0));
      Serial.println("hi");
      Serial.println(currentTime-previousTime[2]);
     } //
    else{  disablesingle(2);
      drv[2].stop();
      channelFlag[2]==0;
    }
  }
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<= channelDuration*0.5)
    {
      singleActuator(3);
      drv[3].setRealtimeValue(map((currentTime-previousTime[3]),0,70,0,127));
    }
    else if (currentTime-previousTime[3]<channelDuration)
    {
      singleActuator(3);
      drv[3].setRealtimeValue(map((currentTime-previousTime[3]),70,140,127,0));
      Serial.println("hi");
      Serial.println(currentTime-previousTime[3]);
     } //
    else{  disablesingle(3);
      drv[3].stop();
      channelFlag[3]==0;
      //channelInProgress =false;
    }
  }
  if(channelFlag[0]==0&&channelFlag[1]==0&&channelFlag[2]==0&&channelFlag[3]==0){
      channelInProgress = false;
    }
}

