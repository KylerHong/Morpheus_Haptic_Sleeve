#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>
#include <Adafruit_DRV2605.h>


SFE_HMD_DRV2605L HMD[8];

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
QWIICMUX myMux;
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
  Serial.begin(9600);
  Serial.println("Qwiic Mux Shield Read Example");
  digitalWrite(42, HIGH);
  disableotherchannel(0x71);
  disableotherchannel(0x72);
  disableotherchannel(0x73);
  delay(200);
  for (int i = 0; i<8; i++){
    HMD[i].begin();
    HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG,0xFF);
    HMD[i].Mode(0x05);
    HMD[i].MotorSelect(0x86);
    HMD[i].Library(6);
  }

  // byte currentPortNumber = myMux.getPort();
  // put your setup code here, to run once:// put your setup code here, to run once:
  if (myMux.begin() == false)
  {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  } 
  Serial.println("MUX 1 detected");
  channelStartTime = millis();
  currentTime = millis();  // put your setup code here, to run once:
}

void performI2CChannelOperation(uint8_t i){
  singleActuator(i);
}

void singleActuator(uint8_t i){
  if (i>7) return;
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(1<<i);
  Wire.endTransmission();
}
void disableActuator(uint8_t i){
  Wire.beginTransmission(QWIIC_MUX_DEFAULT_ADDRESS);
  Wire.write(0<<i);
  Wire.endTransmission();
}
void disableotherchannel(uint8_t address){
  Wire.beginTransmission(address);
  Wire.write(0);
  Wire.endTransmission();
}
void loop() {
  
  currentTime = millis();
  if (!channelInProgress){
    if (currentTime-channelStartTime>=channelInterval){
      singleActuator(currentChannel);
      HMD[currentChannel].RTP(0);
      channelFlag[currentChannel]=1;
      previousTime[currentChannel] = currentTime;
      previousChannel = currentChannel;
      currentChannel = (currentChannel+1)%4;
      channelStartTime = currentTime ; 
      if (previousChannel == 3){
        channelInProgress = true;
      }
    }
  }
  if (channelFlag[0]==1){
    if (currentTime-previousTime[0]<channelDuration){
      singleActuator(0);
      HMD[0].RTP(127);
    }
    else{
      singleActuator(0);
      HMD[0].RTP(0);
      disableActuator(0);
      HMD[0].stop();
      channelFlag[0] == 0; 
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<channelDuration){
      singleActuator(1);
      HMD[1].RTP(127);}
    else{
      singleActuator(1);
      HMD[1].RTP(0);
      disableActuator(1);
      HMD[1].stop();
      channelFlag[1] == 0;}}
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<channelDuration){
      singleActuator(2);
      HMD[2].RTP(127);}
    else{
      singleActuator(2);
      HMD[2].RTP(0);
      disableActuator(2);
      HMD[2].stop();
      channelFlag[2] == 0;}}
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<channelDuration){
      singleActuator(3);
      HMD[3].RTP(127);}
    else{
      singleActuator(3);
      HMD[3].RTP(0);
      disableActuator(3);
      HMD[3].stop();
      channelFlag[3] == 0;}
  }
  if(channelFlag[0]==0&&channelFlag[1]==0&&channelFlag[2]==0&&channelFlag[3]==0){
    channelInProgress = false;
  } 

}
