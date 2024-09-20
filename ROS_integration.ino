#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>
#include <ros.h>
#include <std_msgs/Float64.h>

SFE_HMD_DRV2605L HMD[8];
#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
QWIICMUX myMux;
ros::NodeHandle nh;
std_msgs::Float64 currentMessage;
bool messageReceived = false;  // Flag to check if a message is received

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

void messageCb(const std_msgs::Float64& message) {
  currentMessage = message;  // Store the received message
  messageReceived = true;    // Set flag to indicate a message has been received
}
ros::Subscriber<std_msgs::Float64> sub("collision/nearest/distance", &messageCb);

void setup() {
  Wire.begin();
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  if (!myMux.begin()) {
    Serial.println("Mux not detected. Freezing...");
    while (1);
  }
  for (int i = 0; i<8; i++){
    singleActuator(i);
    HMD[i].begin();
    HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG,0xFF);
    HMD[i].Mode(0x05);
    HMD[i].MotorSelect(0x86);
    HMD[i].Library(6);
  }
  channelStartTime = millis();
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
void loop() {
  nh.spinOnce();
  delay(100);
  if (messageReceived){
    if (currentMessage.data<0.2){
        Serial.println(currentMessage.data, 4);  // Print the received message for debugging
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
      HMD[0].go();
    }
    else{
      singleActuator(0);
      HMD[0].RTP(0);
      disableActuator(0);
      HMD[0].stop();
      channelFlag[0] = 0; 
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<channelDuration){
      singleActuator(1);
      HMD[1].RTP(127);
      HMD[1].go();}
    else{
      singleActuator(1);
      HMD[1].RTP(0);
      disableActuator(1);
      HMD[1].stop();
      channelFlag[1] = 0;}}
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<channelDuration){
      singleActuator(2);
      HMD[2].RTP(127);
      HMD[2].go();}
    else{
      singleActuator(2);
      HMD[2].RTP(0);
      disableActuator(2);
      HMD[2].stop();
      channelFlag[2] = 0;}}
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<channelDuration){
      singleActuator(3);
      HMD[3].RTP(127);
      HMD[3].go();}
    else{
      singleActuator(3);
      HMD[3].RTP(0);
      disableActuator(3);
      HMD[3].stop();
      channelFlag[3] = 0;}
  }
  if(channelFlag[0]==0&&channelFlag[1]==0&&channelFlag[2]==0&&channelFlag[3]==0){
    channelInProgress = false;
  } 
    }
  }
}
