#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>
#include <Adafruit_DRV2605.h>


SFE_HMD_DRV2605L HMD[8];

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
uint8_t qwiicMuxAddress = QWIIC_MUX_DEFAULT_ADDRESS;

QWIICMUX myMux;
unsigned long channelStartTime;
unsigned long currentTime;
unsigned long channelInterval = 140;//70,140,90
unsigned long channelDuration = 280;//70,280,140
unsigned long startTimes[4] ={0};
unsigned long endTimes[4] ={0};
unsigned long previousTime[4] = {0};
int channelFlag[4];
byte i = 0;
byte j = 0;
byte k = 0;
byte l = 0;
int currentChannel = 0;
int diffcurrentChannel = 7;
int simulChannel = 7; 
int direction = -1; 

int diffsimulChannel = 3;

int previousChannel = 0;
int diffpreviousChannel = 7;
int previoussimulChannel =7; 

boolean channelInProgress = false;
char receivedChar;
int flag =0 ;

void setup() {
  Serial.begin(9600);
  Serial.println("Qwiic Mux Shield Read Example");
  digitalWrite(42, HIGH);

  delay(200);
  for (byte address =1; address < 127; ++address){
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (address != qwiicMuxAddress){
      Serial.println("hi");
      if (error == 0){
      disableotherchannel(address);
    }}
  }

  for (int i = 0; i<8; i++){
    singleActuator(i,qwiicMuxAddress);
    HMD[i].begin();
    HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG,0xFF);
    HMD[i].Mode(0x05);
    HMD[i].MotorSelect(0x86);
    HMD[i].Library(6);
  }
  for (int i=0,j=4;i<4,j<8;i++,j++){
    simul_Actuator(i,j,qwiicMuxAddress);
    HMD[i,j].begin();
    HMD[i,j].writeDRV2605L(OVERDRIVECLAMP_REG,0xFF);
    HMD[i,j].Mode(0x05);
    //HMD[i,j].Mode(0x00);
    HMD[i,j].MotorSelect(0x86);
    HMD[i,j].Library(6);
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


void singleActuator(uint8_t i,uint8_t qwiicMuxAddress){
  if (i>7) return;
  Wire.beginTransmission(qwiicMuxAddress);
  Wire.write(1<<i);
  Wire.endTransmission();
}
void disableActuator(uint8_t i,uint8_t qwiicMuxAddress){
  Wire.beginTransmission(qwiicMuxAddress);
  Wire.write(0<<i);
  Wire.endTransmission();
}
void disableotherchannel(uint8_t address){
  Wire.beginTransmission(address);
  Wire.write(0);
  Wire.endTransmission();
}

void simul_Actuator (uint8_t i, uint8_t j,uint8_t qwiicMuxAddress){
  if (i>4|j>7) return;
  Wire.beginTransmission(qwiicMuxAddress);
  Wire.write(1<<i|1<<j);
  
  Wire.endTransmission();
}
void disablesimul_Actuator(uint8_t i, uint8_t j,uint8_t qwiicMuxAddress){
  if (i>4|j>7) return;
  Wire.beginTransmission(qwiicMuxAddress);
  Wire.write(0<<i|0<<j);
  Wire.endTransmission();
}

void loop() {
  currentTime = millis();
  if (Serial.available()>0){
  recvOneChar();}
  // Serial.println(QWIIC_MUX_DEFAULT_ADDRESS);
  if (!channelInProgress && direction == 0 ){
    if (currentTime-channelStartTime>=channelInterval){
      simul_Actuator(currentChannel, simulChannel,qwiicMuxAddress);
      HMD[currentChannel,simulChannel].RTP(0);
      channelFlag[currentChannel] = 1; 
      previousTime[currentChannel] =currentTime;
      previousChannel = currentChannel; 
      previoussimulChannel = simulChannel;
      currentChannel = (currentChannel+1)%4;
      simulChannel =   7-currentChannel;
      channelStartTime = currentTime;
      if (previousChannel == 3){
        channelInProgress = true; 
      }}
  }
  if (direction == 0){
  if (channelFlag[0]==1){
    if (currentTime-previousTime[0]<channelDuration){
      simul_Actuator(0,7,qwiicMuxAddress);
      HMD[0,7].RTP(127);
    }
    else{
      simul_Actuator(0,7,qwiicMuxAddress);
      HMD[0,7].RTP(0);
      disablesimul_Actuator(0,7,qwiicMuxAddress);
      HMD[0,7].stop();
      channelFlag[0]=0;
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<channelDuration){
      simul_Actuator(1,6,qwiicMuxAddress);
      HMD[1,6].RTP(127);
    }
    else{
      simul_Actuator(1,6,qwiicMuxAddress);
      HMD[1,6].RTP(0);
      disablesimul_Actuator(1,6,qwiicMuxAddress);
      HMD[1,6].stop();
      channelFlag[1]=0;
    }
  }
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<channelDuration){
      simul_Actuator(2,5,qwiicMuxAddress);
      HMD[2,5].RTP(127);
    }
    else{
      simul_Actuator(2,5,qwiicMuxAddress);
      HMD[2,5].RTP(0);
      disablesimul_Actuator(2,5,qwiicMuxAddress);
      HMD[2,5].stop();
      channelFlag[2]=0;
    }
  }
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<channelDuration){
      simul_Actuator(3,4,qwiicMuxAddress);
      HMD[3,4].RTP(127);
    }
    else{
      simul_Actuator(3,4,qwiicMuxAddress);
      HMD[3,4].RTP(0);
      disablesimul_Actuator(3,4,qwiicMuxAddress);
      HMD[3,4].stop();
      channelFlag[3]=0;
      // Serial.println("what?");
    }
  }}
  if (!channelInProgress && direction == 2 ){
    if (currentTime-channelStartTime>=channelInterval){
      singleActuator(currentChannel,qwiicMuxAddress);
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
  if(direction ==2){
  if (channelFlag[0]==1){
    if (currentTime-previousTime[0]<channelDuration){
      singleActuator(0,qwiicMuxAddress);
      HMD[0].RTP(127);
    }
    else{
      singleActuator(0,qwiicMuxAddress);
      HMD[0].RTP(0);
      disableActuator(0,qwiicMuxAddress);
      HMD[0].stop();
      channelFlag[0] = 0; 
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<channelDuration){
      singleActuator(1,qwiicMuxAddress);
      HMD[1].RTP(127);}
    else{
      singleActuator(1,qwiicMuxAddress);
      HMD[1].RTP(0);
      disableActuator(1,qwiicMuxAddress);
      HMD[1].stop();
      channelFlag[1] = 0;}}
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<channelDuration){
      singleActuator(2,qwiicMuxAddress);
      HMD[2].RTP(127);}
    else{
      singleActuator(2,qwiicMuxAddress);
      HMD[2].RTP(0);
      disableActuator(2,qwiicMuxAddress);
      HMD[2].stop();
      channelFlag[2] = 0;}}
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<channelDuration){
      singleActuator(3,qwiicMuxAddress);
      HMD[3].RTP(127);}
    else{
      singleActuator(3,qwiicMuxAddress);
      HMD[3].RTP(0);
      disableActuator(3,qwiicMuxAddress);
      HMD[3].stop();
      channelFlag[3] = 0;}
  }
  }

  if (!channelInProgress && direction == 3 ){
    if (currentTime-channelStartTime>=channelInterval){
      diffcurrentChannel = 7 - currentChannel;
      singleActuator(diffcurrentChannel,qwiicMuxAddress);
      HMD[diffcurrentChannel].RTP(0);
      channelFlag[currentChannel]=1;
      previousTime[currentChannel] = currentTime;
      diffpreviousChannel = diffcurrentChannel;
      currentChannel = (currentChannel+1)%4;
      diffcurrentChannel = 7-currentChannel;
      channelStartTime = currentTime ; 
      if (diffpreviousChannel == 4){
        channelInProgress = true;
      }
    }
  }
  if(direction ==3){
  if (channelFlag[0]==1){
    if (currentTime-previousTime[0]<channelDuration){
      singleActuator(7,qwiicMuxAddress);
      HMD[7].RTP(127);
    }
    else{
      singleActuator(7,qwiicMuxAddress);
      HMD[7].RTP(0);
      disableActuator(7,qwiicMuxAddress);
      HMD[7].stop();
      channelFlag[0] = 0; 
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<channelDuration){
      singleActuator(6,qwiicMuxAddress);
      HMD[6].RTP(127);}
    else{
      singleActuator(6,qwiicMuxAddress);
      HMD[6].RTP(0);
      disableActuator(6,qwiicMuxAddress);
      HMD[6].stop();
      channelFlag[1] = 0;}}
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<channelDuration){
      singleActuator(5,qwiicMuxAddress);
      HMD[5].RTP(127);}
    else{
      singleActuator(5,qwiicMuxAddress);
      HMD[5].RTP(0);
      disableActuator(5,qwiicMuxAddress);
      HMD[5].stop();
      channelFlag[2] = 0;}}
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<channelDuration){
      singleActuator(4,qwiicMuxAddress);
      HMD[4].RTP(127);}
    else{
      singleActuator(4,qwiicMuxAddress);
      HMD[4].RTP(0);
      disableActuator(4,qwiicMuxAddress);
      HMD[4].stop();
      channelFlag[3] = 0;}
  }}
  //it will start 3-4 // 2-5 // 1-6 // 0-7
  if (!channelInProgress && direction == 1 ){
    if (currentTime-channelStartTime>=channelInterval){
      diffcurrentChannel = 7 - diffsimulChannel;
      simul_Actuator(diffsimulChannel, diffcurrentChannel,qwiicMuxAddress);
      HMD[diffsimulChannel,diffcurrentChannel].RTP(0);
      channelFlag[diffsimulChannel] = 1; 
      previousTime[diffsimulChannel] =currentTime;
      previousChannel = diffsimulChannel; 
      previoussimulChannel = diffcurrentChannel;
      diffsimulChannel = (diffsimulChannel-1)%4;
      diffcurrentChannel =   7-diffsimulChannel;
      channelStartTime = currentTime;
      if (previousChannel == 0){
        channelInProgress = true; 
      }}
  }
  if (direction == 1){
  if (channelFlag[3]==1){
    if (currentTime-previousTime[3]<channelDuration){
      simul_Actuator(3,4,qwiicMuxAddress);
      HMD[3,4].RTP(127);
    }
    else{
      simul_Actuator(3,4,qwiicMuxAddress);
      HMD[3,4].RTP(0);
      disablesimul_Actuator(3,4,qwiicMuxAddress);
      HMD[3,4].stop();
      channelFlag[3]=0;
    }
  }
  if (channelFlag[2]==1){
    if (currentTime-previousTime[2]<channelDuration){
      simul_Actuator(2,5,qwiicMuxAddress);
      HMD[2,5].RTP(127);
    }
    else{
      simul_Actuator(2,5,qwiicMuxAddress);
      HMD[2,5].RTP(0);
      disablesimul_Actuator(2,5,qwiicMuxAddress);
      HMD[2,5].stop();
      channelFlag[2]=0;
    }
  }
  if (channelFlag[1]==1){
    if (currentTime-previousTime[1]<channelDuration){
      simul_Actuator(1,6,qwiicMuxAddress);
      HMD[1,6].RTP(127);
    }
    else{
      simul_Actuator(1,6,qwiicMuxAddress);
      HMD[1,6].RTP(0);
      disablesimul_Actuator(1,6,qwiicMuxAddress);
      HMD[1,6].stop();
      channelFlag[1]=0;
    }
  }
  if (channelFlag[0]==1){
    if (currentTime-previousTime[0]<channelDuration){
      simul_Actuator(0,7,qwiicMuxAddress);
      HMD[0,7].RTP(127);
    }
    else{
      simul_Actuator(0,7,qwiicMuxAddress);
      HMD[0,7].RTP(0);
      disablesimul_Actuator(0,7,qwiicMuxAddress);
      HMD[0,7].stop();
      channelFlag[0]=0;
      // Serial.println("what?");
    }
  }}
  if(channelFlag[0]==0&&channelFlag[1]==0&&channelFlag[2]==0&&channelFlag[3]==0){
    channelInProgress = false;
    currentChannel = 0;
    diffsimulChannel =3;
    direction = -1;
    simulChannel = 7;
    // Serial.print("hi");
    previousTime[4] = {0};
}
}
int recvOneChar(){
  if(Serial.available()>0){
    char receivedChar = Serial.read();
    Serial.print("You Tyeped");
    Serial.println(receivedChar);
    if (receivedChar == 'a'){
      direction = 0 ; 
      qwiicMuxAddress = 112;
    }
    if (receivedChar == 'b'){
      direction = 1 ; 
      qwiicMuxAddress = 112;
    }
    if(receivedChar == 'c'){
      direction = 0;
      qwiicMuxAddress = 113;
    }
    if(receivedChar == 'd'){
      direction = 1;
      qwiicMuxAddress = 113;
    }

    if (receivedChar == 'e'){
      direction = 2 ; 
      qwiicMuxAddress = 112;
    }
    if (receivedChar == 'f'){
      direction = 3 ; 
      qwiicMuxAddress = 112;
    }
  }
  return direction, qwiicMuxAddress;
}