#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Sparkfun_DRV2605L.h>
#include <Adafruit_DRV2605.h>


SFE_HMD_DRV2605L HMD[8];
unsigned long activationTimes[4];     // Store activation times for each channel pair
int phase1Count = 0;                  // Track the number of Phase 1 repetitions
int phase2Count = 0;                  // Track the number of Phase 2 repetitions

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
#define QWIIC_MUX_PHASE2_ADDRESS 0x71  // Define address for Phase 2
bool phase1CompleteTransitionActive = false;  // New flag for Phase 1 complete transition
unsigned long phase1CompleteTransitionStartTime = 0;  // Time to track Phase 1 complete transition

uint8_t qwiicMuxAddress = QWIIC_MUX_DEFAULT_ADDRESS;
int repeatCount = 0; // Variable to track how many times the sequence has been repeated
int maxRepeats = 3;  // Maximum number of times to repeat the sequence
unsigned long lastTime = 0;  // Variable to store the last time a channel was activated
bool transitionDelayComplete = false; // Flag to handle the delay between Phase 1 and Phase 2
unsigned long transitionStartTime = 0; // To track when the delay starts
bool transitionDelayActive = false;   // Flag to track transition delay between phases
unsigned long phase1TransitionStartTime = 0; // Time when the transition between phases starts
bool phase1TransitionActive = false;   // New flag to handle transition between Phase 1 and Phase 2

QWIICMUX myMux;
unsigned long channelStartTime;
unsigned long currentTime;
unsigned long channelInterval = 140;//70,140,90
unsigned long channelDuration = 280;//70,280,140
unsigned long lastActivationTime = 0;  // Variable to track the last activation time
int activeChannels = 0;                // Counter for active channels
bool sequenceComplete = false;         // Flag to indicate if a sequence is complete
bool deactivated[4];                  // Track if each channel has been deactivated
bool isPairPatternComplete = false;   // Flag to indicate if the first pattern of pairs is complete
bool isPhase1Complete = false;        // Flag to indicate if Phase 1 is complete
bool isPhase2Complete = false;        // Flag to indicate if Phase 2 is complete

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
int count = 0;

void setup() {
  Serial.begin(9600);
    for (int i = 0; i < 4; i++) {
    deactivated[i] = false;
  }

  Serial.println("Qwiic Mux Shield Read Example");
  digitalWrite(42, HIGH);
  digitalWrite(43, HIGH);
  digitalWrite(44, HIGH);
  digitalWrite(45, HIGH);

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

void transitionToPhase2() {
  // Initialize Phase 2 with a new address (0x71)
  qwiicMuxAddress = QWIIC_MUX_PHASE2_ADDRESS;
  for (byte address =1; address < 127; ++address){
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (address != qwiicMuxAddress){
      if (error == 0){
      disableotherchannel(address);
    }}
  }

  Serial.println("Mux 2 (0x71) detected for Phase 2");

  // Setup actuators for Phase 2 (individual channels)
  for (int i = 0; i < 4; i++) {
    singleActuator(i, qwiicMuxAddress);
    HMD[i].begin();
    HMD[i].writeDRV2605L(OVERDRIVECLAMP_REG, 0xFF);
    HMD[i].Mode(0x05);
    HMD[i].MotorSelect(0x86);
    HMD[i].Library(6);
  }

  Serial.println("Setup for Phase 2 completed. Ready to start Phase 2.");
  channelStartTime = millis();
  currentTime = millis(); 
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

// void loop() {
//   currentTime = millis();

//   // Stop the loop after 3 repetitions
//   if (repeatCount >= maxRepeats) {
//     Serial.println("Sequence completed 3 times. Stopping...");
//     while (true); // Stop the program
//   }

//   // Check if it's time to activate the next pair of channels
//   if (!sequenceComplete && (currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
//     // Activate the next pair of channels
//     simul_Actuator(currentChannel, 7 - currentChannel, qwiicMuxAddress);
//     HMD[currentChannel, 7 - currentChannel].RTP(127); // Start the channel with a strength
//     activationTimes[currentChannel] = currentTime;    // Record the activation time
//     deactivated[currentChannel] = false;              // Reset the deactivation state for this channel
//     Serial.print("Activating channels [");
//     Serial.print(currentChannel);
//     Serial.print(", ");
//     Serial.print(7 - currentChannel);
//     Serial.println("] at time: " + String(currentTime) + "ms");

//     // Update for the next channel
//     lastActivationTime = currentTime;
//     currentChannel++;
//   }

//   // Check if it's time to deactivate each pair after 280ms
//   for (int i = 0; i < currentChannel; i++) {
//     if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
//       // Deactivate the channel pair after 280ms
//       simul_Actuator(i, 7 - i, qwiicMuxAddress);
//       HMD[i, 7 - i].RTP(0);  // Stop the channel
//       disablesimul_Actuator(i, 7 - i, qwiicMuxAddress);
//       HMD[i, 7 - i].stop();
//       Serial.print("Deactivating channels [");
//       Serial.print(i);
//       Serial.print(", ");
//       Serial.print(7 - i);
//       Serial.println("] at time: " + String(currentTime) + "ms");

//       // Mark the channel as deactivated
//       deactivated[i] = true;
//     }
//   }

//   // Check if all pairs have been activated and deactivated
//   if (currentChannel == 4 && allDeactivated()) {
//     delay(100);  // 100 ms delay after the last pair [3,4] is deactivated
//     sequenceComplete = true;  // Mark the sequence as complete
//     repeatCount++;            // Increment the repeat counter
//     currentChannel = 0;       // Reset for the next sequence
//     Serial.println("100ms delay complete. Ready for next sequence.");
//     sequenceComplete = false; // Reset the flag for the next cycle
//     lastActivationTime = millis(); // Reset timer for the next cycle

//     // Reset deactivation states for the next sequence
//     for (int i = 0; i < 4; i++) {
//       deactivated[i] = false;
//     }
//   }
// }
// void loop() {
//   currentTime = millis();

//   // Stop the loop after 3 repetitions
//   if (repeatCount >= maxRepeats) {
//     Serial.println("Sequence completed 3 times. Stopping...");
//     while (true); // Stop the program
//   }

//   // Phase 1: Activate pairs of channels [0,7], [1,6], [2,5], [3,4]
//   if (!isPairPatternComplete) {
//     // Check if it's time to activate the next pair of channels
//     if (!sequenceComplete && (currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
//       // Activate the next pair of channels
//       simul_Actuator(currentChannel, 7 - currentChannel, qwiicMuxAddress);
//       HMD[currentChannel, 7 - currentChannel].RTP(127); // Start the channel with a strength
//       activationTimes[currentChannel] = currentTime;    // Record the activation time
//       deactivated[currentChannel] = false;              // Reset the deactivation state for this channel
//       Serial.print("Activating channels [");
//       Serial.print(currentChannel);
//       Serial.print(", ");
//       Serial.print(7 - currentChannel);
//       Serial.println("] at time: " + String(currentTime) + "ms");

//       // Update for the next channel
//       lastActivationTime = currentTime;
//       currentChannel++;
//     }

//     // Check if it's time to deactivate each pair after 280ms
//     for (int i = 0; i < currentChannel; i++) {
//       if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
//         // Deactivate the channel pair after 280ms
//         simul_Actuator(i, 7 - i, qwiicMuxAddress);
//         HMD[i, 7 - i].RTP(0);  // Stop the channel
//         disablesimul_Actuator(i, 7 - i, qwiicMuxAddress);
//         HMD[i, 7 - i].stop();
//         Serial.print("Deactivating channels [");
//         Serial.print(i);
//         Serial.print(", ");
//         Serial.print(7 - i);
//         Serial.println("] at time: " + String(currentTime) + "ms");

//         // Mark the channel as deactivated
//         deactivated[i] = true;
//       }
//     }

//     // Check if all pairs have been activated and deactivated
//     if (currentChannel == 4 && allDeactivated()) {
//       delay(100);  // 100 ms delay after the last pair [3,4] is deactivated
//       sequenceComplete = true;  // Mark the sequence as complete
//       currentChannel = 0;       // Reset for the next sequence
//       Serial.println("100ms delay complete. Ready for next sequence.");
//       sequenceComplete = false; // Reset the flag for the next cycle
//       lastActivationTime = millis(); // Reset timer for the next cycle

//       // Reset deactivation states for the next sequence
//       for (int i = 0; i < 4; i++) {
//         deactivated[i] = false;
//       }

//       // Switch to the second pattern: individual channels [0], [1], [2], [3]
//       isPairPatternComplete = true;
//     }
//   }

//   // Phase 2: Activate individual channels [0], [1], [2], [3] with the same timing
//   if (isPairPatternComplete) {
//     // Check if it's time to activate the next individual channel
//     if ((currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
//       // Activate the next individual channel
//       singleActuator(currentChannel, qwiicMuxAddress);
//       HMD[currentChannel].RTP(127); // Start the individual channel with a strength
//       activationTimes[currentChannel] = currentTime;    // Record the activation time
//       deactivated[currentChannel] = false;              // Reset the deactivation state for this channel
//       Serial.print("Activating individual channel [");
//       Serial.print(currentChannel);
//       Serial.println("] at time: " + String(currentTime) + "ms");

//       // Update for the next individual channel
//       lastActivationTime = currentTime;
//       currentChannel++;
//     }

//     // Check if it's time to deactivate each individual channel after 280ms
//     for (int i = 0; i < currentChannel; i++) {
//       if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
//         // Deactivate the individual channel after 280ms
//         singleActuator(i, qwiicMuxAddress);
//         HMD[i].RTP(0);  // Stop the individual channel
//         disableActuator(i, qwiicMuxAddress);
//         HMD[i].stop();
//         Serial.print("Deactivating individual channel [");
//         Serial.print(i);
//         Serial.println("] at time: " + String(currentTime) + "ms");

//         // Mark the individual channel as deactivated
//         deactivated[i] = true;
//       }
//     }

//     // Check if all individual channels have been activated and deactivated
//     if (currentChannel == 4 && allDeactivated()) {
//       delay(100);  // 100 ms delay after the last individual channel is deactivated
//       sequenceComplete = true;  // Mark the sequence as complete
//       repeatCount++;            // Increment the repeat counter
//       currentChannel = 0;       // Reset for the next cycle
//       Serial.println("100ms delay complete. Ready for next pattern.");
      
//       // Reset deactivation states for the next sequence
//       for (int i = 0; i < 4; i++) {
//         deactivated[i] = false;
//       }

//       // Reset flag for the next full cycle (both pair and individual phases)
//       isPairPatternComplete = false;
//       sequenceComplete = false;
//       lastActivationTime = millis();
//     }
//   }
// }

// // Helper function to check if all channels are deactivated
// bool allDeactivated() {
//   for (int i = 0; i < 4; i++) {
//     if (!deactivated[i]) {
//       return false;
//     }
//   }
//   return true;
// }

// void loop() {
//   currentTime = millis();

//   // Handle the transition delay between Phase 1 and Phase 2
//   if (transitionDelayActive) {
//     if (currentTime - transitionStartTime >= 100) {
//       transitionDelayActive = false;  
//       isPhase1Complete = true;             // End the delay
//       transitionStartTime = 0;           // Reset the timer
//       currentChannel = 0;                // Reset the current channel for Phase 2
//       lastActivationTime = millis();     // Reset activation timing
//       qwiicMuxAddress = QWIIC_MUX_PHASE2_ADDRESS;

//       Serial.println("Transition complete. Starting Phase 2 (Individual Channels).");
//     } else {
//       // Still in the delay, return and do nothing until the delay completes
//       return;
//     }
//   }
//   // If Phase 1 has just completed, activate a transition delay before starting Phase 2
//   if (phase1TransitionActive) {
//     if (currentTime - phase1TransitionStartTime >= 500) {  // Set a 500ms transition delay
//       phase1TransitionActive = false;  // End transition
//       transitionToPhase2();  // Switch to Phase 2 mux configuration
//       Serial.println("Mux 2 (0x71) detected for Phase 2");
//       return;
//     } else {
//       return;  // Exit the loop during the transition delay
//     }
//   }
//   // Check if Phase 1 is complete (after three repetitions)
//   if (phase1Count >= maxRepeats && isPhase1Complete && !transitionDelayActive) {
//     Serial.println("Phase 1 (Pairs) completed 3 times. Moving to Phase 2 after delay...");
//     transitionStartTime = millis();  // Start the transition delay
//     delay(100);
//     transitionToPhase2();

//     transitionDelayActive = true;    // Activate the delay
//      // Switch to Phase 2 mux configuration
//     return;  // Exit the loop temporarily to handle the delay
//   }

//   // Check if Phase 2 is complete (after three repetitions)
//   if (phase2Count >= maxRepeats && isPhase2Complete) {
//     Serial.println("Phase 2 (Individuals) completed 3 times. Stopping...");
//     while (true); // Stop the program
//   }

//   // Phase 1: Activate pairs of channels [0,7], [1,6], [2,5], [3,4]
//   if (!isPhase1Complete && phase1Count < maxRepeats) {
//     // Check if it's time to activate the next pair of channels
//     if ((currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
//       // Activate the next pair of channels
//       simul_Actuator(currentChannel, 7 - currentChannel, qwiicMuxAddress);
//       HMD[currentChannel, 7 - currentChannel].RTP(127); // Start the channel with a strength
//       activationTimes[currentChannel] = currentTime;    // Record the activation time
//       deactivated[currentChannel] = false;              // Reset the deactivation state for this channel
//       Serial.print("Activating pair [");
//       Serial.print(currentChannel);
//       Serial.print(", ");
//       Serial.print(7 - currentChannel);
//       Serial.println("] at time: " + String(currentTime) + "ms");

//       // Update for the next channel
//       lastActivationTime = currentTime;
//       currentChannel++;
//     }

//     // Check if it's time to deactivate each pair after 280ms
//     for (int i = 0; i < currentChannel; i++) {
//       if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
//         // Deactivate the channel pair after 280ms
//         simul_Actuator(i, 7 - i, qwiicMuxAddress);
//         HMD[i, 7 - i].RTP(0);  // Stop the channel
//         disablesimul_Actuator(i, 7 - i, qwiicMuxAddress);
//         HMD[i, 7 - i].stop();
//         Serial.print("Deactivating pair [");
//         Serial.print(i);
//         Serial.print(", ");
//         Serial.print(7 - i);
//         Serial.println("] at time: " + String(currentTime) + "ms");

//         // Mark the channel as deactivated
//         deactivated[i] = true;
//       }
//     }

//     // Check if all pairs have been activated and deactivated
//     if (currentChannel == 4 && allDeactivated()) {
//       delay(100);  // 100 ms delay after the last pair [3,4] is deactivated
//       currentChannel = 0;       // Reset for the next sequence
//       Serial.println("100ms delay complete. Ready for next Phase 1 repetition.");
      
//       // Increment Phase 1 count
//       phase1Count++;
      
//       // Check if Phase 1 should complete after 3 repetitions
//       if (phase1Count == maxRepeats) {
//         isPhase1Complete = true;
//       }

//       // Reset deactivation states for the next sequence
//       for (int i = 0; i < 4; i++) {
//         deactivated[i] = false;
//       }

//       lastActivationTime = millis(); // Reset timer for the next cycle
//     }
//   }

//   // Phase 2: Activate individual channels [0], [1], [2], [3]
//   if (isPhase1Complete && !isPhase2Complete && phase2Count < maxRepeats && !transitionDelayActive) {
//     // Check if it's time to activate the next individual channel
//     if ((currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
//       // Activate the next individual channel
//       singleActuator(currentChannel, qwiicMuxAddress);
//       HMD[currentChannel].RTP(127); // Start the individual channel with a strength
//       activationTimes[currentChannel] = currentTime;    // Record the activation time
//       deactivated[currentChannel] = false;              // Reset the deactivation state for this channel
//       Serial.print("Activating individual channel [");
//       Serial.print(currentChannel);
//       Serial.println("] at time: " + String(currentTime) + "ms");

//       // Update for the next individual channel
//       lastActivationTime = currentTime;
//       currentChannel++;
//     }

//     // Check if it's time to deactivate each individual channel after 280ms
//     for (int i = 0; i < currentChannel; i++) {
//       if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
//         // Deactivate the individual channel after 280ms
//         singleActuator(i, qwiicMuxAddress);
//         HMD[i].RTP(0);  // Stop the individual channel
//         disableActuator(i, qwiicMuxAddress);
//         HMD[i].stop();
//         Serial.print("Deactivating individual channel [");
//         Serial.print(i);
//         Serial.println("] at time: " + String(currentTime) + "ms");

//         // Mark the individual channel as deactivated
//         deactivated[i] = true;
//       }
//     }

//     // Check if all individual channels have been activated and deactivated
//     if (currentChannel == 3 && allDeactivated()) {
//       delay(100);  // 100 ms delay after the last individual channel is deactivated
//       currentChannel = 0;       // Reset for the next sequence
//       Serial.println("100ms delay complete. Ready for next Phase 2 repetition.");
      
//       // Increment Phase 2 count
//       phase2Count++;
      
//       // Check if Phase 2 should complete after 3 repetitions
//       if (phase2Count == maxRepeats) {
//         isPhase2Complete = true;
//       }

//       // Reset deactivation states for the next sequence
//       for (int i = 0; i < 4; i++) {
//         deactivated[i] = false;
//       }

//       lastActivationTime = millis(); // Reset timer for the next cycle
//     }
//   }
// }
void loop() {
  currentTime = millis();

  // Handle the transition delay between Phase 1 and Phase 2
  if (transitionDelayActive) {
    if (currentTime - transitionStartTime >= 100) {
      transitionDelayActive = false;
      phase1CompleteTransitionActive = true;
      isPhase1Complete = true;  // End the delay
      transitionStartTime = 0;  // Reset the timer
      currentChannel = 0;       // Reset the current channel for Phase 2
      lastActivationTime = millis();  // Reset activation timing
      qwiicMuxAddress = QWIIC_MUX_PHASE2_ADDRESS;
      transitionToPhase2();  // Switch to Phase 2 mux configuration
      Serial.println("Transition complete. Starting Phase 2 (Individual Channels).");
    } else {
      // Still in the delay, return and do nothing until the delay completes
      return;
    }
  }
  // Check if Phase 1 is complete (after three repetitions)
  if (phase1Count >= maxRepeats && !phase1CompleteTransitionActive) {
    Serial.println("Phase 1 (Pairs) completed 3 times. Moving to Phase 2 after delay...");
    phase1CompleteTransitionStartTime = millis();  // Start the transition delay
    phase1CompleteTransitionActive = true;  // Activate the transition delay
    transitionDelayActive   = true; 
    return;  // Exit the loop temporarily to handle the delay
  }
  // Handle delay between Phase 1 completion and Phase 2 start
  // if (phase1CompleteTransitionActive) {
  //   if (currentTime - phase1CompleteTransitionStartTime >= 50) {  // 500ms transition delay
  //     phase1CompleteTransitionActive = false;  // End the transition delay
  //     transitionDelayActive =false;
  //     Serial.println("Mux 2 (0x71) detected for Phase 2");
  //     return;
  //   } else {
  //     return;  // Wait during the transition delay
  //   }
  // }
  // Check if Phase 2 is complete (after three repetitions)
  if (phase2Count >= maxRepeats && isPhase2Complete) {
    Serial.println("Phase 2 (Individuals) completed 3 times. Stopping...");
    while (true);  // Stop the program
  }

  // Phase 1: Activate pairs of channels [0,7], [1,6], [2,5], [3,4]
  if (!isPhase1Complete && phase1Count < maxRepeats) {
    // Check if it's time to activate the next pair of channels
    if ((currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
      // Activate the next pair of channels
      simul_Actuator(currentChannel, 7 - currentChannel, qwiicMuxAddress);
      HMD[currentChannel, 7 - currentChannel].RTP(127);  // Start the channel with a strength
      activationTimes[currentChannel] = currentTime;     // Record the activation time
      deactivated[currentChannel] = false;               // Reset the deactivation state for this channel
      Serial.print("Activating pair [");
      Serial.print(currentChannel);
      Serial.print(", ");
      Serial.print(7 - currentChannel);
      Serial.println("] at time: " + String(currentTime) + "ms");

      // Update for the next channel
      lastActivationTime = currentTime;
      currentChannel++;
    }

    // Check if it's time to deactivate each pair after 280ms
    for (int i = 0; i < currentChannel; i++) {
      if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
        // Deactivate the channel pair after 280ms
        simul_Actuator(i, 7 - i, qwiicMuxAddress);
        HMD[i, 7 - i].RTP(0);  // Stop the channel
        disablesimul_Actuator(i, 7 - i, qwiicMuxAddress);
        HMD[i, 7 - i].stop();
        Serial.print("Deactivating pair [");
        Serial.print(i);
        Serial.print(", ");
        Serial.print(7 - i);
        Serial.println("] at time: " + String(currentTime) + "ms");

        // Mark the channel as deactivated
        deactivated[i] = true;
      }
    }

    // Check if all pairs have been activated and deactivated
    if (currentChannel == 4 && allDeactivated()) {
      delay(100);  // 100 ms delay after the last pair [3,4] is deactivated
      currentChannel = 0;  // Reset for the next sequence
      Serial.println("100ms delay complete. Ready for next Phase 1 repetition.");
      // Increment Phase 1 count
      phase1Count++;
      // Check if Phase 1 should complete after 3 repetitions

      // Reset deactivation states for the next sequence
      for (int i = 0; i < 4; i++) {
        deactivated[i] = false;
      }

      lastActivationTime = millis();  // Reset timer for the next cycle
    }
  }

  // Phase 2: Activate individual channels [0], [1], [2], [3]
  if (isPhase1Complete && !isPhase2Complete && phase2Count < maxRepeats && !transitionDelayActive) {
    // Check if it's time to activate the next individual channel
    Serial.println("Entering Phase 2 activation...");  // Debug message to confirm phase entry

    if ((currentTime - lastActivationTime >= channelInterval) && currentChannel < 4) {
      // Activate the next individual channel
      singleActuator(currentChannel, qwiicMuxAddress);
      HMD[currentChannel].RTP(127);  // Start the individual channel with a strength
      activationTimes[currentChannel] = currentTime;     // Record the activation time
      deactivated[currentChannel] = false;               // Reset the deactivation state for this channel
      Serial.print("Activating individual channel [");
      Serial.print(currentChannel);
      Serial.println("] at time: " + String(currentTime) + "ms");

      // Update for the next individual channel
      lastActivationTime = currentTime;
      currentChannel++;
    }

    // Check if it's time to deactivate each individual channel after 280ms
    for (int i = 0; i < currentChannel; i++) {
      if (!deactivated[i] && (currentTime - activationTimes[i] >= channelDuration)) {
        // Deactivate the individual channel after 280ms
        singleActuator(i, qwiicMuxAddress);
        HMD[i].RTP(0);  // Stop the individual channel
        disableActuator(i, qwiicMuxAddress);
        HMD[i].stop();
        Serial.print("Deactivating individual channel [");
        Serial.print(i);
        Serial.println("] at time: " + String(currentTime) + "ms");

        // Mark the individual channel as deactivated
        deactivated[i] = true;
      }
    }

    // Check if all individual channels have been activated and deactivated
    if (currentChannel == 4 && allDeactivated()) {
      delay(100);  // 100 ms delay after the last individual channel is deactivated
      currentChannel = 0;  // Reset for the next sequence
      Serial.println("100ms delay complete. Ready for next Phase 2 repetition.");
      
      // Increment Phase 2 count
      phase2Count++;
      
      // Check if Phase 2 should complete after 3 repetitions
      if (phase2Count == maxRepeats) {
        isPhase2Complete = true;
      }

      // Reset deactivation states for the next sequence
      for (int i = 0; i < 4; i++) {
        deactivated[i] = false;
      }

      lastActivationTime = millis();  // Reset timer for the next cycle
    }
  }
}


// Helper function to check if all channels are deactivated
bool allDeactivated() {
  for (int i = 0; i < 4; i++) {
    if (!deactivated[i]) {
      return false;
    }
  }
  return true;
}






