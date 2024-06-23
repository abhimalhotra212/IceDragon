/* ICE DRAGON DEPLOYMENT SYSTEM
 * Author:  Matthew Law
 * Contact: melaw@purdue.edu
 * Version: 16 April 2024
 */

// Heartbeat parameters
#define pinHeartbeat 4
#define cycleDuration 2000

// Deployment parameters
#define pinDeployCtrl1 3
#define pinDeployCtrl2 5
#define deployPolarity 1      // 1 = Positive polarity, -1 = negative polarity
#define deployDuration 5000

// Indicator parameters
#define pinIndicatePower 8
#define pinIndicateReady 9
#define pinIndicateFault 10

// Input parameters
#define pinInputOvr1A 14
#define pinInputOvr1B 15
#define pinInputOvr2A 16
#define pinInputOvr2B 17
#define pinInputPiFault 6
#define pinInputDeploy 2

void setup() {
  // Pin modes
  pinMode(pinHeartbeat, OUTPUT);

  pinMode(pinDeployCtrl1, OUTPUT);
  pinMode(pinDeployCtrl2, OUTPUT);

  pinMode(pinIndicatePower, OUTPUT);
  pinMode(pinIndicateReady, OUTPUT);
  pinMode(pinIndicateFault, OUTPUT);

  pinMode(pinInputOvr1A, INPUT);
  pinMode(pinInputOvr1B, INPUT);
  pinMode(pinInputOvr2A, INPUT);
  pinMode(pinInputOvr2B, INPUT);
  pinMode(pinInputPiFault, INPUT);
  pinMode(pinInputDeploy, INPUT);

  // Initial output states
  digitalWrite(pinHeartbeat, HIGH);

  digitalWrite(pinDeployCtrl1, LOW);
  digitalWrite(pinDeployCtrl2, LOW);

  digitalWrite(pinIndicatePower, HIGH);
  digitalWrite(pinIndicateReady, HIGH);
  digitalWrite(pinIndicateFault, LOW);
}

int cycleTime = 0;
bool deployed = false;

void loop() {
  heartbeat(cycleTime);
  updateFault();
  
  deployed = updateDeploySignal(deployed);

  if(checkOverride()) {
    overrideDeploy();
  } else if(deployed){
    deploy();
  }
  
  cycleTime = updateCycleTime(cycleTime);
}

int updateCycleTime(int curCycleTime) {
  delay(1);
  
  if(curCycleTime < (cycleDuration - 1)) {
    curCycleTime++;
  } else {
    curCycleTime = 0;
  }

  return curCycleTime;
}

void heartbeat(int curCycleTime) {
  if(curCycleTime < (cycleDuration / 2)) {
    digitalWrite(pinHeartbeat, LOW);
  } else {
    digitalWrite(pinHeartbeat, HIGH);
  }
}

void deploy() {
  if(deployPolarity > 0) {
    digitalWrite(pinDeployCtrl2, LOW);
  } else {
    digitalWrite(pinDeployCtrl2, HIGH);
  }
  
  digitalWrite(pinDeployCtrl1, HIGH);
}

bool updateDeploySignal(bool curDeployed) {
  int deploySignal = true;
  
  if(curDeployed) {
    return true;
  } else {
    deploySignal = digitalRead(pinInputDeploy);
    return(!deploySignal);
  }
}

bool checkOverride() {
  int overrides = 0;
  overrides += (int) !digitalRead(pinInputOvr1A);
  overrides += (int) !digitalRead(pinInputOvr1B);
  return(overrides > 0);
}

void overrideDeploy() {
  bool ovr1A = !digitalRead(pinInputOvr1A);
  bool ovr1B = !digitalRead(pinInputOvr1B);
  bool ovr2A = !digitalRead(pinInputOvr2A);
  bool ovr2B = !digitalRead(pinInputOvr2B);

  if(ovr1A) {
    if(ovr2A) {
      digitalWrite(pinDeployCtrl1, HIGH);
      digitalWrite(pinDeployCtrl2, HIGH);
    } else if(ovr2B) {
      digitalWrite(pinDeployCtrl1, HIGH);
      digitalWrite(pinDeployCtrl2, LOW);
    } else {
      digitalWrite(pinDeployCtrl1, LOW);
      digitalWrite(pinDeployCtrl2, LOW);
    }
  } else {
    digitalWrite(pinDeployCtrl1, LOW);
    digitalWrite(pinDeployCtrl2, LOW);
  }
}

void updateFault() {
  bool piFault = digitalRead(pinInputPiFault);

  if(piFault) {
    digitalWrite(pinIndicateFault, HIGH);
  } else {
    digitalWrite(pinIndicateFault, LOW);
  }
}
