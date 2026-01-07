/*ESP32-WDP-Remote-Simple
Copyright (C) 2024 Markus Herzog

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

#include <WiFiClient.h>
#include <WiFi.h>

// Include the AiEsp32RotaryEncoder library found here or via Arduino IDE Library Manager :
// https://github.com/igorantolic/ai-esp32-rotary-encoder
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "config.h"
#include <EEPROM.h>

// Include the Bounce2 library found here or via Arduino IDE Library Manager :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 4

#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000

unsigned long lTick;
unsigned long lTick2;
int PauseHeartBeat = 0;
byte TickHeartBeat = 0;
boolean up = true;
byte HeartBeat[] = {0xFF,0xFD,0xFB,0xF7,0xF3,0xEB,0xDF,0xD3,0xC7,0xBB,0xAF,0xA3,0x97,0x8B,0x7F,0x73,0x67,0x5B,0x4F,0x43,0x37,0x2B,0x1F,0x13,0x0,0xF,0x2D,0x4B,0x69,0x87,0xA5,0xC3,0xD7,0xE6,0xE6,0xD7,0xC3,0xA5,0x87,0x69,0x4B,0x2D,0xF,0x0,0x19,0x32,0x4B,0x64,0x7D,0x96,0xAF,0xC8,0xE1,0xEB,0xEF,0xF7,0xFB,0xFD,0xFF};

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

Bounce2::Button * buttons = new Bounce2::Button[maxGPIO];

float versionF = 1.0;
String sendi = "";
int tick = 0;

struct lokData {
  long id;
  String br;
  int dnr;
};

int lokCount = 0;
int curLokIndex = -1;
int curLokDNr = 0;
long curLokID = 0;
lokData loks[1000];
bool lokDir = true;

double measPF[29];
double measPB[29];

int sollG;
int istG;
double sollV;

bool fx[32];
bool fxAvail[32];
String fxName[32];

int steps[2][150];
int stepsCount[2];

int SollGSend[256];
int SollGIndex;
int sysState = 0;

bool goStop;



WiFiClient client;

long EEPROMReadlong(long address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void EEPROMWritelong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}


void sendF(int F);

void sendDir(bool newDir){
  if (curLokID==0) return;
  lokDir=newDir;
  client.print(String("[Command LokDirection " + String(curLokID) + ";" + (lokDir ? "f" : "r") + "]"));
}

void rotary_onButtonClick() //turn loco on rotary button click
{
  static unsigned long lastTimePressed = 0;
  if (millis() - lastTimePressed < 500)
    return; //ignore multiple press in that time milliseconds
  lastTimePressed = millis();

  sendDir(!lokDir);
  
}

void rotary_loop() //send spend on rotary turn
{
  if (rotaryEncoder.encoderChanged())
  {
    int i = rotaryEncoder.readEncoder();
    Serial.print("Value: ");
    Serial.println(rotaryEncoder.readEncoder());
    if ((i >= 0) && (i <= stepsCount[(lokDir ? 0 : 1)])) {
      sendF(i);
    }
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

void setEncoderBoundaries() {  //set encoder boundaries according to loco speed steps
  bool circleValues = false;
  rotaryEncoder.setBoundaries(0, stepsCount[(lokDir ? 0 : 1)], circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  if (rotaryEncoder.readEncoder() > stepsCount[(lokDir ? 0 : 1)]) rotaryEncoder.setEncoderValue(stepsCount[(lokDir ? 0 : 1)]);

  if (stepsCount[(lokDir ? 0 : 1)]>=32){
     rotaryEncoder.setAcceleration(ROTARY_ENCODER_ACCEL_START); //for many speed Steps turn on acceleration
  } else {
      rotaryEncoder.disableAcceleration();
  }
  
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  EEPROM.begin(EEPROM_SIZE);
  curLokID=EEPROMReadlong(0);

  Serial.printf("CurLokID %d \n", curLokID);

  for (int i = 0; i < maxGPIO; i++) {
    if (buttConfig[i] != 0) {
      buttons[i].attach(i, INPUT_PULLUP  );       //setup the bounce instance for the current button
      buttons[i].interval(25);              // interval in ms
      buttons[i].setPressedState(LOW);
    }
    if (ledConfig[i] != 0) {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }

  if (statusAlive != 0) {
    //ledcAttachPin(statusAlive, PWM1_Ch);
    //ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
    ledcAttach(statusAlive, PWM1_Freq, PWM1_Res);
    ledcWrite(statusAlive, 128);
  }


  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  setEncoderBoundaries();
 

  tick = 0;


  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
  sysState = 2;
}



//Calc km/h to internal 1000 speed steps
double SpeedtoG(double Speed) {
  int FS28;
  double A;
  double s;

  if (Speed <= 0) {
    return 0;
  }
  s = Speed;
  FS28 = 0;
  if (lokDir == true) {
    if (s >= measPF[28]) {
      return 1000;
    }
    for (int i = 1; i < 29; i++) {
      if ( (measPF[i - 1] < s) && (s <= measPF[i]) ) {
        FS28 = i - 1;
        break;
      }
    }
    A = measPF[FS28 + 1] - measPF[FS28];
    return (((s - measPF[FS28]) / A) + FS28) * 1000 / 28;
  } else {
    if (s >= measPB[28]) {
      return 1000;
    }
    for (int i = 1; i < 29; i++) {
      if ( (measPB[i - 1] < s) && (s <= measPB[i]) ) {
        FS28 = i - 1;
        break;
      }
    }
    A = measPB[FS28 + 1] - measPB[FS28];
    return (((s - measPB[FS28]) / A) + FS28) * 1000 / 28;

  }
}

//calc internal 1000 speed steps to km/h
double GtoSpeed(int G) {
  double FS28;
  double A;

  FS28 = ((double)G) * 28.0 / 1000.0;
  if (FS28 < 0) {
    FS28 = 0;
  }
  if ((int)FS28 < 28) {
    if (lokDir == true) {
      A = measPF[(int)FS28 + 1] - measPF[(int)FS28];
      return measPF[(int)FS28] + A * (FS28 - (double)((int)FS28));
    } else {
      A = measPB[(int)FS28 + 1] - measPB[(int)FS28];
      return measPB[(int)FS28] + A * (FS28 - (double)((int)FS28));
    }

  } else {
    if (lokDir == true) {
      return measPF[28];
    } else {
      return measPB[28];
    }
  }
}

//calc internal 1000 speed steps to decoder speed step
int GtoRealF(int G) {
  int index = 0;
  int minS = 0;
  if (!lokDir) index = 1;

  for (int i = 0; i <= stepsCount[index]; i++) {
    if  ((G >= minS) && (G <= steps[index][i])) {
      return i;
    }
  }
  return -1;
}

//store last sent speed values
void AddSollGSend(int speed) {
  SollGSend[SollGIndex] = speed;
  SollGIndex++;
  SollGIndex &= 0xff;
}

//clear list of last sent speed values
void ClearSollGSend() {
  for (int i = 0; i <= 255; i++) {
    SollGSend[i] = -1;
    SollGIndex = 0;
  }
}

//check if received speed was sent by myself
boolean speedDifference(int speed) {
  for (int i = 0; i <= 255; i++) {
    if (SollGSend[(SollGIndex + i) & 0xff] == speed) {
      for (int j = 0; j <= i; j++) {
        SollGSend[(SollGIndex + j) & 0xff] = -1;
      }
      return false;
    }
  }
  ClearSollGSend();
  return true;
}

//send speed steop
void sendF(int F) {
  int G;
  if (curLokID == 0) return;
  if (F == 0) {
    G = 0;
  } else if (F == stepsCount[(lokDir ? 0 : 1)]) {
    G = steps[(lokDir ? 0 : 1)][F];
  } else {
    if (F > stepsCount[(lokDir ? 0 : 1)]) return;
    G = (steps[(lokDir ? 0 : 1)][F] + steps[(lokDir ? 0 : 1)][F - 1]) >> 1;
  }
  sollG = G;
  sollV = GtoSpeed(G);
  client.print(String("[Command LokSpeed " + String(curLokID) + ";" + String(G) + "]"));
  AddSollGSend(G);
}


void connectWDP() {
  Serial.printf("\n[Connecting to %s ... ", host);
  if (client.connect(host, port))
  {
    Serial.println("connected]");

    client.print(String("[Command NoUTF8]"));
    client.print(String("[Command AllowLokAssign]"));
    client.print(String("[Info OS Arduino]"));
    client.print(String("[Info Version " + String(versionF, 1) + "]") );
    sendi = String("[Info DeviceID " + WiFi.macAddress() + "-" +  WiFi.macAddress() + "]");
    sendi.replace(":", "");
    client.print(sendi);
    client.print(String("[Command SysStatus]"));
    client.print(String("[Command LokParams 1;1]"));
    client.print(String("[Command LokList]"));
    sysState = 3;
  } else {
    Serial.println("connection failed!]");
    client.stop();
  }
}


void updateLED() {
  
  for (int i = 0; i < maxGPIO; i++) {
    if (ledConfig[i] != 0) {

        if (ledConfig[i] == 2) {
          digitalWrite(i, (goStop ? LOW : HIGH));
        } else if (ledConfig[i] == 3) {
          digitalWrite(i, (goStop ? HIGH : LOW));
        } else if (ledConfig[i] == 4) {
          digitalWrite(i, ((sollG==0) ? HIGH : LOW));
        } else if (ledConfig[i] == 6) {
          digitalWrite(i, (lokDir ? LOW : HIGH));
        } else if (ledConfig[i] == 7) {
          digitalWrite(i, (lokDir ? HIGH : LOW));
        } else if (ledConfig[i] == 101) {
          digitalWrite(i, ((WiFi.waitForConnectResult() != WL_CONNECTED) ? LOW : HIGH));
        } else if (ledConfig[i] == 102) {
          digitalWrite(i, (client.connected() ? HIGH : LOW));
        } else if (ledConfig[i] == 103) {
          if (!(client.connected())){
            digitalWrite(i,LOW);
          } else if (goStop){
             digitalWrite(i,HIGH); 
          } else {
            digitalWrite(i, (tick<=500) ? LOW : HIGH);
          }
        } else if ((ledConfig[i] >= 1000) && (ledConfig[i] <= 1031)) {
          if (fxAvail[ledConfig[i] - 1000]) {
            digitalWrite(i, fx[ledConfig[i]-1000] ? HIGH : LOW);
          }
        }

    }
  }

}
void processIncoming() {
  //Serial.println("Start-Read");
  String line = client.readStringUntil('\n');
  //Serial.println("Read");
  //Serial.println(line);
  int startC = line.indexOf('[');
  int endC = line.indexOf(']');
  int startC2;
  int startC3;
  int endC2;
  bool extFX;
  int endC3;
  int index;
  int index2;
  int index3;
  int funcNo;
  int decoderNo;
  int iconNo;
  int sollGTemp;
  bool fxChange=false;
  bool newFX;
  if ((startC >= 0) && (endC > startC)) {
    line = line.substring(startC + 1, endC);
    endC = line.indexOf(' ');
    if ((endC > 0) && (endC < line.length() - 2)) {
      String cmdType = line.substring(0, endC);
      line = line.substring(endC + 1);
      endC = line.indexOf(' ');

      if (((endC > 0) && (endC <= line.length() - 2)) || (endC == -1)) {
        String cmd;
        if (endC == -1) {
          cmd = line.substring(0);
          line = "";
        } else {
          cmd = line.substring(0, endC);
          line = line.substring(endC + 1);
        }

      

        if (cmd.equals("LokState")) {
          index = 0;
          startC = 0;
          fxChange=false;
          endC = line.indexOf(';', startC);
          while (endC > 0) {
            if (endC < 0) endC = line.length();
            if (index == 0) {
              if (line.substring(startC, endC).toInt() != curLokID) {
                break;
              }
            } else if (index == 1) {
              lokDir = line.substring(startC, endC).equals("f");
            } else if (index == 2) {
              sollGTemp = line.substring(startC, endC).toInt();
            } else if (index == 3) {
              istG = line.substring(startC, endC).toInt();
            } else if (index == 4) {
              if (speedDifference(sollGTemp)) {  //avoid loops
                sollG = sollGTemp;
                sollV = line.substring(startC, endC).toFloat();
                if (GtoRealF(sollG) != rotaryEncoder.readEncoder()) {
                  rotaryEncoder.setEncoderValue(GtoRealF(sollG));
                  Serial.println("new SollF " + String(GtoRealF(sollG)));
                }
                //Serial.println(String(GtoSpeed(sollG))+" "+String(GtoSpeed(istG))+" "+String(sollV)+" "+String(GtoRealF(istG)));
              }


            } else if (index <= 5 + 31) {

              if (index==5) extFX=line.substring(startC, endC).equals("ext"); //new LokState Format of WDP 2025?

              if (!extFX){ //old Lokstate format until WDP 2021
                newFX= line.substring(startC, endC).equals("1");
                if (fx[index - 5]!=newFX){
                  fx[index - 5]=newFX;
                  fxChange=true;
                }
              } else {
                if (index==6){
                  newFX= line.substring(startC, endC).equals("1");
                  if (fx[0]!=newFX){
                    fx[0]=newFX;
                    Serial.printf("Fx %d = %d\n", 0,newFX ? 1 : 0);
                    fxChange=true;
                  }
                } else if (index==7){ // | seperated list of fx; each element: funcNo§state
                    String fxStr = line.substring(startC, endC);
                    String fxSubStr;
                    index2 = 0;
                    startC2 = 0;
                    endC2 = fxStr.indexOf('|', startC2);
                    if ((endC2 < 0) && (startC2 < fxStr.length()) && (3 <= fxStr.length()) ) endC2 = fxStr.length();
                    while (endC2 > 0) {
                      String fxSubStr = fxStr.substring(startC2, endC2);    
                      index3 = 0;            
                      startC3 = 0;
                      endC3 = fxSubStr.indexOf('§', startC3);
                      if ((endC3 < 0) && (startC3 < fxSubStr.length())) endC3 = fxSubStr.length();
                      while (endC3 > 0) {         

                        if (index3==0) {
                          funcNo=fxSubStr.substring(startC3, endC3).toInt()+1;
                        } else if (index3==1) {
                          if ((funcNo>0)  && (funcNo<=31)){
                            newFX= fxSubStr.substring(startC3, endC3).equals("1");
                            if (fx[funcNo]!=newFX){
                              fx[funcNo]=newFX;
                              Serial.printf("Fx %d = %d\n", funcNo,newFX ? 1 : 0);
                              fxChange=true;
                            }
                          }
                        }

                        startC3 = endC3 + 1;
                        endC3 = fxSubStr.indexOf('§', startC3);
                        if ((endC3 < 0) && (startC3 < fxSubStr.length())) endC3 = fxSubStr.length();
                        index3++;
                      }

                      startC2 = endC2 + 1;
                      endC2 = fxStr.indexOf('|', startC2);
                      if ((endC2 < 0) && (startC2 < fxStr.length())) endC2 = fxStr.length();
                      index2++;
                    }
                }
              }
              
            } else {
              break;
            }
            index++;
            startC = endC + 1;
            endC = line.indexOf(';', startC);
            if ((endC < 0) && (startC < line.length())) endC = line.length();
          }
          setEncoderBoundaries();
          if (fxChange) updateLED();
        } else if (cmd.equals("Go")) {
          goStop = true;
        } else if (cmd.equals("Stop")) {
          goStop = false;
        } else if (cmd.equals("LokInfo")) {
          for (index3=0;index3<32;index3++){
            fxAvail[index3]=false;
            fx[index3]=false;
            fxName[index3]="";
          }
          index = 0;
          startC = 0;
          endC = line.indexOf(';', startC);
          while (endC > 0) {

            if (index == 0) {
              if (line.substring(startC, endC).toInt() != curLokID) {
                break;
              }
            } else if (index <= 58) { //km/h calibration curve forward and backwar
              if ((index & 1) == 1) {
                measPF[(index - 1) >> 1] = line.substring(startC, endC).toFloat() * 0.001;
              } else {
                measPB[(index - 1) >> 1] = line.substring(startC, endC).toFloat() * 0.001;
              }
            }
            if (index==59) extFX=line.substring(startC, endC).equals("ext"); //new Format of FX List for WDP 2025

            if (!extFX){ //old format until WDP 2021

              if (index <= 59 + 16) { //F0-F16 at 59-75 check if Fx available for decoder
                fxAvail[index - 59] = (line.substring(startC, endC).toInt() > 0);
              } else if (index <= 59 + 16 + 17) { //FD F0-F16 at 76-92 FD decoder not supported yet
              } else if (index <= 93 + 11) { //F17-F28 at 93-104 check if Fx available for decoder
                fxAvail[index - 93 + 17] = (line.substring(startC, endC).toInt() > 0);
              } else if (index <= 93 + 11 + 12) { //FD F17-F28 at 105-116 FD decoder not supported yet
              } else if (index <= 117 + 2) { //F29-F31 at 117-119 check if Fx available for decoder
                fxAvail[index - 117 + 29] = (line.substring(startC, endC).toInt() > 0);
              } else if (index <= 117 + 2 + 3) { //FD F29-F31 at 120-122 FD decoder not supported yet
              }
            } else {
              
              if (index==60){
                  fxAvail[0] = (line.substring(startC, endC).toInt() > 0);
              } else if (index==63){
                  fxName[0] = line.substring(startC, endC);
              } else if (index==62){ // | seperated list of fx; each element: decoderNo§funcNo§iconNo§funcName
                  //Serial.println("fxStr");
                  String fxStr = line.substring(startC, endC);                                    
                  index2 = 0;
                  startC2 = 0;
                  
                  
                  endC2 = fxStr.indexOf('|', startC2);
                  if ((endC2 < 0) && (startC2 < fxStr.length()) && (6 <= fxStr.length()) ) endC2 = fxStr.length();
                  while (endC2 > 0) {
                    String fxSubStr = fxStr.substring(startC2, endC2);  
                    //Serial.println(fxSubStr);      
                    index3 = 0;   
                    startC3 = 0;
                    endC3 = fxSubStr.indexOf('§', startC3);
                     
                    while (endC3 > 0) {         
                      //Serial.printf("%d %d %d\n",index3,startC3,endC3);
                      if (index3==0) {
                        decoderNo=fxSubStr.substring(startC3, endC3).toInt();
                      } else if (index3==1) {
                        funcNo=fxSubStr.substring(startC3, endC3).toInt()+1;
                      } else if (index3==2) {
                        iconNo=fxSubStr.substring(startC3, endC3).toInt();
                        if ((funcNo>0)  && (funcNo<=31) && (decoderNo==0) && (iconNo!=-2)){ //iconNo = -2 empty place
                          fxAvail[funcNo]=(iconNo > 0);
                        }
                        //Serial.printf("Func Data %d %d %d\n",decoderNo,funcNo,iconNo);     
                      } else if (index3==3) {                          
                        if ((funcNo>0)  && (funcNo<=31) && (decoderNo==0)){
                          fxName[funcNo]=fxSubStr.substring(startC3, endC3);
                        }
                      }

                      startC3 = endC3 + 1;
                      endC3 = fxSubStr.indexOf('§', startC3);
                      if ((endC3 < 0) && (startC3 < fxSubStr.length())) endC3 = fxSubStr.length();
                      index3++;
                    }

                    startC2 = endC2 + 1;
                    endC2 = fxStr.indexOf('|', startC2);
                    if ((endC2 < 0) && (startC2 < fxStr.length())) endC2 = fxStr.length();
                    index2++;
                  }
              }
            }
            index++;
            startC = endC + 1;
            endC = line.indexOf(';', startC);
            if ((endC < 0) && (startC < line.length())) endC = line.length();
          }
          if (index >= 58) client.print(String("[Command LokSpeedSteps " + String(curLokID) + "]"));
        } else if (cmd.equals("LokAssign")) { 
          //WDP tells us which loco to use, we can you choose any other by ourself, LokAssign used for displayless devices
          index = line.toInt();
          Serial.printf("LokAssign %d\n", index);
          if (index>0){
            for (int i = 0; i < lokCount; i++) {
              if (loks[i].id == index) {

                if (curLokID != loks[i].id) {
                  curLokID = loks[i].id;
                  EEPROMWritelong(0,curLokID);
                  Serial.printf("CurLokID Save %d\n", curLokID);
                  EEPROM.commit();
                }
                
                curLokIndex = i;
                
                curLokDNr = loks[curLokIndex].dnr;

                ClearSollGSend();
                client.print(String("[Command LokInfo " + String(curLokID) + "]"));
                break;
              }
            }
          } else {
            releaseLoco();
          }
          
        } else if (cmd.equals("LokSpeedSteps")) { //translation table decoder speed step, internal speed step
          index = 0;
          startC = 0;
          endC = line.indexOf(';', startC);
          while (endC > 0) {
            if (index == 0) {
              if (line.substring(startC, endC).toInt() != curLokID) {
                break;
              }
            } else if (index <= 2) {
              String speedStepStr = line.substring(startC, endC);
              index2 = 0;
              startC2 = 0;
              endC2 = speedStepStr.indexOf('#', startC2);
              while (endC2 > 0) {

                if (index2 == 0) {
                  stepsCount[index - 1] = speedStepStr.substring(startC2, endC2).toInt();
                } else if (((index2 - 1) <= stepsCount[index - 1]) && ((index2 - 1) <= 149)) {
                  steps[index - 1][index2 - 1] = speedStepStr.substring(startC2, endC2).toInt();
                }

                startC2 = endC2 + 1;
                endC2 = speedStepStr.indexOf('#', startC2);
                if ((endC2 < 0) && (startC2 < speedStepStr.length())) endC2 = speedStepStr.length();
                index2++;
              }

            }
            index++;
            startC = endC + 1;
            endC = line.indexOf(';', startC);
            if ((endC < 0) && (startC < line.length())) endC = line.length();
          }
          setEncoderBoundaries();
          if (index >= 2) client.print(String("[Command LokState " + String(curLokID) + "]"));
        } else if (cmd.equals("Disconnect")) {
          client.stop();
          return;
        } else if (cmd.equals("LokList")) { //get list of locos available in WDP and store digital no (DNR), id and class (BR)
          String lDataSet;
          lokCount = 0;
          curLokIndex = -1;

          long id;
          String br;
          int dnr;
          while (line.indexOf(';') > 0) {
            endC = line.indexOf('|');
            if (endC < 0) endC = line.length();
            if (endC > 0) {
              lDataSet = line.substring(0, endC);
              line = line.substring(endC + 1);
              endC = lDataSet.indexOf(';');
              if (endC > 0) {
                id = lDataSet.substring(0, endC).toInt();
                endC2 = lDataSet.indexOf(';', endC + 1);
                if (endC2 > 0) {
                  br = lDataSet.substring(endC + 1, endC2);
                  endC = lDataSet.indexOf(';', endC2 + 1);
                  if (endC > 0) {
                    endC2 = lDataSet.indexOf(';', endC + 1);
                    if (endC2 > 0) {
                      dnr = lDataSet.substring(endC + 1, endC2).toInt();
                      lokCount++;
                      loks[lokCount - 1].dnr = dnr;
                      loks[lokCount - 1].br = br;
                      loks[lokCount - 1].id = id;
                      if ((curLokDNr == dnr) && (curLokID == id)) {
                        curLokIndex = lokCount - 1;
                      }
                    }
                  }
                }
              }
            } else {
              lDataSet = line;
              line = "";
            }
          }
          //Serial.println("Lokcount "+String(lokCount));

          for (int i = 0; i < lokCount; i++) {
            //Serial.println(String(loks[i].id)+ " " + loks[i].br + " " + String(loks[i].dnr));
          }
          if (lokCount > 0) {
            curLokIndex=-1;
            for (int i = 0; i < lokCount; i++) {
                if (curLokID == loks[i].id){
                  curLokIndex=i;
                  break;
                }
            }
            
            
            if (curLokIndex < 0) {
              curLokIndex = 0;
              curLokID = 0;
              EEPROMWritelong(0,curLokID);
              //Serial.printf("CurLokID Auto %d ", curLokID);
              EEPROM.commit();
            }
            
            if (curLokID>0){
              curLokDNr = loks[curLokIndex].dnr;
              ClearSollGSend();
            }
            client.print(String("[Command LokInfo " + String(curLokID) + "]"));

          }
        }

      }
    }
  }
  //Serial.println("End-Read");
}

void releaseLoco(){
  curLokIndex=0;
  curLokDNr=0;
  curLokID=0;
  EEPROMWritelong(0,curLokID);
  Serial.printf("CurLokID Save %d\n", curLokID);
  EEPROM.commit();
  client.print(String("[Command LokInfo " + String(curLokID) + "]"));
}

//evaluate button states
void checkButtons() {
  for (int i = 0; i < maxGPIO; i++) {
    if (buttConfig[i] != 0) {
      buttons[i].update();
      if (buttons[i].pressed()) {
        Serial.println("Button "+ String(i) + " pressed");

        if (buttConfig[i] == 1) {
          if (goStop) {
            client.print(String("[Command Stop]"));
          } else {
            client.print(String("[Command Go]"));
          }
        } else if (buttConfig[i] == 2) {
          client.print(String("[Command Stop]"));
        } else if (buttConfig[i] == 3) {
          client.print(String("[Command Go]"));
        } else if (buttConfig[i] == 4) {
          sendF(0);
        } else if (buttConfig[i] == 5) {
          sendDir(!lokDir);
        } else if (buttConfig[i] == 6) {
          sendDir(false);
        } else if (buttConfig[i] == 7) {
          sendDir(true);
        } else if (buttConfig[i] == 8) {
          releaseLoco();
        } else if ((buttConfig[i] >= 1000) && (buttConfig[i] <= 1031)) {
          if ((fxAvail[buttConfig[i] - 1000])&&(curLokID!=0)) {            
            client.print(String("[Command LokFunction " + String(curLokID) + ";" + String(buttConfig[i] - 1000) + ";" + (fx[buttConfig[i] - 1000] ? "0" : "1") + "]"));
          }
        }
      }

    }

  }
}

void loop()
{
  rotary_loop();
  checkButtons();


  if ((tick % 100) == 0) {
    updateLED();
  }
  int oldTick = tick;
  if (millis() - lTick2 >= 2) {
    lTick2 = millis();
    tick++;
    if (tick >= 1000) tick = 0;
  }


  //heartled flickering, cool effect....
  if ( oldTick != tick) {
    if (statusAlive != 0) {
      if (PauseHeartBeat <= 0) {
        if (millis() - lTick > 10) {
          if (TickHeartBeat == 56) {
            PauseHeartBeat = 560;
          } else {
            TickHeartBeat++;
            ledcWrite(statusAlive, 255-HeartBeat[TickHeartBeat]);
            lTick = millis();
          }

        } else if (millis() < lTick) { //wegen Überlauf
          lTick = millis();
        }
      } else {
        PauseHeartBeat--;
        if (PauseHeartBeat == 0) {
          up = false;
          TickHeartBeat = 0;
          TickHeartBeat++;
          ledcWrite(statusAlive, 255-HeartBeat[TickHeartBeat]);
          lTick = millis();
        }
      }
    }
  }




  if (sysState == 2) {
    updateLED();
    connectWDP();
  } else if (sysState == 3) {
    if (!(client.connected())) {
      sysState = 2;
      updateLED();
      client.stop();
      Serial.println("\n[Disconnected]");
    } else if (client.available()) {
      processIncoming();
    }

  }





}
