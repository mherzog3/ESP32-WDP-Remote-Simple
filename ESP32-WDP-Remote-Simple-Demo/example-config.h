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

//WiFi Parameter

const char* ssid = "WLAN-SSID"; //WLAN-Name
const char* password = "WLAN-Password";

//WDP-IP
const char* host = "192.168.12.108";
const int port=15209;

//rotary encoder implementation based on examples from https://github.com/igorantolic/ai-esp32-rotary-encoder/
/*
connecting Rotary encoder KY-040

Rotary encoder side    MICROCONTROLLER side  
-------------------    ---------------------------------------------------------------------
CLK (A pin)            any microcontroler intput pin with interrupt -> in this example pin 32
DT (B pin)             any microcontroler intput pin with interrupt -> in this example pin 33
SW (button pin)        any microcontroler intput pin with interrupt -> in this example pin 25
GND - to microcontroler GND
VCC                    microcontroler VCC (then set ROTARY_ENCODER_VCC_PIN -1) 

***OR in case VCC pin is not free you can cheat and connect:***
VCC                    any microcontroler output pin - but set also ROTARY_ENCODER_VCC_PIN 25 
                        in this example pin 25

*/

#define ROTARY_ENCODER_A_PIN 32 //if Rotation dir is false, Swap A and B
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */

//depending on your encoder - try 1,2 or 4 to get expected behaviour
#define ROTARY_ENCODER_STEPS 4

#define ROTARY_ENCODER_ACCELERATION 50 //when using more than ROTARY_ENCODER_ACCEL_START speed steps use the acceleration function of the rotary encoder driver
                                       //this means when turn the knob fast some speed steps will be skipped, if the effect is to strong
                                       //decrease the number, otherwise increase
#define ROTARY_ENCODER_ACCEL_START 32  //Number of speed steps from which the acceleration is used 

#define statusAlive 14 //LED for AliveHeartBeat-Blinking, 0 if unused

#define maxGPIO 40

//Array (size has to be equal to maxGPIO) for assigning Function to buttons attached to GPIO Pins, Pins are automatically pulled up, Push-Button between GPIO and GND
int buttConfig[]={
    1, 0, 0, 0, 1000, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    4, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0};
    //0 unused
    //1000...1031 F0-F31
    //1 Stop/Go Toggle
    //2 Stop
    //3 Go
    //4 LocoSpeed 0
    //5 LocoDir Toggle
    //6 LocoDir Backwards
    //7 LocoDir Forwards
    //8 Loco Release

//Array (size has to be equal to maxGPIO) for assigning Status LED to GPIO Pins
int ledConfig[]={
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 103, 101, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0};
    //0 unused
    //1000...1031 F0-F31 State
    //2 State Stop
    //3 State Go
    //4 LocoSpeed=0
    //6 LocoDir=Backwards
    //7 LocoDir=Forwards
    //101 WLAN Connnected
    //102 WDP Connected
    //103 WDP Connected and Blink while State Stop, continous while Go

