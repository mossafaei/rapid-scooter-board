#include <MQTT.h>
#include <MQTTClient.h>

#include <timer.h>
#include <timerManager.h>

#include <TinyGPS++.h>
#include "M365.h"
#include <Arduino.h>
#include <Stream.h>
#include "wiring_private.h"
#include <MKRGSM.h>
#include "arduino_secrets.h"
#define SerialScooter Serial1

String scooterID = SCOOTER_ID;
String scooterToken = SECRET_TOKEN;
String UserNumber = "1";
String scooterLongtiude = "0", scooterLatitude = "0";

unsigned long ServerDelay = 4;
int ServerSwitch = 0;
M365 scooter;
command_t command;
stats_t stats;
Timer tim, sendDataToServerTimer;

TinyGPSPlus gps;

GSMClient net;
MQTTClient client;
GPRS gprs;
GSM gsmAccess;
char server[] = "YOUR_IP_ADDRESS";
String content[] = {"id=0&","longtiude=0&","latitude=0&","battery=0&","lock=0&","velocity=0"};

Uart gpsSerial (&sercom3, 0, 1, SERCOM_RX_PAD_1, UART_TX_PAD_0);

unsigned int turn = 0;
////////////////////////////////////

//This function runs when the board get a new message form server
void messageReceived(String &topic, String &payload){

    Serial.println("incoming: " + topic + " - " + payload);
    String buff = "";

    if (payload && payload.length() > 0){
        for (int i=0 ; i < payload.length() ; i++){
            if (payload[i] != '=')
                buff += payload[i];
            else break;
        }
        if (buff == "lock"){
           if (payload[5] == '1')command.lock = 1;
           else if (payload[5] == '0')command.lock = 0;
        }
        if (buff == "night"){
           if (payload[6] == '1')command.tail=1;
           else if (payload[6] == '0')command.tail=0;
        }
    }
    scooter.setCommand(&command);
    scooter.setScooterLock();
    scooter.setScooterTail();

}

//Prepare for connecting to server
void setupClient(){

    client.begin(server,net);
    client.onMessage(messageReceived);
    bool connected = false;
    while (!connected){
      if ((gsmAccess.begin() == GSM_READY) && (gprs.attachGPRS("mtnirancell","","") == GPRS_READY)){ //for MTN IranCell use mtnirancell
          connected = true;
          Serial.println("GSM CONNECTED");
      }
      else{
        delay(1000);
        Serial.println("Faild");
      }
    }
    Serial.print("\nconnecting...");
      while (!client.connect("A1111", "A1111", "abcd")) {
        Serial.print(".");
        delay(1000);
     }

    Serial.println("\nconnected!");
    client.subscribe("/scooter/admin/A1111");

}

//Convert Uint8 to String
String convertUint8ToString(uint8_t str){return String(str);}

//Constructing response message for sending to server
void constructPathString(String id, String longtiude, String latitude, uint8_t battery, uint8_t lock, uint8_t velocity){

    content[0] = "id=" + id + "&";
    content[1] = "longtiude=" + longtiude + "&";
    content[2] = "latitude=" + latitude + "&";
    content[3] = "battery=" + convertUint8ToString(battery) + "&";
    content[4] = "lock=" + convertUint8ToString(lock) + "&";
    content[5] = "velocity=" + convertUint8ToString(velocity);

}


String getNumber(String s){

  String ans = "";
  for (int i=2 ; i < s.length() ; i++){
      if (s[i] == '"')return ans;
      ans += s[i];
  }

}

//Send data to server
void sendDataToServer(){

  constructPathString(scooterID, scooterLongtiude, scooterLatitude, stats.battery, stats.lock, stats.velocity);
  client.publish("/scooter/A1111", content[0] + content[1] + content[2] + content[3] + content[4] + content[5] );
  
  scooter.setScooterLock();
  scooter.setScooterTail();

}

///////////////////////////////////

//Send data to scooter's motor
void SendDataToScooter(){
  scooter.send();
  scooter.process();
  scooter.getStats(&stats);
}

//To prevent scooter from sleep
/*void keepturnon(){
    if (UserNumber != "0"){
        if (stats.eco == 1 || stats.eco == 3){
            turn=0;
            return;
        }
        if(turn >= 4*60){
            Serial.println("SET");
            //cloudCommand("lock");
            scooter.setScooterLock();
            turn = 0;
        }
        if (stats.lock == 2 || stats.lock == 6)scooter.setScooterUnLock();
    }else{
      
        if(turn >= 4*60){
            Serial.println("SET");
            //cloudCommand("unlock");
            scooter.setScooterUnLock();
            //scooter.setScooterLock();
            turn = 0;
        }
        if (stats.lock == 0)scooter.setScooterLock();
    }
    
}*/

//Read GPS data from module
void readGPSData(){
    while (gpsSerial.available() > 0){
      if (gps.encode(gpsSerial.read()) && gps.location.isValid()){
          scooterLongtiude = String(gps.location.lng(),6);
          scooterLatitude = String(gps.location.lat(),6);
      }
    }
}

void setup(){

    Serial.begin(9600);
    gpsSerial.begin(9600);
    pinPeripheral(0, PIO_SERCOM); //GPS: Assign TX function to pin 0
    pinPeripheral(1, PIO_SERCOM); //GPS: Assign RX function to pin 1

    setupClient();
    
    tim.setInterval(20);
    tim.setCallback(SendDataToScooter);

    sendDataToServerTimer.setInterval(3000);
    sendDataToServerTimer.setCallback(sendDataToServer);
    
    scooter.setup(SerialScooter, A5, A4, 2, 5, 4, 3, 6, 7, ServerSwitch, tim);

    sendDataToServerTimer.start();
    
}

void loop(){

    scooter.updateTimer();
    client.loop();
    if (!client.connected())
      setupClient();
    //sendDataToServer();
    sendDataToServerTimer.update();
    readGPSData();
    static unsigned long loopDelay = 0;
    if(millis() - loopDelay > 1000){
        loopDelay = millis();
    }

}

//Setup SERCOM3 for gpsSerial
void SERCOM3_Handler(){
  gpsSerial.IrqHandler();
}

int cloudCommand(String &topic, String &userCommand){
    int confirmation = -1;
    if(userCommand.equals("unlock")){
        command.lock = 0;
        confirmation = 0;
    }
    else if(userCommand.equals("lock")){
        command.lock = 1;
        confirmation = 1;
    }
    else if(userCommand.equals("tailoff")){
        command.tail = 0;
        confirmation = 2;
    }
    else if(userCommand.equals("tailon")){
        command.tail = 1;
        confirmation = 3;
    }
    else if(userCommand.equals("cruiseoff")){
        command.cruise = 0;
        confirmation = 4;
    }
    else if(userCommand.equals("cruiseon")){
        command.cruise = 1;
        confirmation = 5;
    }
    else if(userCommand.equals("ecolow")){
        command.ecoMode = 0;
        confirmation = 6;
    }
    else if(userCommand.equals("ecomed")){
        command.ecoMode = 1;
        confirmation = 7;
    }
    else if(userCommand.equals("ecohigh")){
        command.ecoMode = 2;
        confirmation = 8;
    }
    else if(userCommand.equals("ecooff")){
        command.eco = 0;
        confirmation = 9;
    }
    else if(userCommand.equals("ecoon")){
        command.eco = 1;
        confirmation = 10;
    }
    else if(userCommand.equals("nightoff")){
        command.night = 0;
        confirmation = 11;
    }
    else if(userCommand.equals("nighton")){
        command.night = 1;
        confirmation = 12;
    }
    else if(userCommand.equals("poweroff")){
        command.lock = 0;
        command.power = 0;
        confirmation = 13;
    }
    else if(userCommand.equals("poweron")){
        command.power = 1;
        confirmation = 14;
    }
    else if(userCommand.equals("reset")){
        //System.reset();We Should Reset THE SYSTEM HERE!!!!!
        confirmation = 15;
    }
    else if(userCommand.equals("headoff")){
        command.head = 0;
        confirmation = 16;
    }
    else if(userCommand.equals("headon")){
        command.head = 1;
        confirmation = 17;
    }
    else if(userCommand.equals("ledoff")){
        command.led = 0;
        confirmation = 18;
    }
    else if(userCommand.equals("ledon")){
        command.led = 1;
        confirmation = 19;
    }
    else if(userCommand.equals("alarmoff")){
        command.alarm = 0;
        confirmation = 20;
    }
    else if(userCommand.equals("alarmon")){
        command.alarm = 1;
        confirmation = 21;
    }
    else if(userCommand.equals("alarm")){
        tone(2, 20, 5000);
        confirmation = 22;
    }
    scooter.setCommand(&command);
    return confirmation;
}
