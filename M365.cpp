#include "M365.h"
M365::M365(){
}
M365::~M365(){ delete messageTimer; }

void M365::updateTimer(){
    messageTimer->update(); 
}
void M365::setup(Uart & USARTPort, uint8_t brake, uint8_t throttle, uint8_t buzzer, 
        uint8_t rled, uint8_t gled, uint8_t bled, uint8_t headlight, uint8_t power,int & serverinde,Timer & tm){
    ServerIndex = &serverinde;
    messageTimer = &tm;
    setSerial(USARTPort);
    setBrake(brake);
    setThrottle(throttle);
    setBuzzer(buzzer);
    setRGB(rled, gled, bled);
    setHeadlight(headlight);
    setPower(power);
}
void M365::setSerial(Uart & USARTPort){
    serial = &USARTPort;
    serial->begin(115200);
    messageTimer->start();
}
void M365::setBrake(uint8_t brake){
    input.brake = setAnalogPin(brake);
}
void M365::setThrottle(uint8_t throttle){
    input.throttle = setAnalogPin(throttle);
}
void M365::setBuzzer(uint8_t buzzer){
    input.buzzer = setDigitalPin(buzzer);
}
void M365::setHeadlight(uint8_t headlight){
    input.headlight = setDigitalPin(headlight);
}
void M365::setRGB(uint8_t r, uint8_t g, uint8_t b){
    input.rled = setDigitalPin(r);
    input.gled = setDigitalPin(g);
    input.bled = setDigitalPin(b);
}
void M365::setPower(uint8_t power){
    input.power = setDigitalPin(power);
}

void M365::process(){
    while(serial->available()){
        read(serial->read());
    }
}
void M365::setCommand(const command_t * newCommands){
    command = *newCommands;
}
void M365::getStats(stats_t * oldStats){
    *oldStats = stats;
}

void M365::send(){

    static uint8_t messageType = 0;
    uint8_t brake = getBrake();
    uint8_t speed = getThrottle();

    if(!isConnected()){
      messageType = 4;
    }

    switch(messageType++){
        case 0:
        case 1:
        case 2:
        case 3:{
            uint8_t message[] = {0x55, 0xAA, 0x7, 0x20, 0x65, 0x0, 0x4, speed, brake, 0x0, stats.beep, 0x0, 0x0};
            addSum(message, sizeof(message));
            transmit(message, sizeof(message));
            if(stats.beep) stats.beep = 0;
            break;
        }
        case 4:{
            uint8_t message[] = {0x55, 0xAA, 0x9, 0x20, 0x64, 0x0, 0x6, speed, brake, 0x0, stats.beep, 0x72, 0x0, 0x0, 0x0};
            addSum(message, sizeof(message));
            transmit(message, sizeof(message), true);
            if(stats.beep) stats.beep = 0;
            break;
        }
        case 5:{
            uint8_t message[] = {0x55, 0xAA, 0x6, 0x20, 0x61, 0xB0, 0x20, 0x02, speed, brake, 0x0, 0x0};
            addSum(message, sizeof(message));
            transmit(message, sizeof(message));
            break;
        }
        case 6:{
            uint8_t message[] = {0x55, 0xAA, 0x6, 0x20, 0x61, 0x7B, 0x4, 0x2, speed, brake, 0x0, 0x0};
            addSum(message, sizeof(message));
            transmit(message, sizeof(message));
            break;
        }
        case 7:{
            uint8_t message[] = {0x55, 0xAA, 0x6, 0x20, 0x61, 0x7D, 0x2, 0x2, speed, brake, 0x0, 0x0};
            addSum(message, sizeof(message));
            transmit(message, sizeof(message));
            messageType = 0;
    		break;
        }
    }
    
}
void M365::addSum(uint8_t * message, uint8_t size){
    unsigned long cksm = 0;
    for(int i = 2; i < size - 2; i++) cksm += message[i];
    cksm ^= 0xFFFF;
    message[size - 2] = (uint8_t)(cksm&0xFF);
    message[size - 1] = (uint8_t)((cksm&0xFF00) >> 8);
}
void M365::transmit(const uint8_t * message, uint8_t size, bool override){
    if(isConnected() || override){
        serial->write(message, size);
        serial->flush();
    }
}

void M365::read(uint8_t current_byte){
    serialBuffer[serialIndex] = current_byte;
        
    if(serialIndex && serialBuffer[serialIndex - 1] == 0x55 && serialBuffer[serialIndex] == 0xAA){
        if(check(serialBuffer, serialIndex)) translate(serialBuffer, serialIndex - 3);
        serialIndex = 0;
    }
    else serialIndex > 255 ? serialIndex = 0 : serialIndex++;
}
bool M365::check(const uint8_t * message, uint8_t size){
    unsigned long cksm = 0;
    for(int i = 0; i < size - 3; i++) cksm += message[i];
    cksm ^= 0xFFFF;
    unsigned long mes = message[size - 2];
    mes = mes << 8;
    if(cksm == (message[size - 3] + mes)){
        if(message[1] == 0x20) return false;
        lastValidMessageTimeStamp = millis();    
        return true;
    }
    return false;
}
void M365::translate(const uint8_t * message, uint8_t size){
    switch(message[1]){
        case 0x21:
            switch(message[2]){
                case 0x01:
                    if(message[2] == 0x61) stats.tail = message[3];
                    break;
                case 0x64:
                    stats.eco = message[4];
                    stats.led = message[5];
                    stats.night = message[6];
                    stats.beep = message[7];
                    break;
            }
            break;
        case 0x23:
            switch(message[3]){
                case 0x7B:
                    stats.ecoMode = message[4];
                    stats.cruise = message[6];
                    break;
                case 0x7D:
                    
                    stats.tail = message[4];
                    break;
                case 0xB0:
                    
                    stats.alarmStatus = message[6];
                    stats.lock = message[8];
                    stats.battery = message[12];
                    stats.velocity = ( message[14] + (message[15] * 256)) / 1000 / 1.60934;
                    stats.averageVelocity = (message[16] + (message[17] * 265)) / 1000 / 1.60934;
                    stats.odometer = (message[18] + (message[19] * 256) + (message[20] * 256 * 256)) / 1000 / 1.60934;
                    stats.temperature = ((message[26] + (message[27] * 256)) / 10 * 9 / 5) + 32;
                    if(stats.alarmStatus) tone(input.buzzer, 20, 400);
                    break;
            }
    }
    //compare();
}
void M365::compare(){
    static unsigned long lastDelayedBackgroundProcessTimeStamp = 0;
    
    //Process LED Changes
    if(!command.led){
        digitalWrite(input.rled, LOW);
        digitalWrite(input.gled, LOW);
        digitalWrite(input.bled, LOW);
    }
    else{
        if(stats.lock){
            digitalWrite(input.rled, HIGH);
            digitalWrite(input.gled, LOW);
            digitalWrite(input.bled, LOW);
        }
        else{
            digitalWrite(input.rled, LOW);
            digitalWrite(input.gled, HIGH);
            digitalWrite(input.bled, LOW);
        }
    }
    
    //Process Headlight Changes
    if((command.head || stats.night) && !stats.lock) digitalWrite(input.headlight, HIGH);
    else digitalWrite(input.headlight, LOW);
    
    //Process Power Switch Commands (One at a Time with a Delay)
    if(millis() - lastDelayedBackgroundProcessTimeStamp > 1000 && isConnected()){
        //Process Power Off Command
        if(!command.power && isConnected()){
            if(command.led) command.led = 0; //Turn Off Light
            if(command.lock) command.lock = 0; //Required to be unlocked to be manually turned off.
            else{
                digitalWrite(input.power, HIGH);
                delay(2000);
                digitalWrite(input.power, LOW);
                Serial.println("POWER1");
            }
        }
        //Process Night Mode Changes
        else if((stats.night && !command.night) || (!stats.night && command.night)){
            digitalWrite(input.power, HIGH);
            delay(100);
            digitalWrite(input.power, LOW);
            Serial.println("POWER2");
        }
        
        //Process Eco On/Off Changes
        else if(((!stats.eco || stats.eco == 1) && command.eco) || ((stats.eco == 2 || stats.eco == 3) && !command.eco)){
            digitalWrite(input.power, HIGH);
            delay(100);
            digitalWrite(input.power, LOW);
            delay(50);
            digitalWrite(input.power, HIGH);
            delay(100);
            digitalWrite(input.power, LOW);
            Serial.println("POWER3");
        }
        
        //Process Headlight Status when 0 (usually indicates error state).
        else if(!stats.led && stats.battery > 30){
            //digitalWrite(input.power, HIGH);
            //delay(100);
            //digitalWrite(input.power, LOW);
            //Serial.println("POWER4");
        }
        
        lastDelayedBackgroundProcessTimeStamp = millis();
    } 
    
    //Process Beep Request
    if(stats.beep == 1 && !stats.alarmStatus) tone(input.buzzer, 20, 50);
    else if(stats.beep == 2){
        tone(input.buzzer, 20, 100);
        stats.beep = 1;
    }
    else if(stats.beep == 3){
        tone(input.buzzer, 20, 50);
        stats.beep = 1;
    }
    
    //Process Cruise Change
    if(!command.cruise && stats.cruise){
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7C, 0x0, 0x0, 0x5C, 0xFF};
        transmit(message, sizeof(message));
    }
    else if(command.cruise && !stats.cruise){
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7C, 0x1, 0x0, 0x5B, 0xFF};
        transmit(message, sizeof(message));
    }
    
    //Process Tail Change
    /*if(stats.tail && !command.tail){
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7D, 0x0, 0x0, 0x5B, 0xFF};
        transmit(message, sizeof(message));
    }
    else if(!stats.tail && command.tail){
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7D, 0x2, 0x0, 0x59, 0xFF};
        transmit(message, sizeof(message));
    } */
    
    //Process Eco Mode Change
    if(!stats.ecoMode){
        if(command.ecoMode == 1){
            uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7B, 0x01, 0x0, 0x5C, 0xFF};
            transmit(message, sizeof(message));
        }
        else if(command.ecoMode == 2){
            uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7B, 0x02, 0x0, 0x5B, 0xFF};
            transmit(message, sizeof(message));
        }
    }
    else if(stats.ecoMode == 1){
        if(!command.ecoMode){
            uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7B, 0x00, 0x0, 0x5D, 0xFF};
            transmit(message, sizeof(message));
        }
        else if(command.ecoMode == 2){
            uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7B, 0x02, 0x0, 0x5B, 0xFF};
            transmit(message, sizeof(message));
        }
    }
    else if(stats.ecoMode ==2){
        if(!command.ecoMode){
            uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7B, 0x00, 0x0, 0x5D, 0xFF};
            transmit(message, sizeof(message));
        }
        else if(command.ecoMode == 1){
            uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7B, 0x01, 0x0, 0x5C, 0xFF};
            transmit(message, sizeof(message));
        }
    }

    //return;
    //Process Lock Change
    /*if(stats.lock && !command.lock){
      Serial.println("CHANGED");
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x71, 0x1, 0x0, 0x66, 0xFF};
        transmit(message, sizeof(message));
    }
    else if(!stats.lock && command.lock){
      Serial.println("CHANGED");
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x70, 0x1, 0x0, 0x67, 0xFF};
        transmit(message, sizeof(message));
    }*/

}

void M365::setScooterLock(){
    if(stats.lock && !command.lock){
      Serial.println("CHANGED");
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x71, 0x1, 0x0, 0x66, 0xFF};
        transmit(message, sizeof(message));
    }
    else if(!stats.lock && command.lock){
      Serial.println("CHANGED");
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x70, 0x1, 0x0, 0x67, 0xFF};
        transmit(message, sizeof(message));
    }

     
}

void M365::setScooterTail(){
    if(stats.tail && !command.tail){
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7D, 0x0, 0x0, 0x5B, 0xFF};
        transmit(message, sizeof(message));
    }
    else if(!stats.tail && command.tail){
        uint8_t message[] = {0x55, 0xAA, 0x4, 0x20, 0x3, 0x7D, 0x2, 0x0, 0x59, 0xFF};
        transmit(message, sizeof(message));
    }
}

bool M365::isConnected(){
    if(millis() < 1000) return false;
    else if(millis() - lastValidMessageTimeStamp < 1000){
        stats.isConnected = 1;
        stats.hasConnected = 1;
        messageTimer->setInterval(20);
        return true;
    }
    else if(command.power && millis() - lastValidMessageTimeStamp > 3000){
        digitalWrite(input.power, HIGH);
        delay(100);
        digitalWrite(input.power, LOW);
        Serial.println("POWER5");
    }
    stats.hasConnected = 0;
    messageTimer->setInterval(1000);
    return false;
}
uint8_t M365::setAnalogPin(uint8_t pin){
       pinMode(pin, INPUT);
       return pin;
}
uint8_t M365::setDigitalPin(uint8_t pin){
        pinMode(pin, OUTPUT);
        return pin;
}
uint8_t M365::getBrake(){
    uint16_t brake = analogRead(input.brake);
    //Serial.println(brake);
    if(!input.brake || brake < 250){
        stats.brakeConnected = 0;
        input_stats.minBrake = 260;
        input_stats.maxBrake = 1024;
        return 0x26;
    }
    else{
        stats.brakeConnected = 1;
        if(input_stats.minBrake > brake) input_stats.minBrake = brake;
        if(input_stats.maxBrake < brake) input_stats.maxBrake = brake;
        return map(brake, input_stats.minBrake, input_stats.maxBrake, 0x26, 0xD2);
    }
}
uint8_t M365::getThrottle(){
    uint16_t speed = analogRead(input.throttle);
    //Serial.println(speed);
    if(!input.throttle || speed < 250){
        stats.throttleConnected = 0;
        input_stats.minSpeed = 260;
        input_stats.maxSpeed = 1024;
        return 0x26;
    }
    else{
        stats.throttleConnected = 1;
        if(input_stats.minSpeed > speed) input_stats.minSpeed = speed;
        if(input_stats.maxSpeed < speed) input_stats.maxSpeed = speed;
        return map(speed, input_stats.minSpeed, input_stats.maxSpeed, 0x26, 0xD2);
    }
}
