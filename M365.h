#include <timer.h>
#include <timerManager.h>

#ifndef SCOOTER_CONNECT_M365_H
#define SCOOTER_CONNECT_M365_H

#include <Arduino.h>
#include <Stream.h>
#include "wiring_private.h"
struct stats_t{
    uint8_t alarmStatus=0,
        averageVelocity=0,
        battery=0,
        beep=0,
        brakeConnected=0,
        cruise=0,
        eco=0,
        ecoMode=0,
        hasConnected=0,
        isConnected=0,
        led=0,
        lock=0,
        night=0,
        odometer=0,
        tail=0,
        temperature=0,
        throttleConnected=0,
        velocity=0;
};
struct command_t{
    uint8_t alarm = 0,
        cruise = 0,
        eco = 0,
        ecoMode = 0,
        head = 0,
        led = 0,
        lock = 0,
        night = 0,
        power = 1,
        sound = 0,
        tail = 0;
};

class M365{
    private:
        String *ServerResponse;
    
        Uart *serial;
        uint8_t serialBuffer[256], serialIndex = 0;
        Timer * messageTimer;

        stats_t stats;
        command_t command;
        struct input_t{
            uint8_t brake = 0,
                throttle = 0,
                buzzer = 0,
                headlight = 0,
                power = 0,
                rled = 0,
                bled = 0,
                gled = 0;
        } input;
        struct input_stats_t{
            uint16_t minBrake = 260,
                maxBrake = 1024,
                minSpeed = 260,
                maxSpeed = 1024;
        } input_stats;
        
        unsigned long lastMessageSentTimeStamp = 0;
        unsigned long lastValidMessageTimeStamp = 0;
        unsigned long lastPower = 0;
        int *ServerIndex;
        
        uint8_t setAnalogPin(const uint8_t);
        uint8_t setDigitalPin(const uint8_t);
        bool isConnected();
        uint8_t getBrake();
        uint8_t getThrottle();
        
        void read(uint8_t);
        bool check(const uint8_t * message, uint8_t size);
        void translate(const uint8_t * message, uint8_t size);
        void compare();
        
        void addSum(uint8_t *, uint8_t);

    public:
        M365();
        ~M365();
        
        void transmit(const uint8_t *, uint8_t, bool = false);
        void send();
        void updateTimer();
        void setup(Uart &, uint8_t, uint8_t, uint8_t, 
                uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,int &,Timer &);
        void setSerial(Uart &);
        void setBrake(uint8_t);
        void setThrottle(uint8_t);
        void setBuzzer(uint8_t);
        void setRGB(uint8_t, uint8_t, uint8_t);
        void setHeadlight(uint8_t);
        void setPower(uint8_t);
        void setScooterLock();
        void setScooterTail();
        
        void process();
        void setCommand(const command_t *);
        void getStats(stats_t *);
};

#endif
