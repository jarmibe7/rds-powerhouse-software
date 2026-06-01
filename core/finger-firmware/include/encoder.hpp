#include <Arduino.h>
#include <SPI.h>
#define _USE_MATH_DEFINES
#include <math.h>

struct JointAngle {
    float angle = 0.0;
    float velocity = 0.0;
};

float angleDifference(const float& angleA, const float& angleB);

class Encoder {
    public:
        virtual bool takeMeasurement() = 0;
        void updateVelocity();
        virtual void setup();
        JointAngle getAngle();
    private:
        JointAngle measuredAngle;
};


#define AS5147_CS 10
#define AS5147_MISO 12
#define AS5147_MOSI 11
#define AS5147_SCK 13

#define AS5147_TCSN 350

#define AS5147_NOP 0x0000
#define AS5147_DIAAGC 0x3FFC
#define AS5147_MAG 0x3FFD
#define AS5147_ANGLEUNC 0x3FFE
#define AS5147_ANGLECOM 0x3FFF

#define AS5147_MAX_READING (1<<14) - 1



class AS5147 : public Encoder {
    public:
        AS5147(bool inverted = false);
        void setup();
        bool takeMeasurement() override;
        // void updateVelocity();
        JointAngle getAngle();
    private:
        SPISettings settings;
        uint16_t readFrame(uint16_t address);
        uint16_t getParity(uint16_t val);
        bool checkParity(uint16_t val);
        JointAngle measuredAngle;
        bool inverted;
};
