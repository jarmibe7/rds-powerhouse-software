#include <Arduino.h>
#include <SPI.h>
#define _USE_MATH_DEFINES
#include <math.h>

struct JointAngle {
    float angle = 0.0;
    float velocity = 0.0;
};

class Encoder {
    public:
        virtual void takeMeasurement();
        void updateVelocity();
        virtual void setup();
        JointAngle getAngle();
    private:
        JointAngle measuredAngle;
};


#define AS5147_CS 10
#define AS5147_ANGLECOM 0x3FFF
#define AS5147_TCSN 350

class AS5147 : public Encoder {
    public:
        AS5147(const bool inverted = false);
        void setup() override;
        void takeMeasurement() override;
    private:
        SPISettings settings;
        uint16_t dataFrame(uint16_t address);
        uint16_t getParity(uint16_t val);
        JointAngle measuredAngle;
        const bool inverted;
};