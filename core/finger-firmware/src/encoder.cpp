#include "encoder.hpp"

void Encoder::takeMeasurement() {
    this->measuredAngle = {0.0, 0.0};
    this->updateVelocity();

    Serial.println("Erroneously accessing overridden Encoder::takeMeasurement method");
}

void Encoder::updateVelocity() {
    uint32_t currentTime = micros();
    float currentAngle = this->measuredAngle.angle;

    static bool firstRead = true;
    static uint32_t lastTime = 0;
    static float lastAngle = 0.0;

    float newVelocity = 0.0;

    if(!firstRead) {
        newVelocity = (currentAngle - lastAngle) / (((float)(currentTime - lastTime)) / 1000000);
    } else {
        firstRead = false;
    }

    // Actually update the angle measurement
    this->measuredAngle.velocity = newVelocity;

    lastTime = currentTime;
    lastAngle = currentAngle;
}

void Encoder::setup() {
    Serial.println("Erroneously accessing overridden Encoder::setup method");
}

JointAngle Encoder::getAngle() {
    return this->measuredAngle;
}

AS5147::AS5147() : settings(2000000, MSBFIRST, SPI_MODE1) {}

void AS5147::setup() {
    pinMode(AS5147_CS, OUTPUT);

    SPI.begin();
}

void AS5147::takeMeasurement() {
    SPI.beginTransaction(this->settings);
    digitalWrite(AS5147_CS, LOW);
    delayNanoseconds(AS5147_TCSN);

    SPI.transfer(this->dataFrame(AS5147_ANGLECOM));
    digitalWrite(AS5147_CS, HIGH);

    delayNanoseconds(AS5147_TCSN);
    digitalWrite(AS5147_CS, LOW);
    uint16_t rawAngle = SPI.transfer((uint16_t)0);

    digitalWrite(AS5147_CS, HIGH);
    SPI.endTransaction();

    this->measuredAngle.angle = (2 * M_PI * ((float)rawAngle)) / ((1 << 14) - 1);
    this->updateVelocity();
}

uint16_t AS5147::dataFrame(uint16_t address) {
    return (this->getParity(address) << 15) | (0x3FFF & address);
}

uint16_t AS5147::getParity(uint16_t val) {
    uint16_t temp = val ^ (val >> 1);
    temp ^= (temp >> 2);
    temp ^= (temp >> 4);
    temp ^= (temp >> 8);

    return temp & 0b1;
}