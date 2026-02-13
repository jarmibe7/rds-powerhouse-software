#include "encoder.hpp"

bool Encoder::takeMeasurement() {
    this->measuredAngle = {0.0, 0.0};
    this->updateVelocity();

    Serial.println("Erroneously accessing overridden Encoder::takeMeasurement method");

    return false;
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

AS5147::AS5147(const bool inverted) : settings(2000000, MSBFIRST, SPI_MODE1), inverted(inverted) {}

void AS5147::setup() {
    pinMode(AS5147_CS, OUTPUT);

    SPI.begin();
}

bool AS5147::takeMeasurement() {
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

    // Incorrect parity bit recieved
    if(!checkParity(rawAngle)) return false;

    float angle = (2 * M_PI * ((float)rawAngle)) / ((1 << 14) - 1);

    if(this->inverted) this->measuredAngle.angle = 2 * M_PI - angle;
    else this->measuredAngle.angle = angle;
    this->updateVelocity();

    return true;
}

uint16_t AS5147::dataFrame(uint16_t address) {
    return (this->getParity(address) << 15) | (0x3FFF & address);
}

// AS5147 uses even parity bit
uint16_t AS5147::getParity(uint16_t val) {
    uint16_t temp = val ^ (val >> 1);
    temp ^= (temp >> 2);
    temp ^= (temp >> 4);
    temp ^= (temp >> 8);

    return (~temp) & 0b1;
}

bool AS5147::checkParity(uint16_t val) {
    // only use bits 13:0 for data
    uint16_t data = val & 0x3FFF;
    uint16_t parityBit = (val & 0x8000) != 0x0;
    uint16_t dataParity = this->getParity(data);

    bool correctParity = !(parityBit ^ dataParity);

    if(!correctParity) {
        Serial.print("Invalid AS5147 parity bit. Data recieved: 0x");
        Serial.println(val, HEX);
    }

    return correctParity;

}
