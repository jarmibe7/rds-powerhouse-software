#include "encoder.hpp"

bool Encoder::takeMeasurement() {
    Serial.println("Bad");
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
    SPI.begin();
    pinMode(AS5147_CS, OUTPUT);
    // pinMode(AS5147_MISO, INPUT);
    // pinMode(AS5147_MOSI, OUTPUT);
    // pinMode(AS5147_SCK, OUTPUT);
}

bool AS5147::takeMeasurement() {
    SPI.beginTransaction(this->settings);
    digitalWrite(AS5147_CS, LOW);
    delayNanoseconds(2 * AS5147_TCSN);
    // Serial.println(this->dataFrame(AS5147_ANGLECOM), HEX);

    uint16_t readMsg = (this->readFrame(AS5147_ANGLEUNC));
    uint8_t readHigh = (uint8_t)(readMsg >> 8);
    uint8_t readLow = (uint8_t)(readMsg & 0xff);
    // SPI.transfer((uint16_t)(this->dataFrame(AS5147_ANGLECOM)));
    SPI.transfer(readHigh);
    SPI.transfer(readLow);

    digitalWrite(AS5147_CS, HIGH);

    delayNanoseconds(2 * AS5147_TCSN);
    digitalWrite(AS5147_CS, LOW);

    readMsg = (this->readFrame(AS5147_NOP));
    readHigh = (uint8_t)(readMsg >> 8);
    readLow = (uint8_t)(readMsg & 0xff);
    uint8_t rawAngleHigh = SPI.transfer(readHigh);
    uint8_t rawAngleLow = SPI.transfer(readLow);

    uint16_t rawAngle = ((uint16_t) rawAngleHigh) << 8 | rawAngleLow;

    Serial.print("Raw encoder output: 0x");
    Serial.println(rawAngle, HEX);

    digitalWrite(AS5147_CS, HIGH);
    SPI.endTransaction();

    // Incorrect parity bit recieved
    if(!checkParity(rawAngle)) {
        Serial.println("incorrect parity");
        return false;
    }

    float angle = (2 * M_PI * ((float)(0x3FFF & rawAngle))) / ((1 << 14) - 1);

    if(this->inverted) this->measuredAngle.angle = 2 * M_PI - angle;
    else this->measuredAngle.angle = angle;
    this->updateVelocity();

    Serial.print("Encoder value: ");
    Serial.println(angle);

    return true;
}

uint16_t AS5147::readFrame(uint16_t address) {
    return (this->getParity(address) << 15) | (1<<14) | (0x3FFF & address);
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
