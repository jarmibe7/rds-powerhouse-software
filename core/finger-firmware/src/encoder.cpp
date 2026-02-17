#include "encoder.hpp"


float angleDifference(const float& angleA, const float& angleB) {
    float difference = fmod(angleA - angleB, 2.0 * M_PI);

    if(difference > M_PI) return difference - (2 * M_PI);
    else if(difference < -1 * M_PI) return difference + (2 * M_PI);
    return difference;
}

void Encoder::updateVelocity() {
    uint32_t currentTime = micros();
    float currentAngle = this->measuredAngle.angle;

    static bool firstRead = true;
    static uint32_t lastTime = 0;
    static float lastAngle = 0.0;

    float newVelocity = 0.0;

    if(!firstRead) {
        newVelocity = angleDifference(currentAngle, lastAngle) / (((float)(currentTime - lastTime)) / 1000000);
    } else {
        firstRead = false;
    }

    // Actually update the angle measurement
    this->measuredAngle.velocity = newVelocity;

    lastTime = currentTime;
    lastAngle = currentAngle;
}

// void Encoder::setup() {
//     Serial.println("Erroneously accessing overridden Encoder::setup method");
// }

// // JointAngle Encoder::getAngle() {
// //     return this->measuredAngle;
// // }

AS5147::AS5147(bool inverted) : settings(1000000, MSBFIRST, SPI_MODE1), inverted(inverted) {}

JointAngle AS5147::getAngle() {
    // Serial.print("Encoder getAngle = ");
    Serial.println(measuredAngle.angle);
    return this->measuredAngle;
}

void AS5147::setup() {
    SPI.begin();
    pinMode(AS5147_CS, OUTPUT);
}

bool AS5147::takeMeasurement() {
    static uint8_t failCounter = 0;

    SPI.beginTransaction(this->settings);
    digitalWrite(AS5147_CS, LOW);
    // delayNanoseconds(AS5147_TCSN);
    // Serial.println(this->dataFrame(AS5147_ANGLECOM), HEX);

    uint16_t readMsg = (this->readFrame(AS5147_ANGLECOM));

    uint8_t sendBuf[2];

    sendBuf[0] = (uint8_t)(readMsg >> 8);
    sendBuf[1] = (uint8_t)(readMsg & 0xff);

    // uint8_t readHigh = (uint8_t)(readMsg >> 8);
    // uint8_t readLow = (uint8_t)(readMsg & 0xff);
    // SPI.transfer((uint16_t)(this->dataFrame(AS5147_ANGLECOM)));
    SPI.transfer(sendBuf, 2);
    // SPI.transfer(readLow);

    digitalWrite(AS5147_CS, HIGH);

    // delayNanoseconds(AS5147_TCSN);
    delayMicroseconds(10);
    digitalWrite(AS5147_CS, LOW);

    readMsg = (this->readFrame(AS5147_NOP));
    // readHigh = (uint8_t)(readMsg >> 8);
    // readLow = (uint8_t)(readMsg & 0xff);

    sendBuf[0] = (uint8_t)(readMsg >> 8);
    sendBuf[1] = (uint8_t)(readMsg & 0xff);

    SPI.transfer(sendBuf, 2);
    // uint8_t rawAngleLow = SPI.transfer(readLow);

    uint16_t rawAngle = 0x3FFF & (((uint16_t) sendBuf[0]) << 8 | sendBuf[1]);

    // Serial.print("Raw encoder output: 0x");
    // Serial.println(rawAngle, HEX);

    digitalWrite(AS5147_CS, HIGH);
    SPI.endTransaction();

    // Incorrect parity bit recieved
    if(!checkParity(rawAngle) || !rawAngle) {
        // Serial.println("incorrect parity");
        if(failCounter < 50) {
            failCounter++;
            delayMicroseconds(100);
            return this->takeMeasurement();
        }
        else {
            Serial.println("Exceeded maximum number of encoder read attempts");
            failCounter = 0;
            return false;
        }
        Serial.println("Incorrect parity");
        
    }

    

    float angle = (2 * M_PI) - (2 * M_PI * ((float)(0x3FFF & ((rawAngle) % ((1<<14) - 1))))) / ((1 << 14) - 1);
    // float angle = (360 * ((float)(0x3FFF & rawAngle))) / ((1 << 14) - 1);

    // if(this->inverted) this->measuredAngle.angle = 2 * M_PI - angle;
    // else this->measuredAngle.angle = angle;
    this->measuredAngle.angle = angle;
    // this->updateVelocity();

    Serial.print("Encoder value: ");
    Serial.print(this->measuredAngle.angle);
    Serial.print(" \tFail Counter: ");
    Serial.println(failCounter);

    failCounter = 0;

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
        // Serial.print("Invalid AS5147 parity bit. Data recieved: 0x");
        // Serial.println(val, HEX);
    }

    return correctParity;

}
