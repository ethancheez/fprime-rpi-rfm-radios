#include <RH_RF95.h>
#include <SPI.h>

#define RF95_CS 10
#define RF95_INT 20
#define RF95_RST 14

// Singleton instance of the radio driver
RH_RF95 rf95(RF95_CS, RF95_INT);

void setup() {
    Serial.begin(115200);

    // Reset RFM module
    pinMode(RF95_RST, OUTPUT);
    digitalWrite(RF95_RST, LOW);
    delay(10);
    digitalWrite(RF95_RST, HIGH);
    delay(10);

    if (!rf95.init()) {
        Serial.println("init failed");
        while (true)
            ;
    }

    rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
    rf95.setPreambleLength(8);
    rf95.setFrequency(915.0);
    rf95.setTxPower(20);

    Serial.println("Configured!");
}

void loop() {
    uint8_t sendBuffer[RH_RF95_MAX_MESSAGE_LEN] = {0};
    uint32_t sendBufferIndex = 0;
    while (Serial.available()) {
        sendBuffer[sendBufferIndex] = Serial.read();
        sendBufferIndex++;
        if (sendBufferIndex == RH_RF95_MAX_MESSAGE_LEN) {
            break;
        }
    }
    if (sendBufferIndex > 0) {
        rf95.send(sendBuffer, sendBufferIndex);
        rf95.waitPacketSent();
    }

    if (rf95.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {0};
        uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
        if (rf95.recv(buf, &len)) {
            for (uint8_t i = 0; i < len; i++) {
                Serial.write(buf[i]);
            }
        }
    }
}
