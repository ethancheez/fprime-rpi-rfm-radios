#include <RH_RF69.h>
#include <SPI.h>

#define RF69_CS 10
#define RF69_INT 20
#define RF69_RST 14

// Singleton instance of the radio driver
RH_RF69 rf69(RF69_CS, RF69_INT);

void setup() {
    Serial.begin(115200);

    // Reset RFM module
    pinMode(RF69_RST, OUTPUT);
    digitalWrite(RF69_RST, HIGH);
    delay(10);
    digitalWrite(RF69_RST, LOW);
    delay(10);

    // Initialize the radio driver
    if (!rf69.init()) {
        Serial.println("init failed");
        while (true)
            ;
    }

    // uint8_t syncwords[] = { 0x2D, 0xD4 };
    // rf69.setSyncWords(syncwords, sizeof(syncwords));

    rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
    rf69.setPreambleLength(4);
    rf69.setFrequency(915.0) rf69.setTxPower(20, true);

    // The encryption key has to be the same as the one in the client
    // uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    //                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    // rf69.setEncryptionKey(key);
    rf69.setEncryptionKey(nullptr);
}

void loop() {
    uint8_t sendBuffer[RH_RF69_MAX_MESSAGE_LEN] = {0};
    uint32_t sendBufferIndex = 0;
    while (Serial.available()) {
        sendBuffer[sendBufferIndex] = Serial.read();
        sendBufferIndex++;
        if (sendBufferIndex == RH_RF69_MAX_MESSAGE_LEN) {
            break;
        }
    }
    if (sendBufferIndex > 0) {
        rf69.send(sendBuffer, sendBufferIndex);
        rf69.waitPacketSent();
    }

    if (rf69.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = {0};
        uint8_t len = RH_RF69_MAX_MESSAGE_LEN;
        if (rf69.recv(buf, &len)) {
            for (uint8_t i = 0; i < len; i++) {
                Serial.write(buf[i]);
            }
        }
    }
}
