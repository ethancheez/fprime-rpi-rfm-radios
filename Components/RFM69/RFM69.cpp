// ======================================================================
// \title  RFM69.cpp
// \author ethan
// \brief  cpp file for RFM69 component implementation class
// ======================================================================

#include "Components/RFM69/RFM69.hpp"
#include <Os/RawTime.hpp>
#include "FpConfig.hpp"

#include <Fw/Logger/Logger.hpp>

namespace Radios {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

RFM69::RFM69(const char* const compName)
    : RFM69ComponentBase(compName), m_mode(RF69_Mode::Initialising), m_powerLevel(13) {}

RFM69::~RFM69() {}

void RFM69::config(bool isHighPowerModule) {
    FW_ASSERT(this->isConnected_radioReset_OutputPort(0));
    FW_ASSERT(this->isConnected_spiReadWrite_OutputPort(0));

    this->radioReset_out(0, Fw::Logic::HIGH);
    Os::Task::delay(Fw::TimeInterval(0, 10000));  // 10 ms
    this->radioReset_out(0, Fw::Logic::LOW);
    Os::Task::delay(Fw::TimeInterval(0, 10000));  // 10 ms

    U8 deviceType = this->readRegister(RF69_REG_10_VERSION);
    FW_ASSERT(deviceType != 0x00 && deviceType != 0xFF, deviceType);  // deviceType should not be 0x00 or 0xFF

    this->setModeIdle();

    // Configure important RFM69 registers
    // Here we set up the standard packet format for use by the RF69 library:
    // 4 bytes preamble
    // 2 SYNC words 2d, d4
    // 2 CRC CCITT octets computed on the header, length and data (this in the modem config data)
    // 0 to 60 bytes data
    // RSSI Threshold -114dBm
    // We dont use the RF69s address filtering: instead we prepend our own headers to the beginning of the RF69 payload
    this->writeRegister(RF69_REG_3C_FIFOTHRESH,
                        RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY | 0x0F);  // thresh 15 is default
    // RSSITHRESH is default
    // this->writeRegister(RF69_REG_29_RSSITHRESH, 220); // -110 dbM
    // SYNCCONFIG is default. SyncSize is set later by setSyncWords()
    // this->writeRegister(RF69_REG_2E_SYNCCONFIG, RF69_SYNCCONFIG_SYNCON); // auto, tolerance 0
    // PAYLOADLENGTH is default
    // this->writeRegister(RF69_REG_38_PAYLOADLENGTH, RF69_FIFO_SIZE); // max size only for RX
    // PACKETCONFIG 2 is default
    this->writeRegister(RF69_REG_6F_TESTDAGC, RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);
    // If high power boost set previously, disable it
    this->writeRegister(RF69_REG_5A_TESTPA1, RF69_TESTPA1_NORMAL);
    this->writeRegister(RF69_REG_5C_TESTPA2, RF69_TESTPA2_NORMAL);

    // The following can be changed later by the user if necessary.
    // Set up default configuration
    U8 syncwords[] = {0x2D, 0xD4};
    this->setSyncWords(syncwords, sizeof(syncwords));

    // Reasonably fast and reliable default speed and modulation
    this->setModemConfig(GFSK_Rb250Fd250);

    this->setPreambleLength(4);
    this->setFrequency(915.0);
    this->setEncryptionKey(nullptr);  // No encryption

    if (isHighPowerModule) {
        this->setTxPower(20, true);
    } else {
        this->setTxPower(13);  // +13dBm, same as power-on default
    }
    this->m_isHighPowerModule = isHighPowerModule;

    // Ready to begin transmitting
    Fw::Success radioSuccess = Fw::Success::SUCCESS;
    if (this->isConnected_comStatus_OutputPort(0)) {
        this->comStatus_out(0, radioSuccess);
    }
    Fw::Logger::log("Configured RFM69!\n");
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

Drv::SendStatus RFM69::comDataIn_handler(FwIndexType portNum, Fw::Buffer& sendBuffer) {
    FW_ASSERT(this->isConnected_comStatus_OutputPort(0));

    U32 bytesRemaining = sendBuffer.getSize();
    U32 offset = 0;
    // Fw::Logger::log("Starting send total of %d bytes\n", bytesRemaining);
    while (bytesRemaining > 0) {
        U8 chunkSize = FW_MIN(bytesRemaining, RF69_MAX_MESSAGE_LEN);

        this->send(&sendBuffer.getData()[offset], chunkSize);

        bytesRemaining -= chunkSize;
        offset += chunkSize;
        // Fw::Logger::log("Sending %d bytes. Remaining: %d\n", chunkSize, bytesRemaining);
    }

    this->deallocate_out(0, sendBuffer);

    this->m_packetsSent++;
    if (this->isConnected_tlmOut_OutputPort(0)) {
        this->tlmWrite_PacketsSent(this->m_packetsSent);
    }

    // Ready to transmit another packet
    Fw::Success radioSuccess = Fw::Success::SUCCESS;
    if (this->isConnected_comStatus_OutputPort(0)) {
        this->comStatus_out(0, radioSuccess);
    }
    return Drv::SendStatus::SEND_OK;  // Always send ok to deframer as it does not handle this anyway
}

void RFM69::radioInterrupt_handler(FwIndexType portNum, Os::RawTime& cycleStart) {
    // Get the interrupt cause
    U8 irqflags2 = this->readRegister(RF69_REG_28_IRQFLAGS2);
    if (this->m_mode == RF69_Mode::Tx && (irqflags2 & RF69_IRQFLAGS2_PACKETSENT)) {
        // A transmitter message has been fully sent
        this->setModeIdle();  // Clears FIFO
        // Fw::Logger::log("Packet sent\n");
    }

    // Must look for PAYLOADREADY, not CRCOK, since only PAYLOADREADY occurs _after_ AES decryption
    // has been done
    if (this->m_mode == RF69_Mode::Rx && (irqflags2 & RF69_IRQFLAGS2_PAYLOADREADY)) {
        // A complete message has been received with good CRC
        this->m_lastRssi = -((int8_t)(this->readRegister(RF69_REG_24_RSSIVALUE) >> 1));

        this->setModeIdle();
        // Save it in our buffer
        this->readFIFO();
        // Fw::Logger::log("Packet received\n");
    }
}

void RFM69::run_handler(FwIndexType portNum, U32 context) {
    if (this->available()) {
        U8 bufLen = RF69_MAX_MESSAGE_LEN;
        U8 buf[bufLen];
        this->recv(buf, &bufLen);

        if (bufLen == 0) {
            return;
        }

        Fw::Buffer recvBuffer = this->allocate_out(0, bufLen);
        memcpy(recvBuffer.getData(), buf, bufLen);
        recvBuffer.setSize(bufLen);
        this->m_packetsReceived++;

        // Fw::Logger::log("Received packet of size %d: ", bufLen);
        // for (U32 i = 0; i < bufLen; i++) {
        //     Fw::Logger::log("%d ", buf[i]);
        // }
        // Fw::Logger::log("\n");

        if (this->isConnected_tlmOut_OutputPort(0)) {
            this->tlmWrite_PacketsReceived(this->m_packetsReceived);
            this->tlmWrite_RSSI(this->m_lastRssi);
        }

        this->comDataOut_out(0, recvBuffer, Drv::RecvStatus::RECV_OK);
    }

    if (this->isConnected_tlmOut_OutputPort(0)) {
        this->tlmWrite_TxPowerLevel(this->m_powerLevel);
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void RFM69::SET_TX_POWER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, I8 power) {
    this->setTxPower(power, this->m_isHighPowerModule);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

bool RFM69::available() {
    if (this->m_mode == RF69_Mode::Tx) {
        return false;
    }
    this->setModeRx();
    return this->m_recvBufferValid;
}

void RFM69::send(U8* data, U8 len) {
    FW_ASSERT(len <= RF69_MAX_MESSAGE_LEN, len);  // Should never be more than the max message length

    // Cannot send 0 byte payloads.
    if (len == 0) {
        return;
    }

    this->waitPacketSent();  // Make sure we dont interrupt an outgoing message
    this->setModeIdle();     // Prevent RX while filling the fifo

    U8 buffer[len + 5] = {0};
    FW_ASSERT(sizeof(buffer) <= RF69_FIFO_SIZE - 1, sizeof(buffer));  // Should never be more than the FIFO size

    // Include length of headers
    buffer[0] = len + RF69_HEADER_LEN;
    // First the 4 headers
    buffer[1] = this->m_destinationAddress;
    buffer[2] = this->m_originAddress;
    buffer[3] = this->m_headerId;
    buffer[4] = this->m_headerFlags;
    // Payload
    memcpy(&buffer[5], data, len);

    // Send it!
    this->writeBurstRegister(RF69_REG_00_FIFO, buffer, sizeof(buffer));
    this->setModeTx();
}

void RFM69::recv(U8* data, U8* len) {
    FW_ASSERT(data != nullptr);
    FW_ASSERT(len != nullptr);

    if (!this->available()) {
        *len = 0;
        return;
    }

    if (*len > this->m_recvBufferLen) {
        *len = this->m_recvBufferLen;
    }
    memcpy(data, this->m_recvBuffer, *len);

    this->m_recvBufferValid = false;  // Got the most recent message
}

void RFM69::waitPacketSent(U32 timeout_ms) {
    Os::RawTime start;
    start.now();
    while (this->m_mode == RF69_Mode::Tx) {
        Os::RawTime now;
        now.now();
        U32 diffUsec;
        now.getDiffUsec(start, diffUsec);
        U32 diffms = diffUsec / 1000;
        if (diffms > timeout_ms) {
            this->log_WARNING_LO_WaitPacketSentTimeout();
            return;
        }
        Os::Task::delay(Fw::TimeInterval(0, 1000));  // 1 ms
    }
}

void RFM69::readFIFO() {
    U8 payloadLen = this->readRegister(RF69_REG_00_FIFO, false);

    if (payloadLen <= RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN && payloadLen >= RF69_HEADER_LEN) {
        U8 payload[payloadLen] = {0};
        this->readRawBytes(payload, payloadLen);

        U8 destinationAddress = payload[0];
        // Check addressing
        if (destinationAddress == this->m_originAddress || destinationAddress == RF69_BROADCAST_ADDRESS) {
            // Get the rest of the headers
            U8 originAddress = payload[1];
            U8 headerId = payload[2];
            U8 headerFlags = payload[3];
            // And now the real payload
            memcpy(this->m_recvBuffer, &payload[4], payloadLen - RF69_HEADER_LEN);
            this->m_recvBufferLen = payloadLen - RF69_HEADER_LEN;
            this->m_recvBufferValid = true;

            // Fw::Logger::log("Received packet from %d of size %d\n", originAddress, payloadLen - RF69_HEADER_LEN);
        }
    }
}

void RFM69::setOpMode(U8 mode) {
    U8 opmode = this->readRegister(RF69_REG_01_OPMODE);
    opmode &= ~RF69_OPMODE_MODE;
    opmode |= (mode & RF69_OPMODE_MODE);
    this->writeRegister(RF69_REG_01_OPMODE, opmode);

    // Wait for mode to change.
    U8 max_iter = 100;
    while (!(this->readRegister(RF69_REG_27_IRQFLAGS1) & RF69_IRQFLAGS1_MODEREADY)) {
        if (max_iter-- == 0) {
            FW_ASSERT(0);
        }
    }
}

void RFM69::setModeIdle() {
    if (this->m_mode != RF69_Mode::Idle) {
        if (this->m_powerLevel >= 18) {
            // If high power boost, return power amp to receive mode
            this->writeRegister(RF69_REG_5A_TESTPA1, RF69_TESTPA1_NORMAL);
            this->writeRegister(RF69_REG_5C_TESTPA2, RF69_TESTPA2_NORMAL);
        }
        this->setOpMode(RF69_OPMODE_MODE_STDBY);
        this->m_mode = RF69_Mode::Idle;
        // Fw::Logger::log("Setting to IDLE mode\n");
    }
}

void RFM69::setModeTx() {
    if (this->m_mode != RF69_Mode::Tx) {
        if (this->m_powerLevel >= 18) {
            // If high power boost, return power amp to transmit mode
            this->writeRegister(RF69_REG_5A_TESTPA1, RF69_TESTPA1_BOOST);
            this->writeRegister(RF69_REG_5C_TESTPA2, RF69_TESTPA2_BOOST);
        }
        this->writeRegister(RF69_REG_25_DIOMAPPING1,
                            RF69_DIOMAPPING1_DIO0MAPPING_00);  // Set interrupt line 0 PacketSent
        this->setOpMode(RF69_OPMODE_MODE_TX);                  // Clears FIFO
        this->m_mode = RF69_Mode::Tx;
        // Fw::Logger::log("Setting to TX mode\n");
    }
}

void RFM69::setModeRx() {
    if (this->m_mode != RF69_Mode::Rx) {
        if (this->m_powerLevel >= 18) {
            // If high power boost, return power amp to receive mode
            this->writeRegister(RF69_REG_5A_TESTPA1, RF69_TESTPA1_NORMAL);
            this->writeRegister(RF69_REG_5C_TESTPA2, RF69_TESTPA2_NORMAL);
        }
        this->writeRegister(RF69_REG_25_DIOMAPPING1,
                            RF69_DIOMAPPING1_DIO0MAPPING_01);  // Set interrupt line 0 PayloadReady
        this->setOpMode(RF69_OPMODE_MODE_RX);                  // Clears FIFO
        this->m_mode = RF69_Mode::Rx;
        // Fw::Logger::log("Setting to RX mode\n");
    }
}

void RFM69::setSyncWords(U8* syncWords, U8 len) {
    U8 syncconfig = this->readRegister(RF69_REG_2E_SYNCCONFIG);
    if (syncWords && len && len <= 4) {
        this->writeBurstRegister(RF69_REG_2F_SYNCVALUE1, syncWords, len);
        syncconfig |= RF69_SYNCCONFIG_SYNCON;
    } else {
        syncconfig &= ~RF69_SYNCCONFIG_SYNCON;
    }
    syncconfig &= ~RF69_SYNCCONFIG_SYNCSIZE;
    syncconfig |= (len - 1) << 3;
    this->writeRegister(RF69_REG_2E_SYNCCONFIG, syncconfig);
}

// Sets registers from a canned modem configuration structure
void RFM69::setModemRegisters(ModemConfig* config) {
    this->writeBurstRegister(RF69_REG_02_DATAMODUL, &config->reg_02, 5);
    this->writeBurstRegister(RF69_REG_19_RXBW, &config->reg_19, 2);
    this->writeRegister(RF69_REG_37_PACKETCONFIG1, config->reg_37);
}

void RFM69::setModemConfig(ModemConfigChoice index) {
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig))) {
        FW_ASSERT(0, index);
    }

    ModemConfig cfg;
    memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
    this->setModemRegisters(&cfg);
}

void RFM69::setPreambleLength(U16 bytes) {
    this->writeRegister(RF69_REG_2C_PREAMBLEMSB, bytes >> 8);
    this->writeRegister(RF69_REG_2D_PREAMBLELSB, bytes & 0xff);
}

void RFM69::setFrequency(F32 center) {
    // Frf = FRF / FSTEP
    U32 frf = (uint32_t)((center * 1000000.0) / RF69_FSTEP);
    this->writeRegister(RF69_REG_07_FRFMSB, (frf >> 16) & 0xff);
    this->writeRegister(RF69_REG_08_FRFMID, (frf >> 8) & 0xff);
    this->writeRegister(RF69_REG_09_FRFLSB, frf & 0xff);
}

void RFM69::setEncryptionKey(U8* key) {
    if (key) {
        this->writeBurstRegister(RF69_REG_3E_AESKEY1, key, 16);
        this->writeRegister(RF69_REG_3D_PACKETCONFIG2,
                            this->readRegister(RF69_REG_3D_PACKETCONFIG2) | RF69_PACKETCONFIG2_AESON);
    } else {
        this->writeRegister(RF69_REG_3D_PACKETCONFIG2,
                            this->readRegister(RF69_REG_3D_PACKETCONFIG2) & ~RF69_PACKETCONFIG2_AESON);
    }
}

void RFM69::setTxPower(I8 power, bool isHighPowerModule) {
    U8 palevel;

    if (isHighPowerModule) {
        if (power < -2)
            power = -2;  // RFM69HW only works down to -2.
        if (power <= 13) {
            // -2dBm to +13dBm
            // Need PA1 exclusivelly on RFM69HW
            palevel = RF69_PALEVEL_PA1ON | ((power + 18) & RF69_PALEVEL_OUTPUTPOWER);
        } else if (power >= 18) {
            // +18dBm to +20dBm
            // Need PA1+PA2
            // Also need PA boost settings change when tx is turned on and off, see setModeTx()
            palevel = RF69_PALEVEL_PA1ON | RF69_PALEVEL_PA2ON | ((power + 11) & RF69_PALEVEL_OUTPUTPOWER);
        } else {
            // +14dBm to +17dBm
            // Need PA1+PA2
            palevel = RF69_PALEVEL_PA1ON | RF69_PALEVEL_PA2ON | ((power + 14) & RF69_PALEVEL_OUTPUTPOWER);
        }
    } else {
        if (power < -18)
            power = -18;
        if (power > 13)
            power = 13;  // limit for RFM69W
        palevel = RF69_PALEVEL_PA0ON | ((power + 18) & RF69_PALEVEL_OUTPUTPOWER);
    }
    this->writeRegister(RF69_REG_11_PALEVEL, palevel);

    this->m_powerLevel = power;
}

U8 RFM69::readRegister(U8 address, bool readMask) {
    U8 data[2] = {0};
    data[0] = address;
    if (readMask) {
        data[0] &= ~RF69_SPI_WRITE_MASK;  // ~RF69_SPI_WRITE_MASK is 0x7F
    }
    Fw::Buffer buffer(data, 2);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);

    return buffer.getData()[1];
}

void RFM69::readRawBytes(U8* buf, U8 len) {
    U8 data[len + 1] = {0};
    Fw::Buffer buffer(data, len + 1);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);

    memcpy(buf, &buffer.getData()[1], len);
}

void RFM69::writeRegister(U8 address, U8 value) {
    U8 data[2] = {0};
    data[0] = address | RF69_SPI_WRITE_MASK;
    data[1] = value;
    Fw::Buffer buffer(data, 2);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);
}

void RFM69::writeBurstRegister(U8 address, U8* src, U8 len) {
    U8 data[len + 1] = {0};
    data[0] = address | RF69_SPI_WRITE_MASK;
    memcpy(&data[1], src, len);
    Fw::Buffer buffer(data, len + 1);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);
}

}  // namespace Radios
