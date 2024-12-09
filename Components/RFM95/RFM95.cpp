// ======================================================================
// \title  RFM95.cpp
// \author ethan
// \brief  cpp file for RFM95 component implementation class
// ======================================================================

#include "Components/RFM95/RFM95.hpp"
#include <Fw/Logger/Logger.hpp>
#include "FpConfig.hpp"

namespace Radios {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

RFM95::RFM95(const char* const compName) : RFM95ComponentBase(compName), m_mode(RF95_Mode::Initialising) {}

RFM95::~RFM95() {}

void RFM95::config() {
    FW_ASSERT(this->isConnected_radioReset_OutputPort(0));
    FW_ASSERT(this->isConnected_spiReadWrite_OutputPort(0));

    this->radioReset_out(0, Fw::Logic::LOW);
    Os::Task::delay(Fw::TimeInterval(0, 10000));  // 10 ms
    this->radioReset_out(0, Fw::Logic::HIGH);
    Os::Task::delay(Fw::TimeInterval(0, 10000));  // 10 ms

    U8 deviceType = this->readRegister(RF95_REG_42_VERSION);
    FW_ASSERT(deviceType != 0x00 && deviceType != 0xFF, deviceType);  // deviceType should not be 0x00 or 0xFF

    // Set sleep mode, so we can also set LORA mode:
    this->writeRegister(RF95_REG_01_OP_MODE, RF95_MODE_SLEEP | RF95_LONG_RANGE_MODE);
    Os::Task::delay(Fw::TimeInterval(0, 10000));  // Wait for sleep mode to take over from say, CAD
    // Check we are in sleep mode, with LORA set
    U8 opMode = this->readRegister(RF95_REG_01_OP_MODE);
    FW_ASSERT(opMode == (RF95_MODE_SLEEP | RF95_LONG_RANGE_MODE), opMode);  // No device

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    this->writeRegister(RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    this->writeRegister(RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    this->setModeIdle();

    // Set up default configuration
    // No Sync Words in LORA mode.
    this->setModemConfig(Bw125Cr45Sf128);  // Radio default
    this->setPreambleLength(8);            // Default is 8
    // An innocuous ISM frequency
    this->setFrequency(915.0);
    // Lowish power
    this->setTxPower(13);

    // Ready to begin transmitting
    Fw::Success radioSuccess = Fw::Success::SUCCESS;
    if (this->isConnected_comStatus_OutputPort(0)) {
        this->comStatus_out(0, radioSuccess);
    }
    Fw::Logger::log("RFM95 configured\n");
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

Drv::SendStatus RFM95::comDataIn_handler(FwIndexType portNum, Fw::Buffer& sendBuffer) {
    FW_ASSERT(this->isConnected_comStatus_OutputPort(0));

    U32 bytesRemaining = sendBuffer.getSize();
    U32 offset = 0;
    // Fw::Logger::log("Starting send total of %d bytes\n", bytesRemaining);
    while (bytesRemaining > 0) {
        U8 chunkSize = FW_MIN(bytesRemaining, RF95_MAX_MESSAGE_LEN);

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

void RFM95::radioInterrupt_handler(FwIndexType portNum, Os::RawTime& cycleStart) {
    // Fw::Logger::log("Handling interrupt\n");

    // Read the interrupt register
    U8 irq_flags = this->readRegister(RF95_REG_12_IRQ_FLAGS);
    // Read the RegHopChannel register to check if CRC presence is signalled
    // in the header. If not it might be a stray (noise) packet.*
    U8 hop_channel = this->readRegister(RF95_REG_1C_HOP_CHANNEL);

    this->writeRegister(RF95_REG_12_IRQ_FLAGS, 0xFF);  // Clear all IRQ flags

    // error if:
    // timeout
    // bad CRC
    // CRC is required but it is not present
    if (this->m_mode == RF95_Mode::Rx && ((irq_flags & (RF95_RX_TIMEOUT | RF95_PAYLOAD_CRC_ERROR)) ||
                                          (this->m_enableCRC && !(hop_channel & RF95_RX_PAYLOAD_CRC_IS_ON)))) {
        this->m_badPackets++;
        this->log_WARNING_LO_BadPacketReceived();
        if (this->isConnected_tlmOut_OutputPort(0)) {
            this->tlmWrite_BadPackets(this->m_badPackets);
        }
        this->clearRxBuf();
    }

    // It is possible to get RX_DONE and CRC_ERROR and VALID_HEADER all at once
    // so this must be an else
    else if (this->m_mode == RF95_Mode::Rx && irq_flags & RF95_RX_DONE) {
        // Packet received, no CRC error
        // Fw::Logger::log("Packet received, no CRC error\n");
        // Have received a packet
        U8 payloadLen = this->readRegister(RF95_REG_13_RX_NB_BYTES);

        // Reset the fifo read ptr to the beginning of the packet
        U8 payload[payloadLen] = {0};
        this->writeRegister(RF95_REG_0D_FIFO_ADDR_PTR, this->readRegister(RF95_REG_10_FIFO_RX_CURRENT_ADDR));
        payload[0] = this->readRegister(RF95_REG_00_FIFO);  // Start reading address
        this->readRawBytes(&payload[1], payloadLen);

        // Remember the last signal to noise ratio, LORA mode
        // Per page 111, SX1276/77/78/79 datasheet
        this->m_lastSNR = (I8)this->readRegister(RF95_REG_19_PKT_SNR_VALUE) / 4;

        // Remember the RSSI of this packet, LORA mode
        // this is according to the doc, but is it really correct?
        // weakest receiveable signals are reported RSSI at about -66
        this->m_lastRssi = this->readRegister(RF95_REG_1A_PKT_RSSI_VALUE);
        // Adjust the RSSI, datasheet page 87
        if (this->m_lastSNR < 0)
            this->m_lastRssi = this->m_lastRssi + this->m_lastSNR;
        else
            this->m_lastRssi = (int)this->m_lastRssi * 16 / 15;
        if (this->m_usingHFport)
            this->m_lastRssi -= 157;
        else
            this->m_lastRssi -= 164;

        // Validate RX Buffer
        if (payloadLen > 4) {
            // Extract the 4 headers
            U8 destinationAddress = payload[0];
            U8 originAddress = payload[1];
            U8 headerId = payload[2];
            U8 headerFlags = payload[3];

            // Fw::Logger::log("Destination: %d, Origin: %d, Header ID: %d, Header Flags: %d\n", destinationAddress,
            //                 originAddress, headerId, headerFlags);

            if (destinationAddress == this->m_originAddress || destinationAddress == RF95_BROADCAST_ADDRESS) {
                // Fw::Logger::log("Valid packet\n");
                memcpy(this->m_recvBuffer, &payload[4], payloadLen - RF95_HEADER_LEN);
                this->m_recvBufferLen = payloadLen - RF95_HEADER_LEN;
                this->m_recvBufferValid = true;
                setModeIdle();
            }
        }

    } else if (this->m_mode == RF95_Mode::Tx && irq_flags & RF95_TX_DONE) {
        setModeIdle();
    } else if (this->m_mode == RF95_Mode::Cad && irq_flags & RF95_CAD_DONE) {
        this->m_cad = irq_flags & RF95_CAD_DETECTED;
        setModeIdle();
    }

    this->writeRegister(RF95_REG_12_IRQ_FLAGS, 0xff);  // Clear all IRQ flags
    this->writeRegister(RF95_REG_12_IRQ_FLAGS, 0xff);  // Clear all IRQ flags
}

void RFM95::run_handler(FwIndexType portNum, U32 context) {
    if (this->available()) {
        U8 bufLen = RF95_MAX_MESSAGE_LEN;
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

void RFM95 ::SET_TX_POWER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, I8 power) {
    this->setTxPower(power);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

bool RFM95::available() {
    if (this->m_mode == RF95_Mode::Tx) {
        return false;
    }
    this->setModeRx();
    return this->m_recvBufferValid;
}

void RFM95::waitPacketSent(U32 timeout_ms) {
    Os::RawTime start;
    start.now();
    while (this->m_mode == RF95_Mode::Tx) {
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

void RFM95::send(U8* data, U8 len) {
    FW_ASSERT(len <= RF95_MAX_MESSAGE_LEN, len);  // Should never be more than the max message length

    // Cannot send 0 byte payloads.
    if (len == 0) {
        return;
    }

    this->waitPacketSent();
    this->setModeIdle();

    // Position at the beginning of the FIFO
    this->writeRegister(RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    this->writeRegister(RF95_REG_00_FIFO, this->m_destinationAddress);
    this->writeRegister(RF95_REG_00_FIFO, this->m_originAddress);
    this->writeRegister(RF95_REG_00_FIFO, this->m_headerId);
    this->writeRegister(RF95_REG_00_FIFO, this->m_headerFlags);
    // The message data
    this->writeBurstRegister(RF95_REG_00_FIFO, data, len);
    this->writeRegister(RF95_REG_22_PAYLOAD_LENGTH, len + RF95_HEADER_LEN);

    this->setModeTx();  // Start the transmitter
}

void RFM95::recv(U8* data, U8* len) {
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

    this->clearRxBuf();  // This message accepted and cleared
}

// Sets registers from a canned modem configuration structure
void RFM95::setModemRegisters(ModemConfig* config) {
    this->writeRegister(RF95_REG_1D_MODEM_CONFIG1, config->reg_1d);
    this->writeRegister(RF95_REG_1E_MODEM_CONFIG2, config->reg_1e);
    this->writeRegister(RF95_REG_26_MODEM_CONFIG3, config->reg_26);
}

void RFM95::setModemConfig(ModemConfigChoice index) {
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig))) {
        FW_ASSERT(0, index);
    }

    ModemConfig cfg;
    memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
    this->setModemRegisters(&cfg);
}

void RFM95::setPreambleLength(U16 bytes) {
    this->writeRegister(RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    this->writeRegister(RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

void RFM95::setFrequency(F32 center) {
    // Frf = FRF / FSTEP
    uint32_t frf = (center * 1000000.0) / RF95_FSTEP;
    this->writeRegister(RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    this->writeRegister(RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    this->writeRegister(RF95_REG_08_FRF_LSB, frf & 0xff);
    this->m_usingHFport = (center >= 779.0);
}

void RFM95::setTxPower(I8 power, bool useRFO) {
    this->m_useRFO = useRFO;

    // Different behaviours depending on whether the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (this->m_useRFO) {
        if (power > 15)
            power = 15;
        if (power < 0)
            power = 0;
        // Set the MaxPower register to 0x7 => MaxPower = 10.8 + 0.6 * 7 = 15dBm
        // So Pout = Pmax - (15 - power) = 15 - 15 + power
        this->writeRegister(RF95_REG_09_PA_CONFIG, RF95_MAX_POWER | power);
        this->writeRegister(RF95_REG_4D_PA_DAC, RF95_PA_DAC_DISABLE);
    } else {
        if (power > 20)
            power = 20;
        if (power < 2)
            power = 2;

        // For RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
        // RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will use it
        // for 8, 19 and 20dBm
        if (power > 17) {
            this->writeRegister(RF95_REG_4D_PA_DAC, RF95_PA_DAC_ENABLE);
            power -= 3;
        } else {
            this->writeRegister(RF95_REG_4D_PA_DAC, RF95_PA_DAC_DISABLE);
        }

        // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
        // pin is connected, so must use PA_BOOST
        // Pout = 2 + OutputPower (+3dBm if DAC enabled)
        this->writeRegister(RF95_REG_09_PA_CONFIG, RF95_PA_SELECT | (power - 2));
    }

    this->m_powerLevel = power;
}

void RFM95::setModeIdle() {
    if (this->m_mode != RF95_Mode::Idle) {
        this->writeRegister(RF95_REG_01_OP_MODE, RF95_MODE_STDBY);
        this->m_mode = RF95_Mode::Idle;
    }
}

void RFM95::setModeRx() {
    if (this->m_mode != RF95_Mode::Rx) {
        this->writeRegister(RF95_REG_01_OP_MODE, RF95_MODE_RXCONTINUOUS);
        this->writeRegister(RF95_REG_40_DIO_MAPPING1, 0x00);  // Interrupt on RxDone
        this->m_mode = RF95_Mode::Rx;
    }
}

void RFM95::setModeTx() {
    if (this->m_mode != RF95_Mode::Tx) {
        this->writeRegister(RF95_REG_01_OP_MODE, RF95_MODE_TX);
        this->writeRegister(RF95_REG_40_DIO_MAPPING1, 0x40);  // Interrupt on TxDone
        this->m_mode = RF95_Mode::Tx;
    }
}

void RFM95::clearRxBuf() {
    memset(this->m_recvBuffer, 0, RF95_MAX_MESSAGE_LEN);
    this->m_recvBufferLen = 0;
    this->m_recvBufferValid = false;
}

U8 RFM95::readRegister(U8 address, bool readMask) {
    U8 data[2] = {0};
    data[0] = address;
    if (readMask) {
        data[0] &= ~RF95_SPI_WRITE_MASK;  // ~RF95_SPI_WRITE_MASK is 0x7F
    }
    Fw::Buffer buffer(data, 2);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);

    return buffer.getData()[1];
}

void RFM95::readRawBytes(U8* buf, U8 len) {
    U8 data[len + 1] = {0};
    Fw::Buffer buffer(data, len + 1);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);

    memcpy(buf, &buffer.getData()[1], len);
}

void RFM95::writeRegister(U8 address, U8 value) {
    U8 data[2] = {0};
    data[0] = address | RF95_SPI_WRITE_MASK;
    data[1] = value;
    Fw::Buffer buffer(data, 2);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);
}

void RFM95::writeBurstRegister(U8 address, U8* src, U8 len) {
    U8 data[len + 1] = {0};
    data[0] = address | RF95_SPI_WRITE_MASK;
    memcpy(&data[1], src, len);
    Fw::Buffer buffer(data, len + 1);

    Os::ScopeLock lock(this->m_spiMutex);
    this->radioChipSelect_out(0, Fw::Logic::LOW);
    this->spiReadWrite_out(0, buffer, buffer);
    this->radioChipSelect_out(0, Fw::Logic::HIGH);
}

}  // namespace Radios
