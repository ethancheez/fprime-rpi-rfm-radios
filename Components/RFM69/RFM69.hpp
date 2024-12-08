// ======================================================================
// \title  RFM69.hpp
// \author ethan
// \brief  hpp file for RFM69 component implementation class
// ======================================================================

#ifndef Radios_RFM69_HPP
#define Radios_RFM69_HPP

#include <Os/Mutex.hpp>
#include <limits>
#include "Components/RFM69/RFM69ComponentAc.hpp"
#include "RFM69_Registers.hpp"

namespace Radios {

class RFM69 : public RFM69ComponentBase {
    // These are indexed by the values of ModemConfigChoice
    // Stored in flash (program) memory to save SRAM
    // It is important to keep the modulation index for FSK between 0.5 and 10
    // modulation index = 2 * Fdev / BR
    // Note that I have not had much success with FSK with Fd > ~5
    // You have to construct these by hand, using the data from the RF69 Datasheet :-(
    // or use the SX1231 starter kit software (Ctl-Alt-N to use that without a connected radio)
    const U8 CONFIG_FSK = (RF69_DATAMODUL_DATAMODE_PACKET | RF69_DATAMODUL_MODULATIONTYPE_FSK |
                           RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE);
    const U8 CONFIG_GFSK = (RF69_DATAMODUL_DATAMODE_PACKET | RF69_DATAMODUL_MODULATIONTYPE_FSK |
                            RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0);
    const U8 CONFIG_OOK = (RF69_DATAMODUL_DATAMODE_PACKET | RF69_DATAMODUL_MODULATIONTYPE_OOK |
                           RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE);

    // Choices for RF69_REG_37_PACKETCONFIG1:
    const U8 CONFIG_NOWHITE = (RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF69_PACKETCONFIG1_DCFREE_NONE |
                               RF69_PACKETCONFIG1_CRC_ON | RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE);
    const U8 CONFIG_WHITE = (RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF69_PACKETCONFIG1_DCFREE_WHITENING |
                             RF69_PACKETCONFIG1_CRC_ON | RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE);
    const U8 CONFIG_MANCHESTER = (RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF69_PACKETCONFIG1_DCFREE_MANCHESTER |
                                  RF69_PACKETCONFIG1_CRC_ON | RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE);

    typedef enum {
        Initialising = 0,  ///< Transport is initialising. Initial default value until init() is called..
        Sleep,             ///< Transport hardware is in low power sleep mode (if supported)
        Idle,              ///< Transport is idle.
        Tx,                ///< Transport is in the process of transmitting a message.
        Rx,                ///< Transport is in the process of receiving a message.
        Cad                ///< Transport is in the process of detecting channel activity (if supported)
    } RF69_Mode;

    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate RF69 registers
    /// to set the desired modulation type, data rate and deviation/bandwidth.
    typedef struct {
        U8 reg_02;  ///< Value for register RF69_REG_02_DATAMODUL
        U8 reg_03;  ///< Value for register RF69_REG_03_BITRATEMSB
        U8 reg_04;  ///< Value for register RF69_REG_04_BITRATELSB
        U8 reg_05;  ///< Value for register RF69_REG_05_FDEVMSB
        U8 reg_06;  ///< Value for register RF69_REG_06_FDEVLSB
        U8 reg_19;  ///< Value for register RF69_REG_19_RXBW
        U8 reg_1a;  ///< Value for register RF69_REG_1A_AFCBW
        U8 reg_37;  ///< Value for register RF69_REG_37_PACKETCONFIG1
    } ModemConfig;

    /// Choices for setModemConfig() for a selected subset of common
    /// modulation types, and data rates. If you need another configuration,
    /// use the register calculator.  and call setModemRegisters() with your
    /// desired settings.
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// CAUTION: some of these configurations do not work corectly and are marked as such.
    typedef enum {
        FSK_Rb2Fd5 = 0,    ///< FSK, Whitening, Rb = 2kbs,    Fd = 5kHz
        FSK_Rb2_4Fd4_8,    ///< FSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
        FSK_Rb4_8Fd9_6,    ///< FSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
        FSK_Rb9_6Fd19_2,   ///< FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
        FSK_Rb19_2Fd38_4,  ///< FSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
        FSK_Rb38_4Fd76_8,  ///< FSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
        FSK_Rb57_6Fd120,   ///< FSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
        FSK_Rb125Fd125,    ///< FSK, Whitening, Rb = 125kbs,  Fd = 125kHz
        FSK_Rb250Fd250,    ///< FSK, Whitening, Rb = 250kbs,  Fd = 250kHz
        FSK_Rb55555Fd50,   ///< FSK, Whitening, Rb = 55555kbs,Fd = 50kHz for RFM69 lib compatibility

        GFSK_Rb2Fd5,        ///< GFSK, Whitening, Rb = 2kbs,    Fd = 5kHz
        GFSK_Rb2_4Fd4_8,    ///< GFSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
        GFSK_Rb4_8Fd9_6,    ///< GFSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
        GFSK_Rb9_6Fd19_2,   ///< GFSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
        GFSK_Rb19_2Fd38_4,  ///< GFSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
        GFSK_Rb38_4Fd76_8,  ///< GFSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
        GFSK_Rb57_6Fd120,   ///< GFSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
        GFSK_Rb125Fd125,    ///< GFSK, Whitening, Rb = 125kbs,  Fd = 125kHz
        GFSK_Rb250Fd250,    ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz
        GFSK_Rb55555Fd50,   ///< GFSK, Whitening, Rb = 55555kbs,Fd = 50kHz

        OOK_Rb1Bw1,        ///< OOK, Whitening, Rb = 1kbs,    Rx Bandwidth = 1kHz.
        OOK_Rb1_2Bw75,     ///< OOK, Whitening, Rb = 1.2kbs,  Rx Bandwidth = 75kHz.
        OOK_Rb2_4Bw4_8,    ///< OOK, Whitening, Rb = 2.4kbs,  Rx Bandwidth = 4.8kHz.
        OOK_Rb4_8Bw9_6,    ///< OOK, Whitening, Rb = 4.8kbs,  Rx Bandwidth = 9.6kHz.
        OOK_Rb9_6Bw19_2,   ///< OOK, Whitening, Rb = 9.6kbs,  Rx Bandwidth = 19.2kHz.
        OOK_Rb19_2Bw38_4,  ///< OOK, Whitening, Rb = 19.2kbs, Rx Bandwidth = 38.4kHz.
        OOK_Rb32Bw64,      ///< OOK, Whitening, Rb = 32kbs,   Rx Bandwidth = 64kHz.
    } ModemConfigChoice;

    ModemConfig MODEM_CONFIG_TABLE[27] = {
        //  02,        03,   04,   05,   06,   19,   1a,  37
        // FSK, No Manchester, no shaping, whitening, CRC, no address filtering
        // AFC BW == RX BW == 2 x bit rate
        // Low modulation indexes of ~ 1 at slow speeds do not seem to work very well. Choose MI of 2.
        {CONFIG_FSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf4, CONFIG_WHITE},  // FSK_Rb2Fd5
        {CONFIG_FSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE},  // FSK_Rb2_4Fd4_8
        {CONFIG_FSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE},  // FSK_Rb4_8Fd9_6

        {CONFIG_FSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE},  // FSK_Rb9_6Fd19_2
        {CONFIG_FSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE},  // FSK_Rb19_2Fd38_4
        {CONFIG_FSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE},  // FSK_Rb38_4Fd76_8

        {CONFIG_FSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE},  // FSK_Rb57_6Fd120
        {CONFIG_FSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE},  // FSK_Rb125Fd125
        {CONFIG_FSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE},  // FSK_Rb250Fd250
        {CONFIG_FSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE},  // FSK_Rb55555Fd50

        //  02,        03,   04,   05,   06,   19,   1a,  37
        // GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
        // AFC BW == RX BW == 2 x bit rate
        {CONFIG_GFSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf5, CONFIG_WHITE},  // GFSK_Rb2Fd5
        {CONFIG_GFSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE},  // GFSK_Rb2_4Fd4_8
        {CONFIG_GFSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE},  // GFSK_Rb4_8Fd9_6

        {CONFIG_GFSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE},  // GFSK_Rb9_6Fd19_2
        {CONFIG_GFSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE},  // GFSK_Rb19_2Fd38_4
        {CONFIG_GFSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE},  // GFSK_Rb38_4Fd76_8

        {CONFIG_GFSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE},  // GFSK_Rb57_6Fd120
        {CONFIG_GFSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE},  // GFSK_Rb125Fd125
        {CONFIG_GFSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE},  // GFSK_Rb250Fd250
        {CONFIG_GFSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE},  // GFSK_Rb55555Fd50

        //  02,        03,   04,   05,   06,   19,   1a,  37
        // OOK, No Manchester, no shaping, whitening, CRC, no address filtering
        // with the help of the SX1231 configuration program
        // AFC BW == RX BW
        // All OOK configs have the default:
        // Threshold Type: Peak
        // Peak Threshold Step: 0.5dB
        // Peak threshiold dec: ONce per chip
        // Fixed threshold: 6dB
        {CONFIG_OOK, 0x7d, 0x00, 0x00, 0x10, 0x88, 0x88, CONFIG_WHITE},  // OOK_Rb1Bw1
        {CONFIG_OOK, 0x68, 0x2b, 0x00, 0x10, 0xf1, 0xf1, CONFIG_WHITE},  // OOK_Rb1_2Bw75
        {CONFIG_OOK, 0x34, 0x15, 0x00, 0x10, 0xf5, 0xf5, CONFIG_WHITE},  // OOK_Rb2_4Bw4_8
        {CONFIG_OOK, 0x1a, 0x0b, 0x00, 0x10, 0xf4, 0xf4, CONFIG_WHITE},  // OOK_Rb4_8Bw9_6
        {CONFIG_OOK, 0x0d, 0x05, 0x00, 0x10, 0xf3, 0xf3, CONFIG_WHITE},  // OOK_Rb9_6Bw19_2
        {CONFIG_OOK, 0x06, 0x83, 0x00, 0x10, 0xf2, 0xf2, CONFIG_WHITE},  // OOK_Rb19_2Bw38_4
        {CONFIG_OOK, 0x03, 0xe8, 0x00, 0x10, 0xe2, 0xe2, CONFIG_WHITE},  // OOK_Rb32Bw64

        //    { CONFIG_FSK,  0x68, 0x2b, 0x00, 0x52, 0x55, 0x55, CONFIG_WHITE}, // works: Rb1200 Fd 5000 bw10000, DCC
        //    400
        //    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x52, 0x52, CONFIG_WHITE}, // works 10/40/80
        //    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x53, 0x53, CONFIG_WHITE}, // works 10/40/40
    };

  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct RFM69 object
    RFM69(const char* const compName  //!< The component name
    );

    //! Destroy RFM69 object
    ~RFM69();

    void config(bool isHighPowerModule = RF69_DEFAULT_HIGHPOWER);

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for comDataIn
    //!
    //! Data coming in from the framing component
    Drv::SendStatus comDataIn_handler(FwIndexType portNum,    //!< The port number
                                      Fw::Buffer& sendBuffer  //!< Data to send
                                      ) override;

    //! Handler implementation for radioInterrupt
    //!
    //! Port sending calls to the GPIO driver
    void radioInterrupt_handler(FwIndexType portNum,     //!< The port number
                                Os::RawTime& cycleStart  //!< Cycle start timestamp
                                ) override;

    //! Handler implementation for run
    //!
    //! Port receiving calls from the rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command SET_TX_POWER
    //!
    //! Command to set the radio transmit power
    void SET_TX_POWER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                 U32 cmdSeq,           //!< The command sequence number
                                 I8 power) override;

  public:
    /// ----------------------------------------------------------------------
    /// Public Helpers
    /// ----------------------------------------------------------------------

    /// @brief Sets the sync words for transmit and receive
    /// Caution: SyncWords should be set to the same
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0x2d, 0xd4 }.
    /// Caution: tests here show that with a single sync word (ie where len == 1),
    /// RFM69 reception can be unreliable.
    /// To disable sync word generation and detection, call with the defaults: setSyncWords();
    /// @param syncWords Array of sync words, 1 to 4 octets long. NULL if no sync words to be used.
    /// @param len Number of sync words to set, 1 to 4. 0 if no sync words to be used.
    void setSyncWords(U8* syncWords = nullptr, U8 len = 0);

    /// @brief Select one of the predefined modem configurations. If you need a modem configuration not provided
    /// here, use setModemRegisters() with your own ModemConfig. The default after init() is RH_RF69::GFSK_Rb250Fd250.
    /// @param index The configuration choice.
    void setModemConfig(ModemConfigChoice index);

    /// @brief Sets the length of the preamble in bytes.
    /// Caution: this should be set to the same
    /// value on all nodes in your network. Default is 4.
    /// Sets the message preamble length in REG_0?_PREAMBLE?SB
    /// @param bytes Preamble length in bytes.
    void setPreambleLength(U16 bytes);

    /// @brief Sets the transmitter and receiver center frequency
    /// @param center Frequency in MHz. 240.0 to 960.0. Caution, RF69 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    void setFrequency(F32 center);

    /// @brief Enables AES encryption and sets the AES encryption key, used
    /// to encrypt and decrypt all messages. The default is disabled.
    /// @param key The key to use. Must be 16 bytes long. The same key must be installed
    /// in other instances of RF69, otherwise communications will not work correctly. If key is NULL,
    /// encryption is disabled, which is the default.
    void setEncryptionKey(U8* key = nullptr);

    /// @brief Sets the transmitter power output level.
    /// Be a good neighbour and set the lowest power level you need.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm for a low power module.
    /// If you are using a high p[ower modfule such as an RFM69HW, you MUST set the power level
    /// with the isHighPowerModule flag set to true. Else you wil get no measurable power output.
    /// Simlarly if you are not using a high power module, you must NOT set the isHighPowerModule
    /// (which is the default)
    /// @param power Transmitter power level in dBm. For RF69W (isHighPowerModule = false),
    /// valid values are from -18 to +13.; Values outside this range are trimmed.
    /// For RF69HW (isHighPowerModule = true), valid values are from -2 to +20.
    /// Caution: at +20dBm, duty cycle is limited to 1% and a
    /// maximum VSWR of 3:1 at the antenna port.
    /// @param isHighPowerModule Set to true if the connected module is a high power module RFM69HW
    void setTxPower(I8 power, bool isHighPowerModule = RF69_DEFAULT_HIGHPOWER);

  private:
    /// ----------------------------------------------------------------------
    /// Private Helpers
    /// ----------------------------------------------------------------------
    
    /// @brief Starts the receiver and checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// @return true if a complete, valid message has been received and is able to be retrieved by recv()
    bool available();

    /// @brief Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// @param data Array of data to be sent
    /// @param len Number of bytes of data to send (> 0)
    void send(U8* data, U8 len);
    
    /// @brief Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// @param data Location to copy the received message
    /// @param len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    void recv(U8* data, U8* len);

    /// @brief Blocks until the transmitter is no longer transmitting.
    /// or until the timeout occuers, whichever happens first
    /// @param timeout_ms Maximum time to wait in milliseconds.
    void waitPacketSent(U32 timeout_ms = 1000);

    /// @brief Low level function to read the FIFO and put the received data into the receive buffer
    /// Should not need to be called by user code.
    void readFIFO();

    /// @brief Sets the parameters for the RF69 OPMODE.
    /// This is a low level device access function, and should not normally need to be used by user code.
    /// Instead can use stModeRx(), setModeTx(), setModeIdle()
    /// @param mode RF69 OPMODE to set, one of RH_RF69_OPMODE_MODE_*.
    void setOpMode(U8 mode);

    /// @brief If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running,
    /// disables them.
    void setModeIdle();
    
    /// @brief If current mode is Rx or Idle, changes it to Tx.
    /// Starts the transmitter in the RF69.
    void setModeTx();

    /// @brief If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF69.
    void setModeRx();
    
    /// @brief Sets all the registers required to configure the data modem in the RF69, including the data rate,
    /// bandwidths etc. You can use this to configure the modem with custom configurations if none of the
    /// canned configurations in ModemConfigChoice suit you.
    /// @param config A ModemConfig structure containing values for the modem configuration registers.
    void setModemRegisters(ModemConfig* config);

    /// @brief Reads a single byte register value from the RF69
    /// @param address Register address to read
    /// @param readMask Whether to add the read mask to the address or not
    /// @return Byte value stored in the register
    U8 readRegister(U8 address, bool readMask = true);

    /// @brief Reads a series of raw bytes from the SPI bus
    /// @param buf Pointer to where the bytes should be stored
    /// @param len The number of bytes to read
    void readRawBytes(U8* buf, U8 len);

    /// @brief Write a byte to a register on the RF69
    /// @param address Register address to write to
    /// @param value Value to write to the register
    void writeRegister(U8 address, U8 value);

    /// @brief Write a series of bytes to a register on the RF69
    /// @param address Register address to write to
    /// @param src Pointer to the data to write
    /// @param len Number of bytes to write
    void writeBurstRegister(U8 address, U8* src, U8 len);

  private:
    U8 m_destinationAddress = RF69_BROADCAST_ADDRESS;
    U8 m_originAddress = RF69_BROADCAST_ADDRESS;
    U8 m_headerId = 0;
    U8 m_headerFlags = 0;

    U8 m_mode;
    I8 m_powerLevel;
    bool m_isHighPowerModule = false;
    I16 m_lastRssi;
    U8 m_recvBuffer[RF69_MAX_MESSAGE_LEN] = {0};
    U8 m_recvBufferLen = 0;
    U8 m_recvBufferValid = false;

    U32 m_packetsSent = 0;
    U32 m_packetsReceived = 0;

    Os::Mutex m_spiMutex;
};

}  // namespace Radios

#endif
