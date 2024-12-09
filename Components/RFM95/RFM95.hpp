// ======================================================================
// \title  RFM95.hpp
// \author ethan
// \brief  hpp file for RFM95 component implementation class
// ======================================================================

#ifndef Radios_RFM95_HPP
#define Radios_RFM95_HPP

#include <Os/Mutex.hpp>
#include "Components/RFM95/RFM95ComponentAc.hpp"
#include "RFM95_Registers.hpp"

namespace Radios {

class RFM95 : public RFM95ComponentBase {
    typedef enum {
        Initialising = 0,  ///< Transport is initialising. Initial default value until init() is called..
        Sleep,             ///< Transport hardware is in low power sleep mode (if supported)
        Idle,              ///< Transport is idle.
        Tx,                ///< Transport is in the process of transmitting a message.
        Rx,                ///< Transport is in the process of receiving a message.
        Cad                ///< Transport is in the process of detecting channel activity (if supported)
    } RF95_Mode;

    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate registers
    /// to set the desired spreading factor, coding rate and bandwidth
    typedef struct {
        U8 reg_1d;  ///< Value for register RH_RF95_REG_1D_MODEM_CONFIG1
        U8 reg_1e;  ///< Value for register RH_RF95_REG_1E_MODEM_CONFIG2
        U8 reg_26;  ///< Value for register RH_RF95_REG_26_MODEM_CONFIG3
    } ModemConfig;

    /// Choices for setModemConfig() for a selected subset of common
    /// data rates. If you need another configuration,
    /// determine the necessary settings and call setModemRegisters() with your
    /// desired settings. It might be helpful to use the LoRa calculator mentioned in
    /// http://www.semtech.com/images/datasheet/LoraDesignGuide_STD.pdf
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// Caution: if you are using slow packet rates and long packets with RHReliableDatagram or subclasses
    /// you may need to change the RHReliableDatagram timeout for reliable operations.
    /// Caution: for some slow rates nad with ReliableDatagrams you may need to increase the reply timeout
    /// with manager.setTimeout() to
    /// deal with the long transmission times.
    /// Caution: SX1276 family errata suggests alternate settings for some LoRa registers when 500kHz bandwidth
    /// is in use. See the Semtech SX1276/77/78 Errata Note. These are not implemented by RH_RF95.
    typedef enum {
        Bw125Cr45Sf128 = 0,  ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
        Bw500Cr45Sf128,      ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
        Bw31_25Cr48Sf512,    ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
        Bw125Cr48Sf4096,     ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
        Bw125Cr45Sf2048,     ///< Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
    } ModemConfigChoice;

    ModemConfig MODEM_CONFIG_TABLE[5] = {
        // 1d,  1e,   26
        {0x72, 0x74, 0x04},  // Bw125Cr45Sf128 (the chip default), AGC enabled
        {0x92, 0x74, 0x04},  // Bw500Cr45Sf128, AGC enabled
        {0x48, 0x94, 0x04},  // Bw31_25Cr48Sf512, AGC enabled
        {0x78, 0xc4, 0x0c},  // Bw125Cr48Sf4096, AGC enabled
        {0x72, 0xb4, 0x04},  // Bw125Cr45Sf2048, AGC enabled
    };

  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct RFM95 object
    RFM95(const char* const compName  //!< The component name
    );

    //! Destroy RFM95 object
    ~RFM95();

    void config();

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
    /// ---------------------------------------------------------------------

    /// @brief Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig.
    /// Caution: the slowest protocols may require a radio module with TCXO temperature controlled oscillator
    /// for reliable operation.
    /// @param index The configuration choice.
    void setModemConfig(ModemConfigChoice index);

    /// @brief Sets the length of the preamble in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 8.
    /// Sets the message preamble length in RH_RF95_REG_??_PREAMBLE_?SB
    /// @param bytes Preamble length in bytes.  
    void setPreambleLength(U16 bytes);

    /// @brief Sets the transmitter and receiver center frequency.
    /// @param center Frequency in MHz. 137.0 to 1020.0. Caution: RFM95/96/97/98 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    void setFrequency(F32 center);

    /// @brief Sets the transmitter power output level, and configures the transmitter pin.
    /// Be a good neighbour and set the lowest power level you need.
    /// Some SX1276/77/78/79 and compatible modules (such as RFM95/96/97/98) 
    /// use the PA_BOOST transmitter pin for high power output (and optionally the PA_DAC)
    /// while some (such as the Modtronix inAir4 and inAir9) 
    /// use the RFO transmitter pin for lower power but higher efficiency.
    /// You must set the appropriate power level and useRFO argument for your module.
    /// Check with your module manufacturer which transmtter pin is used on your module
    /// to ensure you are setting useRFO correctly. 
    /// Failure to do so will result in very low 
    /// transmitter power output.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm, with useRFO false (ie PA_BOOST enabled).
    /// @param power Transmitter power level in dBm. For RFM95/96/97/98 LORA with useRFO false, 
    /// valid values are from +2 to +20. For 18, 19 and 20, PA_DAC is enabled, 
    /// For Modtronix inAir4 and inAir9 with useRFO true (ie RFO pins in use), 
    /// valid values are from 0 to 15.
    /// @param useRFO If true, enables the use of the RFO transmitter pins instead of
    /// the PA_BOOST pin (false). Choose the correct setting for your module.
    /// RFM95/96/97/98 usually set to false.
    /// Modtronix inAir4 and inAir9 usually set to true.
    void setTxPower(I8 power, bool useRFO = false);

  private:
    // ----------------------------------------------------------------------
    // Private Helpers
    // ----------------------------------------------------------------------

    /// @brief Tests whether a new message is available from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it will be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// @return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    bool available();

    /// @brief Blocks until the transmitter is no longer transmitting.
    /// or until the timeout occuers, whichever happens first
    /// @param timeout_ms Maximum time to wait in milliseconds.
    void waitPacketSent(U32 timeout_ms = 1000);

    /// @brief Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then optionally waits for Channel Activity Detection (CAD) 
    /// to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// @param data Array of data to be sent
    /// @param len Number of bytes of data to send
    /// specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
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

    /// @brief Sets all the registers required to configure the data modem in the radio, including the bandwidth, 
    /// spreading factor etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// @param config A ModemConfig structure containing values for the modem configuration registers.
    void setModemRegisters(ModemConfig* config);

    /// @brief /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void setModeIdle();

    /// @brief If current mode is Rx or Idle, changes it to Tx.
    /// Starts the transmitter in the RF95/96/97/98.
    void setModeTx();
    
    /// @brief If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF95/96/97/98.
    void setModeRx();

    /// @brief Clear our local receive buffer
    void clearRxBuf();

    /// @brief Reads a single byte register value from the RF95
    /// @param address Register address to read
    /// @param readMask Whether to add the read mask to the address or not
    /// @return Byte value stored in the register
    U8 readRegister(U8 address, bool readMask = true);

    /// @brief Reads a series of raw bytes from the SPI bus
    /// @param buf Pointer to where the bytes should be stored
    /// @param len The number of bytes to read
    void readRawBytes(U8* buf, U8 len);

    /// @brief Write a byte to a register on the RF95
    /// @param address Register address to write to
    /// @param value Value to write to the register
    void writeRegister(U8 address, U8 value);

    /// @brief Write a series of bytes to a register on the RF95
    /// @param address Register address to write to
    /// @param src Pointer to the data to write
    /// @param len Number of bytes to write
    void writeBurstRegister(U8 address, U8* src, U8 len);

  private:
    U8 m_destinationAddress = RF95_BROADCAST_ADDRESS;
    U8 m_originAddress = RF95_BROADCAST_ADDRESS;
    U8 m_headerId = 0;
    U8 m_headerFlags = 0;

    U8 m_mode;
    I8 m_powerLevel;
    bool m_usingHFport = false;
    bool m_useRFO = false;
    bool m_enableCRC = true;

    bool m_cad;
    I16 m_lastRssi;
    I8 m_lastSNR;
    U8 m_recvBuffer[RF95_MAX_MESSAGE_LEN] = {0};
    U8 m_recvBufferLen = 0;
    U8 m_recvBufferValid = false;

    U32 m_packetsSent = 0;
    U32 m_packetsReceived = 0;
    U32 m_badPackets = 0;

    Os::Mutex m_spiMutex;
};

}  // namespace Radios

#endif
