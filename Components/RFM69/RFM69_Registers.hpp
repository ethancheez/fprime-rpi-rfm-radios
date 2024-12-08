#ifndef RF69_REGISTERS_HPP
#define RF69_REGISTERS_HPP

// This is the address that indicates a broadcast
#define RF69_BROADCAST_ADDRESS 0xFF
 
// The crystal oscillator frequency of the RF69 module
#define RF69_FXOSC 32000000.0
 
// The Frequency Synthesizer step = RF69_FXOSC / 2^^19
#define RF69_FSTEP  (RF69_FXOSC / 524288)
 
// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RF69_NUM_INTERRUPTS 3
 
// This is the bit in the SPI address that marks it as a write
#define RF69_SPI_WRITE_MASK 0x80
 
// Max number of octets the RF69 Rx and Tx FIFOs can hold
#define RF69_FIFO_SIZE 66
 
// Maximum encryptable payload length the RF69 can support
#define RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN 64
 
// The length of the headers we add.
// The headers are inside the RF69's payload and are therefore encrypted if encryption is enabled
#define RF69_HEADER_LEN 4
 
// This is the maximum message length that can be supported by this driver. Limited by
// the size of the FIFO, since we are unable to support on-the-fly filling and emptying 
// of the FIFO.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 4 bytes of address and header and payload to be included in the 64 byte encryption limit.
// the one byte payload length is not encrpyted
#ifndef RF69_MAX_MESSAGE_LEN
#define RF69_MAX_MESSAGE_LEN (RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN - RF69_HEADER_LEN)
#endif
 
// Keep track of the mode the RF69 is in
#define RF69_MODE_IDLE         0
#define RF69_MODE_RX           1
#define RF69_MODE_TX           2
 
// This is the default node address,
#define RF69_DEFAULT_NODE_ADDRESS 0
 
// You can define the following macro (either by editing here or by passing it as a compiler definition
// to change the default value of the ishighpowermodule argument to setTxPower to true
// 
// #define RFM69_HW
#ifdef RFM69_HW
#define RF69_DEFAULT_HIGHPOWER true
#else
#define RF69_DEFAULT_HIGHPOWER false
#endif
 
// Register names
#define RF69_REG_00_FIFO                                 0x00
#define RF69_REG_01_OPMODE                               0x01
#define RF69_REG_02_DATAMODUL                            0x02
#define RF69_REG_03_BITRATEMSB                           0x03
#define RF69_REG_04_BITRATELSB                           0x04
#define RF69_REG_05_FDEVMSB                              0x05
#define RF69_REG_06_FDEVLSB                              0x06
#define RF69_REG_07_FRFMSB                               0x07
#define RF69_REG_08_FRFMID                               0x08
#define RF69_REG_09_FRFLSB                               0x09
#define RF69_REG_0A_OSC1                                 0x0a
#define RF69_REG_0B_AFCCTRL                              0x0b
#define RF69_REG_0C_RESERVED                             0x0c
#define RF69_REG_0D_LISTEN1                              0x0d
#define RF69_REG_0E_LISTEN2                              0x0e
#define RF69_REG_0F_LISTEN3                              0x0f
#define RF69_REG_10_VERSION                              0x10
#define RF69_REG_11_PALEVEL                              0x11
#define RF69_REG_12_PARAMP                               0x12
#define RF69_REG_13_OCP                                  0x13
#define RF69_REG_14_RESERVED                             0x14
#define RF69_REG_15_RESERVED                             0x15
#define RF69_REG_16_RESERVED                             0x16
#define RF69_REG_17_RESERVED                             0x17
#define RF69_REG_18_LNA                                  0x18
#define RF69_REG_19_RXBW                                 0x19
#define RF69_REG_1A_AFCBW                                0x1a
#define RF69_REG_1B_OOKPEAK                              0x1b
#define RF69_REG_1C_OOKAVG                               0x1c
#define RF69_REG_1D_OOKFIX                               0x1d
#define RF69_REG_1E_AFCFEI                               0x1e
#define RF69_REG_1F_AFCMSB                               0x1f
#define RF69_REG_20_AFCLSB                               0x20
#define RF69_REG_21_FEIMSB                               0x21
#define RF69_REG_22_FEILSB                               0x22
#define RF69_REG_23_RSSICONFIG                           0x23
#define RF69_REG_24_RSSIVALUE                            0x24
#define RF69_REG_25_DIOMAPPING1                          0x25
#define RF69_REG_26_DIOMAPPING2                          0x26
#define RF69_REG_27_IRQFLAGS1                            0x27
#define RF69_REG_28_IRQFLAGS2                            0x28
#define RF69_REG_29_RSSITHRESH                           0x29
#define RF69_REG_2A_RXTIMEOUT1                           0x2a
#define RF69_REG_2B_RXTIMEOUT2                           0x2b
#define RF69_REG_2C_PREAMBLEMSB                          0x2c
#define RF69_REG_2D_PREAMBLELSB                          0x2d
#define RF69_REG_2E_SYNCCONFIG                           0x2e
#define RF69_REG_2F_SYNCVALUE1                           0x2f
// another 7 sync word bytes follow, 30 through 36 inclusive
#define RF69_REG_37_PACKETCONFIG1                        0x37
#define RF69_REG_38_PAYLOADLENGTH                        0x38
#define RF69_REG_39_NODEADRS                             0x39
#define RF69_REG_3A_BROADCASTADRS                        0x3a
#define RF69_REG_3B_AUTOMODES                            0x3b
#define RF69_REG_3C_FIFOTHRESH                           0x3c
#define RF69_REG_3D_PACKETCONFIG2                        0x3d
#define RF69_REG_3E_AESKEY1                              0x3e
// Another 15 AES key bytes follow
#define RF69_REG_4E_TEMP1                                0x4e
#define RF69_REG_4F_TEMP2                                0x4f
#define RF69_REG_58_TESTLNA                              0x58
#define RF69_REG_5A_TESTPA1                              0x5a
#define RF69_REG_5C_TESTPA2                              0x5c
#define RF69_REG_6F_TESTDAGC                             0x6f
#define RF69_REG_71_TESTAFC                              0x71
 
// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RFM69 Manual
 
// RF69_REG_01_OPMODE
#define RF69_OPMODE_SEQUENCEROFF                         0x80
#define RF69_OPMODE_LISTENON                             0x40
#define RF69_OPMODE_LISTENABORT                          0x20
#define RF69_OPMODE_MODE                                 0x1c
#define RF69_OPMODE_MODE_SLEEP                           0x00
#define RF69_OPMODE_MODE_STDBY                           0x04
#define RF69_OPMODE_MODE_FS                              0x08
#define RF69_OPMODE_MODE_TX                              0x0c
#define RF69_OPMODE_MODE_RX                              0x10
 
// RF69_REG_02_DATAMODUL
#define RF69_DATAMODUL_DATAMODE                          0x60
#define RF69_DATAMODUL_DATAMODE_PACKET                   0x00
#define RF69_DATAMODUL_DATAMODE_CONT_WITH_SYNC           0x40
#define RF69_DATAMODUL_DATAMODE_CONT_WITHOUT_SYNC        0x60
#define RF69_DATAMODUL_MODULATIONTYPE                    0x18
#define RF69_DATAMODUL_MODULATIONTYPE_FSK                0x00
#define RF69_DATAMODUL_MODULATIONTYPE_OOK                0x08
#define RF69_DATAMODUL_MODULATIONSHAPING                 0x03
#define RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE        0x00
#define RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0       0x01
#define RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_5       0x02
#define RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_3       0x03
#define RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE        0x00
#define RF69_DATAMODUL_MODULATIONSHAPING_OOK_BR          0x01
#define RF69_DATAMODUL_MODULATIONSHAPING_OOK_2BR         0x02
 
// RF69_REG_11_PALEVEL
#define RF69_PALEVEL_PA0ON                               0x80
#define RF69_PALEVEL_PA1ON                               0x40
#define RF69_PALEVEL_PA2ON                               0x20
#define RF69_PALEVEL_OUTPUTPOWER                         0x1f
 
// RF69_REG_23_RSSICONFIG
#define RF69_RSSICONFIG_RSSIDONE                         0x02
#define RF69_RSSICONFIG_RSSISTART                        0x01
 
// RF69_REG_25_DIOMAPPING1
#define RF69_DIOMAPPING1_DIO0MAPPING                     0xc0
#define RF69_DIOMAPPING1_DIO0MAPPING_00                  0x00
#define RF69_DIOMAPPING1_DIO0MAPPING_01                  0x40
#define RF69_DIOMAPPING1_DIO0MAPPING_10                  0x80
#define RF69_DIOMAPPING1_DIO0MAPPING_11                  0xc0
 
#define RF69_DIOMAPPING1_DIO1MAPPING                     0x30
#define RF69_DIOMAPPING1_DIO1MAPPING_00                  0x00
#define RF69_DIOMAPPING1_DIO1MAPPING_01                  0x10
#define RF69_DIOMAPPING1_DIO1MAPPING_10                  0x20
#define RF69_DIOMAPPING1_DIO1MAPPING_11                  0x30
 
#define RF69_DIOMAPPING1_DIO2MAPPING                     0x0c
#define RF69_DIOMAPPING1_DIO2MAPPING_00                  0x00
#define RF69_DIOMAPPING1_DIO2MAPPING_01                  0x04
#define RF69_DIOMAPPING1_DIO2MAPPING_10                  0x08
#define RF69_DIOMAPPING1_DIO2MAPPING_11                  0x0c
 
#define RF69_DIOMAPPING1_DIO3MAPPING                     0x03
#define RF69_DIOMAPPING1_DIO3MAPPING_00                  0x00
#define RF69_DIOMAPPING1_DIO3MAPPING_01                  0x01
#define RF69_DIOMAPPING1_DIO3MAPPING_10                  0x02
#define RF69_DIOMAPPING1_DIO3MAPPING_11                  0x03
 
// RF69_REG_26_DIOMAPPING2
#define RF69_DIOMAPPING2_DIO4MAPPING                     0xc0
#define RF69_DIOMAPPING2_DIO4MAPPING_00                  0x00
#define RF69_DIOMAPPING2_DIO4MAPPING_01                  0x40
#define RF69_DIOMAPPING2_DIO4MAPPING_10                  0x80
#define RF69_DIOMAPPING2_DIO4MAPPING_11                  0xc0
 
#define RF69_DIOMAPPING2_DIO5MAPPING                     0x30
#define RF69_DIOMAPPING2_DIO5MAPPING_00                  0x00
#define RF69_DIOMAPPING2_DIO5MAPPING_01                  0x10
#define RF69_DIOMAPPING2_DIO5MAPPING_10                  0x20
#define RF69_DIOMAPPING2_DIO5MAPPING_11                  0x30
 
#define RF69_DIOMAPPING2_CLKOUT                          0x07
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_                   0x00
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_2                  0x01
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_4                  0x02
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_8                  0x03
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_16                 0x04
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_32                 0x05
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_RC                 0x06
#define RF69_DIOMAPPING2_CLKOUT_FXOSC_OFF                0x07
 
// RF69_REG_27_IRQFLAGS1
#define RF69_IRQFLAGS1_MODEREADY                         0x80
#define RF69_IRQFLAGS1_RXREADY                           0x40
#define RF69_IRQFLAGS1_TXREADY                           0x20
#define RF69_IRQFLAGS1_PLLLOCK                           0x10
#define RF69_IRQFLAGS1_RSSI                              0x08
#define RF69_IRQFLAGS1_TIMEOUT                           0x04
#define RF69_IRQFLAGS1_AUTOMODE                          0x02
#define RF69_IRQFLAGS1_SYNADDRESSMATCH                   0x01
 
// RF69_REG_28_IRQFLAGS2
#define RF69_IRQFLAGS2_FIFOFULL                          0x80
#define RF69_IRQFLAGS2_FIFONOTEMPTY                      0x40
#define RF69_IRQFLAGS2_FIFOLEVEL                         0x20
#define RF69_IRQFLAGS2_FIFOOVERRUN                       0x10
#define RF69_IRQFLAGS2_PACKETSENT                        0x08
#define RF69_IRQFLAGS2_PAYLOADREADY                      0x04
#define RF69_IRQFLAGS2_CRCOK                             0x02
 
// RF69_REG_2E_SYNCCONFIG
#define RF69_SYNCCONFIG_SYNCON                           0x80
#define RF69_SYNCCONFIG_FIFOFILLCONDITION_MANUAL         0x40
#define RF69_SYNCCONFIG_SYNCSIZE                         0x38
#define RF69_SYNCCONFIG_SYNCSIZE_1                       0x00
#define RF69_SYNCCONFIG_SYNCSIZE_2                       0x08
#define RF69_SYNCCONFIG_SYNCSIZE_3                       0x10
#define RF69_SYNCCONFIG_SYNCSIZE_4                       0x18
#define RF69_SYNCCONFIG_SYNCSIZE_5                       0x20
#define RF69_SYNCCONFIG_SYNCSIZE_6                       0x28
#define RF69_SYNCCONFIG_SYNCSIZE_7                       0x30
#define RF69_SYNCCONFIG_SYNCSIZE_8                       0x38
#define RF69_SYNCCONFIG_SYNCSIZE_SYNCTOL                 0x07
 
// RF69_REG_37_PACKETCONFIG1
#define RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE         0x80
#define RF69_PACKETCONFIG1_DCFREE                        0x60
#define RF69_PACKETCONFIG1_DCFREE_NONE                   0x00
#define RF69_PACKETCONFIG1_DCFREE_MANCHESTER             0x20
#define RF69_PACKETCONFIG1_DCFREE_WHITENING              0x40
#define RF69_PACKETCONFIG1_DCFREE_RESERVED               0x60
#define RF69_PACKETCONFIG1_CRC_ON                        0x10
#define RF69_PACKETCONFIG1_CRCAUTOCLEAROFF               0x08
#define RF69_PACKETCONFIG1_ADDRESSFILTERING              0x06
#define RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE         0x00
#define RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE         0x02
#define RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC      0x04
#define RF69_PACKETCONFIG1_ADDRESSFILTERING_RESERVED     0x06
 
// RF69_REG_3B_AUTOMODES
#define RF69_AUTOMODE_ENTER_COND_NONE                    0x00
#define RF69_AUTOMODE_ENTER_COND_FIFO_NOT_EMPTY          0x20
#define RF69_AUTOMODE_ENTER_COND_FIFO_LEVEL              0x40
#define RF69_AUTOMODE_ENTER_COND_CRC_OK                  0x60
#define RF69_AUTOMODE_ENTER_COND_PAYLOAD_READY           0x80
#define RF69_AUTOMODE_ENTER_COND_SYNC_ADDRESS            0xa0
#define RF69_AUTOMODE_ENTER_COND_PACKET_SENT             0xc0
#define RF69_AUTOMODE_ENTER_COND_FIFO_EMPTY              0xe0
 
#define RF69_AUTOMODE_EXIT_COND_NONE                     0x00
#define RF69_AUTOMODE_EXIT_COND_FIFO_EMPTY               0x04
#define RF69_AUTOMODE_EXIT_COND_FIFO_LEVEL               0x08
#define RF69_AUTOMODE_EXIT_COND_CRC_OK                   0x0c
#define RF69_AUTOMODE_EXIT_COND_PAYLOAD_READY            0x10
#define RF69_AUTOMODE_EXIT_COND_SYNC_ADDRESS             0x14
#define RF69_AUTOMODE_EXIT_COND_PACKET_SENT              0x18
#define RF69_AUTOMODE_EXIT_COND_TIMEOUT                  0x1c
 
#define RF69_AUTOMODE_INTERMEDIATE_MODE_SLEEP            0x00
#define RF69_AUTOMODE_INTERMEDIATE_MODE_STDBY            0x01
#define RF69_AUTOMODE_INTERMEDIATE_MODE_RX               0x02
#define RF69_AUTOMODE_INTERMEDIATE_MODE_TX               0x03
 
// RF69_REG_3C_FIFOTHRESH
#define RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY        0x80
#define RF69_FIFOTHRESH_FIFOTHRESHOLD                    0x7f
 
// RF69_REG_3D_PACKETCONFIG2
#define RF69_PACKETCONFIG2_INTERPACKETRXDELAY            0xf0
#define RF69_PACKETCONFIG2_RESTARTRX                     0x04
#define RF69_PACKETCONFIG2_AUTORXRESTARTON               0x02
#define RF69_PACKETCONFIG2_AESON                         0x01
 
// RF69_REG_4E_TEMP1
#define RF69_TEMP1_TEMPMEASSTART                         0x08
#define RF69_TEMP1_TEMPMEASRUNNING                       0x04
 
// RF69_REG_5A_TESTPA1
#define RF69_TESTPA1_NORMAL                              0x55
#define RF69_TESTPA1_BOOST                               0x5d
 
// RF69_REG_5C_TESTPA2
#define RF69_TESTPA2_NORMAL                              0x70
#define RF69_TESTPA2_BOOST                               0x7c
 
// RF69_REG_6F_TESTDAGC
#define RF69_TESTDAGC_CONTINUOUSDAGC_NORMAL              0x00
#define RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAON  0x20
#define RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF 0x30
 
// Define this to include Serial printing in diagnostic routines
#define RF69_HAVE_SERIAL
 
#endif // RF69_REGISTERS_HPP
