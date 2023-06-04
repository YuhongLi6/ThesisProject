
/**
 * @file nrf52840_nrf24l01
 *
 * Class declaration for nrf52840_nrf24l01
 */
#ifndef __nrf52840_nrf24l01_H__
#define __nrf52840_nrf24l01_H__
#include <Arduino.h>

#define NRF52_RADIO_LIBRARY
#define DEFAULT_MAX_PAYLOAD_SIZE 32
#define ACTUAL_MAX_PAYLOAD_SIZE 127

typedef enum {
  /**
   * (0) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  -18 dBm | -6 dBm | -12 dBm
   */
  NRF_PA_MIN = 0,
  /**
   * (1) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  -12 dBm | 0 dBm | -4 dBm
   */
  NRF_PA_LOW,
  /**
   * (2) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  -6 dBm | 3 dBm | 1 dBm
   */
  NRF_PA_HIGH,
  /**
   * (3) represents:
   * nRF24L01 | Si24R1 with<br>lnaEnabled = 1 | Si24R1 with<br>lnaEnabled = 0
   * :-------:|:-----------------------------:|:----------------------------:
   *  0 dBm | 7 dBm | 4 dBm
   */
  NRF_PA_MAX,
  /**
   * (4) This should not be used and remains for backward compatibility.
   */
  NRF_PA_ERROR
} nrf_pa_dbm_e;

/**
 * @}
 * @defgroup Datarate datarate
 * How fast data moves through the air. Units are in bits per second (bps).
 * @see
 * - RF24::setDataRate()
 * - RF24::getDataRate()
 * @{
 */
typedef enum {
  /** (0) represents 1 Mbps */
  NRF_1MBPS = 0,
  /** (1) represents 2 Mbps */
  NRF_2MBPS,
  /** (2) represents 250 kbps */
  NRF_250KBPS
} nrf_datarate_e;

/**
 * @}
 * @defgroup CRCLength CRC length
 * The length of a CRC checksum that is used (if any). Cyclical Redundancy
 * Checking (CRC) is commonly used to ensure data integrity.
 * @see
 * - RF24::setCRCLength()
 * - RF24::getCRCLength()
 * - RF24::disableCRC()
 * @{
 */
typedef enum {
  /** (0) represents no CRC checksum is used */
  NRF_CRC_DISABLED = 0,
  /** (1) represents CRC 8 bit checksum is used */
  NRF_CRC_8,
  /** (2) represents CRC 16 bit checksum is used */
  NRF_CRC_16
} nrf_crclength_e;

class nrf_to_nrf {

public:
  nrf_to_nrf();

  uint8_t radioData[ACTUAL_MAX_PAYLOAD_SIZE + 2];

  bool begin();
  uint8_t sample_ed(void);
  bool available();
  bool available(uint8_t *pipe_num);
  void read(void *buf, uint8_t len);
  bool write(void *buf, uint8_t len, bool multicast = false);
  bool writeFast(const void *buf, uint8_t len, const bool multicast = 0);
  bool startWrite(void *buf, uint8_t len, bool multicast);
  bool writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
  void enableAckPayload();
  void disableAckPayload();
  void enableDynamicAck();
  void startListening(bool resetAddresses = true);
  void stopListening(bool setWritingPipe = true, bool resetAddresses = true);
  uint8_t getDynamicPayloadSize();
  bool isValid();
  bool isChipConnected();
  void setChannel(uint8_t channel);
  uint8_t getChannel();
  bool setDataRate(uint8_t speed);
  void setPALevel(uint8_t level, bool lnaEnable = true);
  uint8_t getPALevel();
  void setAutoAck(bool enable);
  void setAutoAck(uint8_t pipe, bool enable);
  void enableDynamicPayloads(uint8_t payloadSize = DEFAULT_MAX_PAYLOAD_SIZE);
  void disableDynamicPayloads();
  void setPayloadSize(uint8_t size);
  uint8_t getPayloadSize();
  void setCRCLength(nrf_crclength_e length);
  nrf_crclength_e getCRCLength();
  void disableCRC();
  void setRetries(uint8_t retryVar, uint8_t attempts);
  void openReadingPipe(uint8_t child, uint64_t address);
  void openWritingPipe(uint64_t address);
  void openReadingPipe(uint8_t child, const uint8_t *address);
  void openWritingPipe(const uint8_t *address);
  void setAddressWidth(uint8_t a_width);
  void printDetails();
  
  void powerUp();
  void powerDown();
  bool failureDetected;
  bool txStandBy();
  bool txStandBy(uint32_t timeout, bool startTx = 0);
  bool testCarrier();
  bool testRPD();
  uint8_t getARC();
  uint32_t addrConv32(uint32_t addr);

private:
  bool acksEnabled(uint8_t pipe);
  bool acksPerPipe[8];
  uint8_t retries;
  uint8_t retryDuration;
  uint8_t rxBuffer[ACTUAL_MAX_PAYLOAD_SIZE + 1];
  uint8_t ackBuffer[ACTUAL_MAX_PAYLOAD_SIZE + 1];
  uint8_t rxFifoAvailable;
  bool DPL;
  bool ackPayloadsEnabled;
  bool inRxMode;
  uint8_t staticPayloadSize;
  uint8_t ackPID;
  uint8_t ackPipe;
  bool lastTxResult;
  bool overWrite;
  uint32_t rxBase;
  uint32_t rxPrefix;
  uint32_t txBase;
  uint32_t txPrefix;
  bool radioConfigured;
  bool ackPayloadAvailable;
  uint8_t ackAvailablePipeNo;
  uint8_t lastPacketCounter = 0;
  uint8_t lastData = 0;
  bool dynamicAckEnabled = 1;
  uint8_t ARC;
  uint8_t addressWidth;
};

#endif //__nrf52840_nrf24l01_H__
