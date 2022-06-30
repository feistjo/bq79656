#include "bq_comm.h"
#include "Crc16.h"
#define serialdebug 1

uint8_t bqBuf[176];
uint8_t bqRespBufs[num_segments + 1][176];
int bqBufDataLen = 0;
int stackSize = 0;

Crc16 crc;
#define CONTROL1_SEND_WAKE 0b00100000

void BQ79656::BeginUart()
{
  uart_.begin(BQ_UART_FREQ, SERIAL_8N1_HALF_DUPLEX); // BQ79656 uart interface is half duplex
}

/*
 * Initializes communication with the BQ796XX stack
 */
void BQ79656::Initialize()
{
  BeginUart();

  // send commands to start/configure stack
  // todo
  WakePing();
  // byte byteArr[] = {CONTROL1_SEND_WAKE};
  // Comm(BQ79656::RequestType::SINGLE_WRITE, 1, 0, RegisterAddress::CONTROL1, byteArr);
  // delay(11 * 15); // at least 10ms+600us per chip
  //  byteArr[0] = 0x00;
  //  bqComm(BQ_BROAD_WRITE, 1, 0, STACK_COMM_TIMEOUT_CONF, byteArr);
  AutoAddressing(num_segments);

  /*//enable NFAULT and FCOMM, is enabled by default
  byte[] byteArr = {0b00010100};
  bqComm(BQ_SINGLE_WRITE, 1, 0, BRIDGE_DEV_CONF1, byteArr);*/

  // enable FCOMM_EN of stack devices, unnecessary because enabled by default

  // Broadcast write sleep time to prevent sleeping
}

/*
 * Communicates (reads/writes a register) with the BQBQ796XX daisychain
 * req_type is the request type, as defined in bq_comm.h
 * data_size is the size of the data being sent, which is 1 for a read
 * dev_addr is the address of the device being communicated with, ignored for stack or broadcast
 * reg_addr is the address of the register being accessed
 * data is the 1-8 byte payload, which for a read is one less than the number of bytes being read
 * Returns an array of response buffers
 */
void BQ79656::Comm(RequestType req_type, byte data_size, byte dev_addr, RegisterAddress reg_addr, byte *data)
{
  data_size -= 1;                                                                 // 0 means 1 byte
  bqBuf[0] = 0b10000000 | static_cast<byte>(req_type) | (data_size & 0b00000111); // command | req_type | data_size
  bool isStackOrBroad = (req_type == RequestType::STACK_READ) || (req_type == RequestType::STACK_WRITE) ||
                        (req_type == RequestType::BROAD_READ) || (req_type == RequestType::BROAD_WRITE) ||
                        (req_type == RequestType::BROAD_WRITE_REV);
  if (!isStackOrBroad)
  {
    bqBuf[1] = dev_addr;
  }
  bqBuf[1 + (!isStackOrBroad)] = static_cast<uint16_t>(reg_addr) >> 8;
  bqBuf[2 + (!isStackOrBroad)] = static_cast<uint16_t>(reg_addr) & 0xFF;
  for (int i = 0; i <= data_size; i++)
  {
    bqBuf[3 + i + (!isStackOrBroad)] = data[i];
  }
  uint16_t command_crc = crc.Modbus(bqBuf, 0, isStackOrBroad ? 4 : 5); // calculates the CRC, but the bytes are backwards
  bqBuf[4 + data_size + (!isStackOrBroad)] = command_crc & 0xFF;
  bqBuf[5 + data_size + (!isStackOrBroad)] = command_crc >> 8;

#if serialdebug
  Serial.println("Command: ");
  for (int i = 0; i <= 5 + data_size + (!isStackOrBroad); i++)
  {
    Serial.print(bqBuf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

#if serialdebug
  Serial.println("Sending frame:");
#endif

  for (int i = 0; i <= 5 + data_size + (!isStackOrBroad); i++)
  {
    uart_.write(bqBuf[i]);
  }
}

void BQ79656::DummyReadReg(RequestType req_type, byte dev_addr, RegisterAddress reg_addr, byte resp_size)
{
  byte data[] = {0xFF};
  // bqComm(BQ_SINGLE_WRITE, 1, 0, BRIDGE_FAULT_RST, data);
  resp_size -= 1; // 0 means 1 byte
  data[0] = resp_size;
  Comm(req_type, 1, dev_addr, reg_addr, data);
  delay(1);
  uart_.clear();
  /*
  bqBufDataLen = resp_size + 7;
  delay(10);
  if (digitalRead(spi_rdy_pin_bq))
  {
    for (int j = 0; j < bqBufDataLen; j++)
    {
      bqRespBufs[0][j] = 0xFF;
    }
    SPI.transfer(bqRespBufs, bqBufDataLen);
  }
  else
  {
    Serial.println("Comm clear");
    CommClear();
  } */
  /*bqCommClear();
  //data[0] = {0};
  Serial.println("Fault summary");
  bqReadReg(BQ_SINGLE_READ, 0, BRIDGE_FAULT_SUMMARY, 1);
  Serial.println("Fault comm1");
  bqReadReg(BQ_SINGLE_READ, 0, BRIDGE_FAULT_COMM1, 1);
  Serial.println("Fault comm2");
  bqReadReg(BQ_SINGLE_READ, 0, BRIDGE_FAULT_COMM2, 1);
  Serial.println("Fault reg");
  bqReadReg(BQ_SINGLE_READ, 0, BRIDGE_FAULT_REG, 1);
  Serial.println("Fault sys");
  bqReadReg(BQ_SINGLE_READ, 0, BRIDGE_FAULT_SYS, 1);*/
}

/*
 * Reads a register from the BQBQ796XX daisychain
 * req_type is the request type, as defined in bq_comm.h
 * dev_addr is the address of the device being communicated with
 * reg_addr is the address of the register being accessed
 * resp_size is the number of bytes being read
 * Returns an array of response buffers
 */
uint8_t *BQ79656::ReadReg(RequestType req_type, byte dev_addr, RegisterAddress reg_addr, byte resp_size)
{
  resp_size -= 1; // 0 means 1 byte
  byte data[] = {resp_size};
  Comm(req_type, 1, dev_addr, reg_addr, data);

  bqBufDataLen = resp_size + 7;

  int numExpectedResponses = 1;
  if (req_type == RequestType::STACK_READ)
  {
    numExpectedResponses = stackSize;
  }
  if (req_type == RequestType::BROAD_READ)
  {
    numExpectedResponses = stackSize + 1;
  }

  for (int i = 0; i < numExpectedResponses; i++)
  {
#if serialdebug
    Serial.println("Waiting for data");
#endif

    while (!uart_.available())
      ;

#if serialdebug
    Serial.println("Reading data");
#endif

    uart_.readBytes(bqRespBufs[i], bqBufDataLen);

#if serialdebug
    for (int j = 0; j < bqBufDataLen; j++)
    {
      Serial.print(bqRespBufs[i][j], HEX);
      Serial.print(" ");
    }
    Serial.println();
#endif
  }
  return (uint8_t *)bqRespBufs; //(uint8_t**)bqRespBufs;
}

/*
 * Starts the BQ chips and auto-addresses the stack, as defined in section 4 of the BQ79616-Q1 software design reference
 */
void BQ79656::AutoAddressing(byte numDevices)
{
  stackSize = numDevices;
  byte byteArr[] = {0x00};

  // Step 1: dummy broadcast write 0x00 to OTP_ECC_TEST (sync up internal DLL)
  Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::OTP_ECC_TEST, byteArr);

  // Step 2: broadcast write 0x01 to CONTROL to enable auto addressing
  byteArr[0] = 0x01;
  Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::CONTROL1, byteArr);

  // Step 3: broadcast write consecutively to DIR0_ADDR = 0, 1, 2, 3, ...
  for (byte i = 0; i <= numDevices; i++)
  {
    byteArr[0] = i;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::DIR0_ADDR, byteArr);
  }

  // Step 4: broadcast write 0x02 to COMM_CTRL to set everything as stack device
  byteArr[0] = 0x02;
  Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::COMM_CTRL, byteArr);

  // Step 8: single device write to set base and top of stack
  byteArr[0] = 0x00;
  Comm(RequestType::SINGLE_WRITE, 1, 0, RegisterAddress::COMM_CTRL, byteArr);
  byteArr[0] = 0x03;
  Comm(RequestType::SINGLE_WRITE, 1, numDevices, RegisterAddress::COMM_CTRL, byteArr);

  // Step 9: dummy broadcast read OTP_ECC_TEST (sync up internal DLL)
  DummyReadReg(RequestType::BROAD_READ, 0, RegisterAddress::OTP_ECC_TEST, 1);

  // stack read address 0x306 to verify addresses
  // ReadReg(RequestType::STACK_READ, 0, RegisterAddress::DIR0_ADDR, 1);
}

// Starts balancing with timers set at 300 seconds and stop voltage at 4V
// Only works with cells per segment <= 14! (due to single stack write limitations)
void BQ79656::StartBalancingSimple()
{
  int seriesPerSegment = num_series / num_segments;
  // set up balancing time control registers to 300s (0x04)
  byte balTimes[seriesPerSegment] = {0x04};
  Comm(RequestType::STACK_WRITE, seriesPerSegment, 0, static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) + 1 - seriesPerSegment), balTimes);

  // set balancing end voltage
  byte vcbDone[] = {0x3F};
  Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::VCB_DONE_THRESH, vcbDone);

  // start balancing with FLTSTOP_EN to stop on fault, OTCB_EN to pause on overtemp, AUTO_BAL to automatically cycle between even/odd
  byte startBal[] = {0b00110011};
  Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::BAL_CTRL2, startBal);
}

/* void BQ79656::RunBalanceRound(double* voltages) {
  int seriesPerSegment = num_series / num_segments;


  for (int i = 1; i <= stackSize; i++) {

  }
} */

void BQ79656::SetStackSize(int newSize)
{
  stackSize = newSize;
}

/*uint16_t calculateCRC() {
  return crc.Modbus(txBuf, 0, txDataLen);
}*/

bool BQ79656::verifyCRC(uint8_t *buf)
{
  return crc.Modbus(buf, 0, bqBufDataLen) == 0;
}

uint8_t *BQ79656::GetBuf()
{
  return bqBuf;
}

/* uint8_t** BQ79656::GetRespBufs() {
  return (uint8_t**)bqRespBufs;
} */

int *BQ79656::GetDataLen()
{
  return &bqBufDataLen;
}

void BQ79656::GetVoltages(float *voltages)
{
  // read voltages from battery
  int seriesPerSegment = num_series / num_segments;
  ReadReg(RequestType::STACK_READ, 0, static_cast<RegisterAddress>((static_cast<uint16_t>(RegisterAddress::VCELL1_LO) + 1) - (seriesPerSegment * 2)), seriesPerSegment * 2);

  // fill in num_series voltages to array
  for (int i = 1; i <= stackSize; i++)
  {
    for (int j = 0; j < seriesPerSegment; j++)
    {
      int16_t voltage[1];
      ((uint8_t *)voltage)[0] = bqRespBufs[i][(2 * j) + 4];
      ((uint8_t *)voltage)[1] = bqRespBufs[i][(2 * j) + 5];
      voltages[((i - 1) * seriesPerSegment) + j] = voltage[0] * (190.73 * 0.000001); // Result * V_LSB_ADC
    }
  }
  return;
}

void BQ79656::GetTemps(float *temps)
{
  // read temps from battery
  int thermoPerSegment = num_series / num_segments;
  ReadReg(RequestType::STACK_READ, 0, static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::GPIO1_HI) - 1), thermoPerSegment * 2);

  // fill in num_thermo temperatures to array
  for (int i = 1; i <= stackSize; i++)
  {
    for (int j = 0; j < thermoPerSegment; j++)
    {
      int16_t temp[1];
      ((uint8_t *)temp)[0] = bqRespBufs[i][(2 * j) + 4];
      ((uint8_t *)temp)[1] = bqRespBufs[i][(2 * j) + 5];
      temps[((i - 1) * thermoPerSegment) + j] = temp[0] * (152.59 * 0.000001); // Result * V_LSB_GPIO to get voltage
      // TODO: calculate temp from voltage
    }
  }
  return;
}

void BQ79656::EnableUartDebug()
{
  // void bqComm(byte req_type, byte data_size, byte dev_addr, uint16_t reg_addr, byte* data);
  byte byteArr[] = {0b00001110};
  Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::DEBUG_COMM_CTRL1, byteArr);
}

void BQ79656::GetCurrent(float *current)
{
  // read current from battery

  return;
}

// Convert a raw voltage measurement into a temperature
/* double BQ79656::RawToTemp(int raw) {
  double volts = raw * BQ_THERM_LSB;
  return;
} */

// Convert a voltage measurement into a current
/* double BQ79656::VoltageToCurrent(int raw) {
  double volts = raw * BQ_CURR_LSB;
  return volts / SHUNT_RESISTANCE;
} */

void BQ79656::WakePing()
{
  // Output a pulse of low on RX for ~2.5ms to wake chip
  uart_.end();
  pinMode(tx_pin_, OUTPUT);
  digitalWrite(tx_pin_, LOW);
  delayMicroseconds(2500);
  digitalWrite(tx_pin_, HIGH);
  BeginUart();
  delayMicroseconds((10000 + 600) * num_segments); //(10ms shutdown to active transition + 600us propogation of wake) * number_of_devices
}

void BQ79656::CommClear()
{
  // Output one byte of 0 to reset comm interface
  digitalWrite(cs_bq, LOW);
  SPI.beginTransaction(SPISettings(BQ_SPI_FREQ, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(cs_bq, HIGH);
}
