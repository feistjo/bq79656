#include "bq_comm.h"

#include <algorithm>
#include <cmath>

#include "Crc16.h"
#define serialdebug 1

#define CONTROL1_SEND_WAKE 0b00100000

Crc16 crc;

/**
 * @brief Starts the uart interface connected to the BQ79656 bridge chip
 *
 */

void BQ79656::BeginUart()
{
    uart_.addMemoryForRead(bq_uart_rx_buffer, 200);
    uart_.addMemoryForWrite(bq_uart_tx_buffer, 200);
    uart_.begin(BQ_UART_FREQ);  //, SERIAL_8N1_HALF_DUPLEX);  // BQ79656 uart interface is half duplex, but Teensy can't
                                // switch from write to read fast enough
}

/**
 * @brief Initializes communication with the BQ796XX stack
 *
 */

void BQ79656::Initialize()
{
    BeginUart();

    delay(50);
    // send commands to start/configure stack
    // todo
    WakePing();
    WakePing();  // two needed for some reason

    AutoAddressing(kNumSegments);
    // AutoAddressing(stack_size_);

    data_arr_[0] = 0b00001101;  // disable short comm timeout, long timeout action shutdown, long comm timeout 10 min
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::COMM_TIMEOUT_CONF, data_arr_);

    // set active cells for OV/UV
    uint8_t series_per_segment = kNumCellsSeries / kNumSegments;
    data_arr_[0] = 0b00001111 & (series_per_segment - 6);
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::ACTIVE_CELL, data_arr_);

    // enable TSREF
    data_arr_[0] = 0b00000001;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::CONTROL2, data_arr_);

    // set up all GPIOs as ADC + OTUT inputs
    data_arr_[0] = 0b00001001;
    data_arr_[1] = 0b00001001;
    data_arr_[2] = 0b00001001;
    data_arr_[3] = 0b00001001;
    Comm(RequestType::STACK_WRITE, 4, 0, RegisterAddress::GPIO_CONF1, data_arr_);

#ifdef serialdebug
    Serial.println("Start main ADC to run continuously");
#endif
    data_arr_[0] = 0b00000110;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::ADC_CTRL1, data_arr_);
    delay(10);

    data_arr_[0] = kFaultMask1;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::FAULT_MSK1, data_arr_);

    data_arr_[0] = kFaultMask2;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::FAULT_MSK2, data_arr_);

    // clear all faults
    data_arr_[0] = 0xFF;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::FAULT_RST1, data_arr_);
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::FAULT_RST2, data_arr_);
}

void BQ79656::StartOVUV()
{
    data_arr_[0] = 0b00000101;  // OVUV_GO, OVUV_MODE round robin
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::OVUV_CTRL, data_arr_);
}

void BQ79656::StartOTUT()
{
    data_arr_[0] = 0b00000101;  // OTUT_GO, mode=round robin
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::OTUT_CTRL, data_arr_);
}

/**
 * @brief Set the overvoltage, undervoltage, overtemperature, and undertemperature registers on the stack and starts the
 * protectors in round-robin mode
 *
 * @param ov_thresh
 * @param uv_thresh
 * @param ot_thresh
 * @param ut_thresh
 */
void BQ79656::SetProtectors(float ov_thresh, float uv_thresh, float ot_thresh, float ut_thresh)
{
    uint8_t ov_offset = (ov_thresh - 4.175f) / 0.025f;
    data_arr_[0] = 0b00111111 & (ov_offset + 0x22);
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::OV_THRESH, data_arr_);

    uint8_t uv_offset = (uv_thresh - 1.2f) / 0.050f;
    data_arr_[0] = 0b00111111 & uv_offset;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::UV_THRESH, data_arr_);

    uint8_t ut_offset = ((thermistor_.TemperatureToVoltage(ut_thresh) / 5.0f) * (100 / 2)) - 66;
    uint8_t ot_offset = ((thermistor_.TemperatureToVoltage(ot_thresh) / 5.0f) * 100) - 10;
    data_arr_[0] = (0b11100000 & (ut_offset << 5)) | (0b00011111 & ot_offset);
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::OTUT_THRESH, data_arr_);

    delay(5);

    StartOVUV();
    //  StartOTUT(); TODO
}

/**
 * @brief Communicates (reads/writes a register) with the BQBQ796XX daisychain
 *
 * @param req_type Request type, as defined in bq_comm.h
 * @param data_size The size of the data being sent, which is 1 for a read
 * @param dev_addr The address of the device being communicated with, ignored for stack or broadcast
 * @param reg_addr The address of the register being accessed
 * @param data The 1-8 byte payload, which for a read is one less than the number of bytes being read
 */

void BQ79656::Comm(
    RequestType req_type, byte data_size, byte dev_addr, RegisterAddress reg_addr, std::vector<byte> data)
{
    data_size -= 1;  // 0 means 1 byte
    bq_buffer_[0] =
        0b10000000 | static_cast<byte>(req_type) | (data_size & 0b00000111);  // command | req_type | data_size
    bool isStackOrBroad = (req_type == RequestType::STACK_READ) || (req_type == RequestType::STACK_WRITE)
                          || (req_type == RequestType::BROAD_READ) || (req_type == RequestType::BROAD_WRITE)
                          || (req_type == RequestType::BROAD_WRITE_REV);
    if (!isStackOrBroad)
    {
        bq_buffer_[1] = dev_addr;
    }
    bq_buffer_[1 + (!isStackOrBroad)] = static_cast<uint16_t>(reg_addr) >> 8;
    bq_buffer_[2 + (!isStackOrBroad)] = static_cast<uint16_t>(reg_addr) & 0xFF;
    for (int i = 0; i <= data_size; i++)
    {
        bq_buffer_[3 + i + (!isStackOrBroad)] = data[i];
    }
    uint16_t command_crc = crc.Modbus(
        bq_buffer_.data(), 0, 4 + data_size + (!isStackOrBroad));  // calculates the CRC, but the bytes are backwards
    bq_buffer_[4 + data_size + (!isStackOrBroad)] = command_crc & 0xFF;
    bq_buffer_[5 + data_size + (!isStackOrBroad)] = command_crc >> 8;

#ifdef serialdebug
    Serial.println("Command: ");
    for (int i = 0; i <= 5 + data_size + (!isStackOrBroad); i++)
    {
        Serial.print(bq_buffer_[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif

#ifdef serialdebug
    Serial.println("Sending frame:");
#endif

    uart_.write(bq_buffer_.data(), 5 + data_size + (!isStackOrBroad) + 1);
    delay(4);
}

/**
 * @brief Reads a register from the BQ79656(s) specified, ignoring any response
 *
 * @param req_type Request type, as defined in bq_comm.h
 * @param dev_addr The address of the device being communicated with, ignored for stack or broadcast
 * @param reg_addr The address of the register being accessed
 * @param resp_size The size in bytes of the expected response
 */

void BQ79656::DummyReadReg(RequestType req_type, byte dev_addr, RegisterAddress reg_addr, byte resp_size)
{
    // bqComm(BQ_SINGLE_WRITE, 1, 0, BRIDGE_FAULT_RST, data);
    resp_size -= 1;  // 0 means 1 byte
    data_arr_[0] = resp_size;
    Comm(req_type, 1, dev_addr, reg_addr, data_arr_);
    delay(1);
    uart_.clear();
    /*
    bq_buffer_data_length_ = resp_size + 7;
    delay(10);
    if (digitalRead(spi_rdy_pin_bq))
    {
      for (int j = 0; j < bq_buffer_data_length_; j++)
      {
        bq_response_buffers_[0][j] = 0xFF;
      }
      SPI.transfer(bq_response_buffers_, bq_buffer_data_length_);
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

/**
 * @brief Reads a register from the BQBQ796XX daisychain
 *
 * @param req_type The request type, as defined in bq_comm.h
 * @param dev_addr The address of the device being communicated with
 * @param reg_addr The address of the register being accessed
 * @param resp_size The number of bytes being read
 * @return uint8_t* An array of response buffers
 */

std::vector<std::vector<uint8_t>> BQ79656::ReadReg(RequestType req_type,
                                                   byte dev_addr,
                                                   RegisterAddress reg_addr,
                                                   byte resp_size)
{
    resp_size -= 1;  // 0 means 1 byte
    data_arr_[0] = resp_size;
    Comm(req_type, 1, dev_addr, reg_addr, data_arr_);

    bq_buffer_data_length_ = resp_size + 7;

    int numExpectedResponses = 1;
    if (req_type == RequestType::STACK_READ)
    {
        numExpectedResponses = stack_size_;
    }
    if (req_type == RequestType::BROAD_READ)
    {
        numExpectedResponses = stack_size_ + 1;
    }

    for (int i = 0; i < numExpectedResponses; i++)
    {
#ifdef serialdebug
        Serial.println("Waiting for data");
#endif

        while (!uart_.available())
            ;

#ifdef serialdebug
        Serial.println("Reading data");
#endif

        uart_.readBytes(bq_response_buffers_[i].data(), bq_buffer_data_length_);

#ifdef serialdebug
        for (int j = 0; j < bq_buffer_data_length_; j++)
        {
            Serial.print(bq_response_buffers_[i][j], HEX);
            Serial.print(" ");
        }
        Serial.println();
#endif
    }
    return bq_response_buffers_;
}

/**
 * @brief Starts the BQ chips and auto-addresses the stack, as defined in section 4 of the BQ79616-Q1 software design
 * reference
 *
 * @param numDevices The number of devices in the stack, defaults to kNumSegments
 */

void BQ79656::AutoAddressing(byte numDevices)
{
    stack_size_ = numDevices;
    data_arr_[0] = 0x00;

    // Step 1: dummy broadcast write 0x00 to OTP_ECC_TEST (sync up internal DLL)
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::OTP_ECC_TEST, data_arr_);

    // Step 2: broadcast write 0x01 to CONTROL to enable auto addressing
    data_arr_[0] = 0x01;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::CONTROL1, data_arr_);

    // Step 3: broadcast write consecutively to DIR0_ADDR = 0, 1, 2, 3, ...
    for (byte i = 0; i <= numDevices; i++)
    {
        data_arr_[0] = i;
        Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::DIR0_ADDR, data_arr_);
    }

    // Step 4: broadcast write 0x02 to COMM_CTRL to set everything as stack device
    data_arr_[0] = 0x02;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::COMM_CTRL, data_arr_);

    // Step 8: single device write to set base and top of stack
    data_arr_[0] = 0x00;  // not stack device
    Comm(RequestType::SINGLE_WRITE, 1, 0, RegisterAddress::COMM_CTRL, data_arr_);
    data_arr_[0] = 0x03;  // stack and top
    Comm(RequestType::SINGLE_WRITE, 1, numDevices, RegisterAddress::COMM_CTRL, data_arr_);

    // Step 9: dummy broadcast read OTP_ECC_TEST (sync up internal DLL)
    DummyReadReg(RequestType::BROAD_READ, 0, RegisterAddress::OTP_ECC_TEST, 1);

    // clear all faults
    data_arr_[0] = 0xFF;
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::FAULT_RST1, data_arr_);
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::FAULT_RST2, data_arr_);

    // Read fault summary register
    // ReadReg(RequestType::STACK_READ, 0, RegisterAddress::FAULT_SUMMARY, 1);

    // stack read address 0x306 to verify addresses
    // ReadReg(RequestType::BROAD_READ, 0, RegisterAddress::DIR0_ADDR, 1);
}

/**
 * @brief Starts balancing with timers set at 300 seconds and stop voltage at 4V
 *
 */
void BQ79656::ProcessBalancingSimple(uint32_t current_millis)
{
    data_arr_[0] = kFaultMask1 | kFaultMask1OverVoltage;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::FAULT_MSK1, data_arr_);
    static uint32_t last_millis = 0;
    if (current_millis - last_millis < 300000 && last_millis != 0)  // don't restart balancing if already running
    {
        return;
    }
    last_millis = current_millis;

    uint8_t seriesPerSegment = kNumCellsSeries / kNumSegments;
    // set up balancing time control registers to 300s (0x04)
    std::vector<byte> balTimes(seriesPerSegment / 2, 0x04);
    Comm(RequestType::STACK_WRITE,
         seriesPerSegment / 2,
         0,
         static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) + 1 - seriesPerSegment),
         balTimes);  // can only do up to 8 in one command
    Comm(RequestType::STACK_WRITE,
         seriesPerSegment / 2,
         0,
         static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) + (seriesPerSegment / 2) + 1
                                      - seriesPerSegment),
         balTimes);

    // set balancing end voltage to 4V (max)
    data_arr_[0] = 0x3F;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::VCB_DONE_THRESH, data_arr_);

    StartOVUV();

    // start balancing with FLTSTOP_EN to stop on fault, OTCB_EN to pause on overtemp, AUTO_BAL to automatically cycle
    // between even/odd
    data_arr_[0] = 0b00110011;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::BAL_CTRL2, data_arr_);
}

/**
 * @brief Runs a round of balancing on all segments in the stack
 *
 * @param voltages A vector<float> of the entire stack's voltages
 * @param max_charge_voltage The max charge voltage to limit the cells to
 */
void BQ79656::ProcessBalancing(std::vector<float> voltages, float max_charge_voltage)
{
    data_arr_[0] = kFaultMask1 | kFaultMask1OverVoltage;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::FAULT_MSK1, data_arr_);
    float min_voltage = *std::min_element(voltages.begin(), voltages.end());
    float max_voltage = *std::max_element(voltages.begin(), voltages.end());
    static constexpr float balancing_threshold{0.01};
    if (max_voltage - min_voltage < balancing_threshold && max_voltage <= max_charge_voltage)
    {
        return;
    }
    // Find all cell voltages above threshold over min voltage, set balance timers for whichever is worse of even/odd
    // for each logical segment
    int seriesPerSegment = kNumCellsSeries / kNumSegments;
    for (int segment = 0; segment < kNumSegments; segment++)
    {
        std::vector<float>::iterator max_segment_voltage_iter =
            std::max_element(voltages.begin() + seriesPerSegment, voltages.begin() + (2 * seriesPerSegment));
        float max_segment_voltage = *max_segment_voltage_iter;
        if (max_segment_voltage - min_voltage >= balancing_threshold || max_segment_voltage > max_charge_voltage)
        {
            SetAllDataArrValues(0);
            int message = 0;  // num_messages = std::round((seriesPerSegment / 8.0f) + 0.5);
            for (int cell = (max_segment_voltage_iter - (voltages.begin() + seriesPerSegment))
                            % 2;  // 0 if even is worse, 1 if odd is worse
                 cell < seriesPerSegment;
                 cell = cell + 2)
            {
                data_arr_[8 - (cell % 8)] =
                    voltages[cell + (segment * seriesPerSegment)] - min_voltage >= balancing_threshold
                            || voltages[cell + (segment * seriesPerSegment)] > max_charge_voltage
                        ? 0x01
                        : 0x00;  // 10s if balancing needed

                if (cell % 8 == 0 || cell == seriesPerSegment - 1)  // if data_arr_ full, send message
                {
                    const int cells_in_message = cell % 8 == 0 ? 8 : cell % 8;
                    Comm(RequestType::SINGLE_WRITE,
                         cells_in_message,
                         segment,
                         static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) + 1
                                                      - ((message * 8) + cells_in_message)),
                         data_arr_);  // data size = cells in message,
                    SetAllDataArrValues(0);
                    message++;
                }
            }
        }
    }

    // set balancing end voltage
    data_arr_[0] = 0x3F;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::VCB_DONE_THRESH, data_arr_);
}

/**
 * @brief Stops balancing and re-enables OV fault detection, clearing OV faults. This function should be called when
 * charging stops.
 *
 */
void BQ79656::StopBalancing()
{
    uint8_t seriesPerSegment = kNumCellsSeries / kNumSegments;
    // set up balancing time control registers to 0s (0x0)
    std::vector<byte> balTimes(seriesPerSegment / 2, 0x04);
    Comm(RequestType::STACK_WRITE,
         seriesPerSegment / 2,
         0,
         static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) + 1 - seriesPerSegment),
         balTimes);  // can only do up to 8 in one command
    Comm(RequestType::STACK_WRITE,
         seriesPerSegment / 2,
         0,
         static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) + (seriesPerSegment / 2) + 1
                                      - seriesPerSegment),
         balTimes);
    data_arr_[0] = 0b00110011;  // write BAL_GO to process registers
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::BAL_CTRL2, data_arr_);

    // clear OV faults
    data_arr_[0] = 0b00001000;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::FAULT_RST1, data_arr_);

    // reset fault mask 1 to re-enable OV faults
    data_arr_[0] = kFaultMask1;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::FAULT_MSK1, data_arr_);
}

void BQ79656::SetAllDataArrValues(byte value)
{
    for (int i = 0; i < data_arr_.size(); i++)
    {
        data_arr_[i] = value;
    }
}

void BQ79656::SetStackSize(int newSize) { stack_size_ = newSize; }

bool BQ79656::VerifyCRC(std::vector<uint8_t> buf) { return crc.Modbus(buf.data(), 0, bq_buffer_data_length_) == 0; }

std::vector<uint8_t> BQ79656::GetBuf() { return bq_buffer_; }

int &BQ79656::GetDataLen() { return bq_buffer_data_length_; }

/**
 * @brief Runs the integrated open wire check on the VC pins of the BQ79656
 * Note: Main ADC must be running in continuous mode before this funcion is called
 *
 * @return true if there is an open wire fault
 * @return false if there is no open wire fault
 */
bool BQ79656::RunOpenWireCheck()
{
    // Before starting the open wire detection, the host ensures
    // The Main ADC is running in continuous mode

    // Configure the open wire detection threshold through DIAG_COMP_CTRL2[OW_THR3:0]
    data_arr_[0] = 0x0 | 6;  // 6*300mv+500mv=2.3v threshold
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::DIAG_COMP_CTRL2, data_arr_);

    // To start the open wire comparison
    // Turn on the VC pins current sink or source through DIAG_COMP_CTRL3[OW_SNK1:0]
    data_arr_[0] = 0b00010000;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::DIAG_COMP_CTRL3, data_arr_);

    // Wait for dV/dt time to deplete capacitors
    delay(3);  // depletes 0.47uf at 380ua minimum, 808V/s, will deplete to at most 1.776V

    // For VC open wire detection, select DIAG_COMP_CTRL3[COMP_ADC_SEL2:0] = OW VC check (0b010) and set COMP_ADC_GO=1
    data_arr_[0] = 0b00010101;  // leave current sinks on
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::DIAG_COMP_CTRL3, data_arr_);

    // Device runs comparisons
    // Wait for comparison completed, ADC_STAT2[DRDY_VCOW]=1
    std::vector<bool> complete_segments(kNumSegments, false);
    bool complete = false;
    while (!complete)
    {
        ReadReg(RequestType::STACK_READ, 0, RegisterAddress::ADC_STAT2, 1);
        complete = true;
        for (int i = 0; i < kNumSegments; i++)
        {
            complete_segments[i] = complete_segments[i] | (bq_response_buffers_[stack_size_ - i - 1][0] & 0b00001000);
            complete &= complete_segments[i];
        }
    }

    // Host then turns of all current sinks and sources through DIAG_COMP_CTRL3[OW_SNK1:0]
    data_arr_[0] = 0b00000000;
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::DIAG_COMP_CTRL3, data_arr_);

    // Host checks the FAULT_COMP_VCOW1/2 registers for comparison result
    // Just check fault summary
    // May not be needed, can return void and let nfault trigger interrupt
    ReadReg(RequestType::STACK_READ, 0, RegisterAddress::FAULT_SUMMARY, 1);
    bool ow_fault = false;
    for (int i = 0; i < kNumSegments; i++)
    {
        ow_fault |= bq_response_buffers_[stack_size_ - i - 1][0] & 0b01000000;
    }
    return ow_fault;
}

/**
 * @brief Reads the voltages from the battery. Note: ADC must be running beforehand!
 *
 * @param voltages A vector<float> to fill in with the newly read voltages
 */
void BQ79656::GetVoltages(std::vector<float> &voltages)
{
    data_arr_[0] = 0b01000000;  // CB_PAUSE, none of the other values are read until BAL_GO is set to 1
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::BAL_CTRL2, data_arr_);
    //  read voltages from battery
    int seriesPerSegment = kNumCellsSeries / kNumSegments;
    ReadReg(
        RequestType::STACK_READ,
        0,
        static_cast<RegisterAddress>((static_cast<uint16_t>(RegisterAddress::VCELL1_LO) + 1) - (seriesPerSegment * 2)),
        seriesPerSegment * 2);

    // fill in num_series voltages to array
    for (int i = 0; i < stack_size_; i++)
    {
        for (int j = 0; j < seriesPerSegment; j++)
        {
            int16_t voltage;
            ((uint8_t *)&voltage)[1] = bq_response_buffers_[stack_size_ - i - 1][(2 * j) + 4];
            ((uint8_t *)&voltage)[0] = bq_response_buffers_[stack_size_ - i - 1][(2 * j) + 5];
            voltages[(i * seriesPerSegment) + j] = voltage * BQ_V_LSB_ADC;
        }
    }

    data_arr_[0] = 0b00000000;  // CB_PAUSE=0 to resume, none of the other values are read until BAL_GO is set to 1
    Comm(RequestType::STACK_WRITE, 1, 0, RegisterAddress::BAL_CTRL2, data_arr_);
    return;
}

/**
 * @brief Reads the temperatures from the battery
 *
 * @param temps A vector<float> to fill in with the newly read temperatures
 */
void BQ79656::GetTemps(std::vector<float> &temps)
{
    // read temps from battery
    int thermoPerSegment = kNumThermistors / kNumSegments;
    ReadReg(RequestType::STACK_READ,
            0,
            static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::GPIO1_HI) - 1),
            thermoPerSegment * 2);

    // fill in kNumThermistors temperatures to array
    for (int i = 0; i < stack_size_; i++)
    {
        for (int j = 0; j < thermoPerSegment; j++)
        {
            int16_t temp;
            ((uint8_t *)&temp)[0] = bq_response_buffers_[stack_size_ - i - 1][(2 * j) + 4];
            ((uint8_t *)&temp)[1] = bq_response_buffers_[stack_size_ - i - 1][(2 * j) + 5];
            temps[(i * thermoPerSegment) + j] = thermistor_.VoltageToTemperature(temp * BQ_V_LSB_GPIO);
        }
    }
    return;
}

void BQ79656::EnableUartDebug()
{
    // void bqComm(byte req_type, byte data_size, byte dev_addr, uint16_t reg_addr, byte* data);
    std::vector<byte> byteArr{0b00001110};
    Comm(RequestType::BROAD_WRITE, 1, 0, RegisterAddress::DEBUG_COMM_CTRL1, byteArr);
}

/**
 * @brief Reads the current from the battery
 *
 * @param current A vector<float> to place the newly read current into
 */
void BQ79656::GetCurrent(std::vector<float> &current)
{
    // read current from battery
    std::vector<std::vector<uint8_t>> resp = ReadReg(RequestType::SINGLE_READ, 1, RegisterAddress::CURRENT_HI, 3);
    int32_t curr;
    ((uint8_t *)&curr)[2] = bq_response_buffers_[0][4];
    ((uint8_t *)&curr)[1] = bq_response_buffers_[0][5];
    ((uint8_t *)&curr)[0] = bq_response_buffers_[0][6];
    curr = curr << 8;
    curr = curr >> 8;
    current[0] = (curr / kShuntResistance) * BQ_CURR_LSB;

    return;
}

/**
 * @brief Sends a 2.5ms active-low wake ping to the BQ79656 bridge
 *
 */

void BQ79656::WakePing()
{
    // Output a pulse of low on RX for ~2.5ms to wake chip
    uart_.end();
    pinMode(tx_pin_, OUTPUT);
    digitalWrite(tx_pin_, LOW);
    delayMicroseconds(2500);
    digitalWrite(tx_pin_, HIGH);
    BeginUart();
    delayMicroseconds(
        (10000 + 600)
        * kNumSegments);  //(10ms shutdown to active transition + 600us propogation of wake) * number_of_devices
}

/**
 * @brief Sends a 15 bit period active-low comm clear ping to the BQ79656 bridge
 *
 */

void BQ79656::CommClear()
{
    // Output a pulse of low on RX for 15 bit periods to wake chip
    uart_.end();
    pinMode(tx_pin_, OUTPUT);
    digitalWrite(tx_pin_, LOW);
    delayMicroseconds(15 * 1000000 / BQ_UART_FREQ);
    digitalWrite(tx_pin_, HIGH);
    BeginUart();
    // delayMicroseconds((10000 + 600) * num_segments); //(10ms shutdown to active transition + 600us propogation of
    // wake) * number_of_devices
}
