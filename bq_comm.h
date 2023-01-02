#include <Arduino.h>
#include <SPI.h>
#include <stdbool.h>

#include <vector>

#define BQ_SPI_FREQ 6000000
#define BQ_UART_FREQ 1000000
#define BQ_THERM_LSB 0.00015259   // Vlsb_gpio = 152.59uV/lsb
#define BQ_CURR_LSB 0.0000000149  // 14.9 nv
#define BQ_V_LSB_ADC (190.73 * 0.000001)
#define BQ_V_LSB_GPIO (152.59 * 0.000001)

#define num_series 140
#define num_thermo 112
#define num_segments 14          // logical segments (BQ79656-Q1 chips)
#define SHUNT_RESISTANCE 0.0001  // 100 uohm

static uint8_t bq_uart_rx_buffer[200] = {0};
static uint8_t bq_uart_tx_buffer[200] = {0};

class BQ79656
{
public:
    BQ79656(HardwareSerial& uart, uint8_t tx_pin) : uart_(uart), tx_pin_(tx_pin), data_arr_(8, 0) {}

    void Initialize();

    void SetProtectors(float ov_thresh, float uv_thresh, float ot_thresh, float ut_thresh);

    void AutoAddressing(byte numDevices = num_segments);

    void ProcessBalancing(std::vector<float> voltages);

    void StartBalancingSimple();

    void GetVoltages(std::vector<float>& voltages);
    void GetTemps(std::vector<float>& temperatures);
    void GetCurrent(std::vector<float>& current);

#ifndef BQTEST
private:
#endif
    enum class RequestType : byte
    {
        SINGLE_READ = 0b00000000,      // single device read
        SINGLE_WRITE = 0b00010000,     // single device write
        STACK_READ = 0b00100000,       // stack read
        STACK_WRITE = 0b00110000,      // stack write
        BROAD_READ = 0b01000000,       // broadcast read
        BROAD_WRITE = 0b01010000,      // broadcast write
        BROAD_WRITE_REV = 0b01100000,  // broadcast write reverse
    };

    enum class
        RegisterAddress : uint16_t  // https://docs.google.com/spreadsheets/d/1GyFOFbGB9zVR0eIfJ3rLILp069PXWmpAPe-eZRBfAMY/edit#gid=0
    {
        DIR0_ADDR_OTP = 0x0,
        DIR1_ADDR_OTP = 0x1,
        DEV_CONF = 0x2,
        ACTIVE_CELL = 0x3,
        OTP_SPARE15 = 0x4,
        BBVC_POSN1 = 0x5,
        BBVC_POSN2 = 0x6,
        ADC_CONF1 = 0x7,
        ADC_CONF2 = 0x8,
        OV_THRESH = 0x9,
        UV_THRESH = 0xA,
        OTUT_THRESH = 0xB,
        UV_DISABLE1 = 0xC,
        UV_DISABLE2 = 0xD,
        GPIO_CONF1 = 0xE,
        GPIO_CONF2 = 0xF,
        GPIO_CONF3 = 0x10,
        GPIO_CONF4 = 0x11,
        OTP_SPARE14 = 0x12,
        OTP_SPARE13 = 0x13,
        OTP_SPARE12 = 0x14,
        OTP_SPARE11 = 0x15,
        FAULT_MSK1 = 0x16,
        FAULT_MSK2 = 0x17,
        PWR_TRANSIT_CONF = 0x18,
        COMM_TIMEOUT_CONF = 0x19,
        TX_HOLD_OFF = 0x1A,
        MAIN_ADC_CAL1 = 0x1B,
        MAIN_ADC_CAL2 = 0x1C,
        AUX_ADC_CAL1 = 0x1D,
        AUX_ADC_CAL2 = 0x1E,
        // OTP_RSVD1F = 0x1F,
        // OTP_RSVD20 = 0x20,
        CUST_MISC1 = 0x21,
        CUST_MISC2 = 0x22,
        CUST_MISC3 = 0x23,
        CUST_MISC4 = 0x24,
        CUST_MISC5 = 0x25,
        CUST_MISC6 = 0x26,
        CUST_MISC7 = 0x27,
        CUST_MISC8 = 0x28,
        STACK_RESPONSE = 0x29,
        BBP_LOC = 0x2A,
        OTP_RSVD_2B = 0x2B,
        OTP_SPARE_10 = 0x2C,
        OTP_SPARE_9 = 0x2D,
        OTP_SPARE_8 = 0x2E,
        OTP_SPARE_7 = 0x2F,
        OTP_SPARE_6 = 0x30,
        OTP_SPARE_5 = 0x31,
        OTP_SPARE_4 = 0x32,
        OTP_SPARE_3 = 0x33,
        OTP_SPARE_2 = 0x34,
        OTP_SPARE_1 = 0x35,
        CUST_CRC_HI = 0x36,
        CUST_CRC_LO = 0x37,
        OTP_PROG_UNLOCK1A = 0x300,
        OTP_PROG_UNLOCK1B = 0x301,
        OTP_PROG_UNLOCK1C = 0x302,
        OTP_PROG_UNLOCK1D = 0x303,
        DIR0_ADDR = 0x306,
        DIR1_ADDR = 0x307,
        COMM_CTRL = 0x308,
        CONTROL1 = 0x309,
        CONTROL2 = 0x30A,
        OTP_PROG_CTRL = 0x30B,
        ADC_CTRL1 = 0x30D,
        ADC_CTRL2 = 0x30E,
        ADC_CTRL3 = 0x30F,
        REG_INT_RSVD = 0x310,
        CB_CELL16_CTRL = 0x318,
        CB_CELL15_CTRL = 0x319,
        CB_CELL14_CTRL = 0x31A,
        CB_CELL13_CTRL = 0x31B,
        CB_CELL12_CTRL = 0x31C,
        CB_CELL11_CTRL = 0x31D,
        CB_CELL10_CTRL = 0x31E,
        CB_CELL9_CTRL = 0x31F,
        CB_CELL8_CTRL = 0x320,
        CB_CELL7_CTRL = 0x321,
        CB_CELL6_CTRL = 0x322,
        CB_CELL5_CTRL = 0x323,
        CB_CELL4_CTRL = 0x324,
        CB_CELL3_CTRL = 0x325,
        CB_CELL2_CTRL = 0x326,
        CB_CELL1_CTRL = 0x327,
        VMB_DONE_THRESH = 0x328,
        MB_TIMER_CTRL = 0x329,
        VCB_DONE_THRESH = 0x32A,
        OTCB_THRESH = 0x32B,
        OVUV_CTRL = 0x32C,
        OTUT_CTRL = 0x32D,
        BAL_CTRL1 = 0x32E,
        BAL_CTRL2 = 0x32F,
        BAL_CTRL3 = 0x330,
        FAULT_RST1 = 0x331,
        FAULT_RST2 = 0x332,
        DIAG_OTP_CTRL = 0x335,
        DIAG_COMM_CTRL = 0x336,
        DIAG_PWR_CTRL = 0x337,
        DIAG_CBFET_CTRL1 = 0x338,
        DIAG_CBFET_CTRL2 = 0x339,
        DIAG_COMP_CTRL1 = 0x33A,
        DIAG_COMP_CTRL2 = 0x33B,
        DIAG_COMP_CTRL3 = 0x33C,
        DIAG_COMP_CTRL4 = 0x33D,
        DIAG_PROT_CTRL = 0x33E,
        OTP_ECC_DATAIN1 = 0x343,
        OTP_ECC_DATAIN2 = 0x344,
        OTP_ECC_DATAIN3 = 0x345,
        OTP_ECC_DATAIN4 = 0x346,
        OTP_ECC_DATAIN5 = 0x347,
        OTP_ECC_DATAIN6 = 0x348,
        OTP_ECC_DATAIN7 = 0x349,
        OTP_ECC_DATAIN8 = 0x34A,
        OTP_ECC_DATAIN9 = 0x34B,
        OTP_ECC_TEST = 0x34C,
        SPI_CONF = 0x34D,
        SPI_TX3 = 0x34E,
        SPI_TX2 = 0x34F,
        SPI_TX1 = 0x350,
        SPI_EXE = 0x351,
        OTP_PROG_UNLOCK2A = 0x352,
        OTP_PROG_UNLOCK2B = 0x353,
        OTP_PROG_UNLOCK2C = 0x354,
        OTP_PROG_UNLOCK2D = 0x355,
        DEBUG_CTRL_UNLOCK = 0x700,
        DEBUG_COMM_CTRL1 = 0x701,
        DEBUG_COMM_CTRL2 = 0x702,
        PARTID = 0x500,
        DEV_REVID = 0xE00,
        DIE_ID1 = 0x501,
        DIE_ID2 = 0x502,
        DIE_ID3 = 0x503,
        DIE_ID4 = 0x504,
        DIE_ID5 = 0x505,
        // DIE_ID6 = 0x506,
        // DIE_ID7 = 0x507,
        // DIE_ID8 = 0x508,
        DIE_ID9 = 0x509,
        CUST_CRC_RSLT_HI = 0x50C,
        CUST_CRC_RSLT_LO = 0x50D,
        OTP_ECC_DATAOUT1 = 0x510,
        OTP_ECC_DATAOUT2 = 0x511,
        OTP_ECC_DATAOUT3 = 0x512,
        OTP_ECC_DATAOUT4 = 0x513,
        OTP_ECC_DATAOUT5 = 0x514,
        OTP_ECC_DATAOUT6 = 0x515,
        OTP_ECC_DATAOUT7 = 0x516,
        OTP_ECC_DATAOUT8 = 0x517,
        OTP_ECC_DATAOUT9 = 0x518,
        OTP_PROG_STAT = 0x519,
        OTP_CUST1_STAT = 0x51A,
        OTP_CUST2_STAT = 0x51B,
        SPI_RX3 = 0x520,
        SPI_RX2 = 0x521,
        SPI_RX1 = 0x522,
        DIAG_STAT = 0x526,
        ADC_STAT1 = 0x527,
        ADC_STAT2 = 0x528,
        GPIO_STAT = 0x52A,
        BAL_STAT = 0x52B,
        DEV_STAT = 0x52C,
        FAULT_SUMMARY = 0x52D,
        FAULT_COMM1 = 0x530,
        FAULT_COMM2 = 0x531,
        FAULT_COMM3 = 0x532,
        FAULT_OTP = 0x535,
        FAULT_SYS = 0x536,
        FAULT_PROT1 = 0x53A,
        FAULT_PROT2 = 0x53B,
        FAULT_OV1 = 0x53C,
        FAULT_OV2 = 0x53D,
        FAULT_UV1 = 0x53E,
        FAULT_UV2 = 0x53F,
        FAULT_OT = 0x540,
        FAULT_UT = 0x541,
        FAULT_COMP_GPIO = 0x543,
        FAULT_COMP_VCCB1 = 0x545,
        FAULT_COMP_VCCB2 = 0x546,
        FAULT_COMP_VCOW1 = 0x548,
        FAULT_COMP_VCOW2 = 0x549,
        FAULT_COMP_VBOW1 = 0x54B,
        FAULT_COMP_VBOW2 = 0x54C,
        FAULT_COMP_CBFET1 = 0x54E,
        FAULT_COMP_CBFET2 = 0x54F,
        FAULT_COMP_MISC = 0x550,
        FAULT_PWR1 = 0x552,
        FAULT_PWR2 = 0x553,
        FAULT_PWR3 = 0x554,
        CB_COMPLETE1 = 0x556,
        CB_COMPLETE2 = 0x557,
        BAL_TIME = 0x558,
        VCELL16_HI = 0x568,
        VCELL16_LO = 0x569,
        VCELL15_HI = 0x56A,
        VCELL15_LO = 0x56B,
        VCELL14_HI = 0x56C,
        VCELL14_LO = 0x56D,
        VCELL13_HI = 0x56E,
        VCELL13_LO = 0x56F,
        VCELL12_HI = 0x570,
        VCELL12_LO = 0x571,
        VCELL11_HI = 0x572,
        VCELL11_LO = 0x573,
        VCELL10_HI = 0x574,
        VCELL10_LO = 0x575,
        VCELL9_HI = 0x576,
        VCELL9_LO = 0x577,
        VCELL8_HI = 0x578,
        VCELL8_LO = 0x579,
        VCELL7_HI = 0x57A,
        VCELL7_LO = 0x57B,
        VCELL6_HI = 0x57C,
        VCELL6_LO = 0x57D,
        VCELL5_HI = 0x57E,
        VCELL5_LO = 0x57F,
        VCELL4_HI = 0x580,
        VCELL4_LO = 0x581,
        VCELL3_HI = 0x582,
        VCELL3_LO = 0x583,
        VCELL2_HI = 0x584,
        VCELL2_LO = 0x585,
        VCELL1_HI = 0x586,
        VCELL1_LO = 0x587,
        // BUSBAR_HI = 0x588,
        // BUSBAR_LO = 0x589,
        TSREF_HI = 0x58C,
        TSREF_LO = 0x58D,
        GPIO1_HI = 0x58E,
        GPIO1_LO = 0x58F,
        GPIO2_HI = 0x590,
        GPIO2_LO = 0x591,
        GPIO3_HI = 0x592,
        GPIO3_LO = 0x593,
        GPIO4_HI = 0x594,
        GPIO4_LO = 0x595,
        GPIO5_HI = 0x596,
        GPIO5_LO = 0x597,
        GPIO6_HI = 0x598,
        GPIO6_LO = 0x599,
        GPIO7_HI = 0x59A,
        GPIO7_LO = 0x59B,
        GPIO8_HI = 0x59C,
        GPIO8_LO = 0x59D,
        DIETEMP1_HI = 0x5AE,
        DIETEMP1_LO = 0x5AF,
        DIETEMP2_HI = 0x5B0,
        DIETEMP2_LO = 0x5B1,
        AUX_CELL_HI = 0x5B2,
        AUX_CELL_LO = 0x5B3,
        AUX_GPIO_HI = 0x5B4,
        AUX_GPIO_LO = 0x5B5,
        AUX_BAT_HI = 0x5B6,
        AUX_BAT_LO = 0x5B7,
        AUX_REFL_HI = 0x5B8,
        AUX_REFL_LO = 0x5B9,
        AUX_VBG2_HI = 0x5BA,
        AUX_VBG2_LO = 0x5BB,
        AUX_AVAO_REF_HI = 0x5BE,
        AUX_AVAO_REF_LO = 0x5BF,
        AUX_AVDD_REF_HI = 0x5C0,
        AUX_AVDD_REF_LO = 0x5C1,
        AUX_OV_DAC_HI = 0x5C2,
        AUX_OV_DAC_LO = 0x5C3,
        AUX_UV_DAC_HI = 0x5C4,
        AUX_UV_DAC_LO = 0x5C5,
        AUX_OT_OTCB_DAC_HI = 0x5C6,
        AUX_OT_OTCB_DAC_LO = 0x5C7,
        AUX_UT_DAC_HI = 0x5C8,
        AUX_UT_DAC_LO = 0x5C9,
        AUX_VCBDONE_DAC_HI = 0x5CA,
        AUX_VCBDONE_DAC_LO = 0x5CB,
        AUX_VCM_HI = 0x5CC,
        AUX_VCM_LO = 0x5CD,
        REFOVDAC_HI = 0x5D0,
        REFOVDAC_LO = 0x5D1,
        DIAG_MAIN_HI = 0x5D2,
        DIAG_MAIN_LO = 0x5D3,
        DIAG_AUX_HI = 0x5D4,
        DIAG_AUX_LO = 0x5D5,
        DEBUG_COMM_STAT = 0x780,
        DEBUG_UART_RC = 0x781,
        DEBUG_UART_RR_TR = 0x782,
        T = 0x783,
        DEBUG_COMH_RC = 0x784,
        DEBUG_COMH_RR_TR = 0x785,
        DEBUG_COML_BIT = 0x786,
        DEBUG_COML_RC = 0x787,
        DEBUG_COML_RR_TR = 0x788,
        DEBUG_UART_DISCARD = 0x789,
        DEBUG_COMH_DISCARD = 0x78A,
        DEBUG_COML_DISCARD = 0x78B,
        DEBUG_UART_VALID_HI = 0x78C,
        DEBUG_UART_VALID_LO = 0x78D,
        DEBUG_COMH_VALID_HI = 0x78E,
        DEBUG_COMH_VALID_LO = 0x78F,
        DEBUG_COML_VALID_HI = 0x790,
        DEBUG_COML_VALID_LO = 0x791,
        DEBUG_OTP_SEC_BLK = 0x7A0,
        DEBUG_OTP_DED_BLK = 0x7A1,
        CS_ADC_CAL1 = 0x1F,
        CS_ADC_CAL2 = 0x20,
        MAIN_CURRENT_HI = 0x588,
        MAIN_CURRENT_LO = 0x589,
        CURRENT_HI = 0x506,
        CURRENT_MID = 0x507,
        CURRENT_LO = 0x508
    };

    std::vector<uint8_t> GetBuf();
    int& GetDataLen();

    void Comm(RequestType req_type, byte data_size, byte dev_addr, RegisterAddress reg_addr, std::vector<byte> data);
    std::vector<std::vector<uint8_t>> ReadReg(RequestType req_type,
                                              byte dev_addr,
                                              RegisterAddress reg_addr,
                                              byte resp_size);
    void DummyReadReg(RequestType req_type, byte dev_addr, RegisterAddress reg_addr, byte resp_size);

    // uint16_t calculateCRC();
    bool verifyCRC(std::vector<uint8_t> buf);
    void WakePing();
    void CommClear();

    void SetAllDataArrValues(byte value);

    void SetStackSize(int newSize);

    void EnableUartDebug();

    void StartOVUV();

    void StartOTUT();

#ifdef BQTEST
private:
#endif
    HardwareSerial& uart_;
    uint8_t tx_pin_;
    std::vector<byte> data_arr_;

    void BeginUart();
};