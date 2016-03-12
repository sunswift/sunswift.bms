#define WRCFG       0x01
#define RDCFG       0x02
#define RDCV        0x04
#define RDCVA       0x06
#define RDCVB       0x08
#define RDCVC       0x0A
#define RDFLG       0x0C
#define RDTMP       0x0E
#define STCVAD      0x10
#define STOWAD      0x20
#define STTMPAD     0x30
#define PLADC       0x40
#define PLINT       0x50
#define DAGN        0x52
#define RDDGNR      0x54
#define STCVDC      0x60
#define STOWDC      0x70




#define LTC6803_CS_PORT  0
#define LTC6803_CS_BIT   2

//Note: Failiures are less than ZERO
#define PEC_SUCCESS      0
#define PEC_FAILIURE    -1

#define CFG_MATCH        0
#define CFG_MISMATCH    -1
#define LVLPL_MISMATCH  -2
#define CELL10_MISMATCH -3
#define CDC_MISMATCH    -4
#define DCC_MISMATCH    -5
#define MC_MISMATCH     -6
#define VUV_MISMATCH    -7
#define VOV_MISMATCH    -8

#define GENERAL_FAILIURE -1

#define CFG_MATCH        0
#define CFG_MISMATCH    -1

#define START_DISCHARGE 1
#define STOP_DISCHARGE  0

#define DEBUG 0
#define DEBUG_SENDING 0

#define ENABLE_BALANCE   1 //Set this to 1 to enable balancing
#define BALANCE_INACTIVE 1
#define BALANCE_ACTIVE   2
#define BALANCE_COOLDOWN 3
#define DISCHARGE_TIME   10000 // 5 minutes = 300s = 300,000ms

#define BALANCE_THRESHOLD 20 //This is the threshold of voltage difference after which balancing will be enabled

#define OVERVOLTAGE_WARNING   4210
#define OVERVOLTAGE_CRITICAL  4250

#define UNDERVOLTAGE_WARNING  2750
#define UNDERVOLTAGE_CRITICAL 2500




typedef uint8_t byte;

typedef struct _LTC6803_CFG_Struct{
    uint8_t GPIO; //[2]:WDT, [1]:GPIO2, [0]:GPIO1
    uint8_t LVLPL; //Level Polling Mode (0b:Toggle polling [def], 1b:Level polling)
    uint8_t CELL10; //10 Cell mode (0b:12 cell [def], 1b: 10 cell)
    uint8_t CDC; //Comparater Duty Cycle
    uint16_t DCC; //Discharge ([11]:Cell 12 -> [0]:Cell 1)
    uint16_t MC; //Disable interrupt for cell ([11]:Cell 12 -> [0]:Cell 1)
    uint8_t VUV; //Undervoltage threshold = (VUV - 31) * 16 * 1.5mV
    uint8_t VOV; //Overvoltage threshold = (VOV - 32) * 16 * 1.5mV
} LTC6803_CFG_Struct;


//Holds the undervoltage and overvoltage flags, see datasheet for details!
typedef struct _LTC6803_FLG_Struct{
    uint8_t FLGR0;
    uint8_t FLGR1;
    uint8_t FLGR2;
} LTC6803_FLG_Struct;

//Temperature readings struct, two external sensors and internal 
typedef struct _LTC6803_TMP_Struct{
    uint16_t ETMP1;
    uint16_t ETMP2;
    uint16_t ITMP2;
    uint8_t  THSD;
} LTC6803_TMP_Struct;

//Diagnostic structure
typedef struct _LTC6803_DGN_Struct{
    uint16_t REF;     // Reference voltage test
    uint8_t MUXFAIL;  // Mux failiure flag
    uint8_t REV;      // Chip revision
} LTC6803_DGN_Struct;

typedef struct _LTC6803_Struct{
  LTC6803_CFG_Struct 	*LTC6803_CFG;
  LTC6803_FLG_Struct 	*LTC6803_FLG;
  LTC6803_TMP_Struct	*LTC6803_TMP;
  LTC6803_DGN_Struct	*LTC6803_DGN;
  int16_t   LTC6803_CV[12];
  uint8_t nCells;
  uint8_t enableDischarge; // If discharge is currently enabled or not
  uint16_t discharge_cells; // The cells which are to be discharged
} LTC6803_Struct;

void init_crc8();
void setPECbyte(byte m);
uint8_t calculatePEC(byte *input ,byte np);

void initiateLine(void);
void terminateLine(void);
void setLTCPinFunctions(void);


// Managing configuration register group
int32_t readConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackSize);
int32_t writeConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackSize);
int32_t verifyConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackSize);
int32_t setConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackPosition, LTC6803_CFG_Struct *cfg_struct);
void printConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackPosition);

// Managing balancing
int enableDischarge(LTC6803_Struct ltc_struct[], uint8_t stackPosition, uint8_t setting);
int setDischarge(LTC6803_Struct ltc_struct[], uint8_t stackPosition, uint16_t dischargeMask);

// Managing voltage register group
int readCellVoltages(LTC6803_Struct ltc_struct[], uint8_t stackSize);
int16_t retrieveCellVoltages(LTC6803_Struct ltc_struct[], uint8_t stackPosition, uint8_t cellPosition);


// Managing flag register group
int readFlags(LTC6803_FLG_Struct *flg_struct);

// Managing temperature register group
int readTemps(LTC6803_TMP_Struct *tmp_struct);

// Managing diagnostics register group
int readDiagnostics(LTC6803_Struct ltc_struct[], uint8_t stackSize);

int testRead(void);