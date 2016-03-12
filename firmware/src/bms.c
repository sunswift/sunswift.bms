/* --------------------------------------------------------------------------                                 
    BMS project main
    File name: bms.c
    Author: Charith Perera, David Favaloro, Geoffrey Chen
    Credits to: 
    Description: 

    Created: 
  -------------------------------------------------------------------------- */

/* 
* This file is part of the Sunswift BMS project
* 
* This tempalte is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* It is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with the project.  If not, see <http://www.gnu.org/licenses/>.
*/

#define BASE_YEL_LED_PORT 3
#define BASE_YEL_LED_BIT 1
#define BASE_RED_LED_PORT 3
#define BASE_RED_LED_BIT 0

#define ERROR_NONE                    0
#define ERROR_COMMS                   1
#define ERROR_TEMPERATURE_WARNING     2
#define ERROR_VOLTAGE_WARNING         2
#define ERROR_TEMPERATURE_CRITICAL    10
#define ERROR_VOLTAGE_CRITICAL        10

#define TEMPERATURE_FAN_THRESHOLD     40
#define TEMPERATURE_WARN_LIMIT        50
#define TEMPERATURE_CRITICAL_LIMIT    65

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>

#include <string.h>

#include <project/driver_config.h>

#include <project/ansi_color.h>
#include <project/target_config.h>

#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#include <arch/ssp.h>
#include <arch/timer16.h>
//#include <project/ltc6803_3.h>
#include <project/bms_helpers.h>
#include <project/onewire.h>
#include <project/scandal_config.h>

#define FAN_SWITCH_PORT 2
#define FAN_SWITCH_BIT  10
#define BUZZER_SWITCH_PORT 0
#define BUZZER_SWITCH_BIT  7

#define ISOLATOR_SWITCH_PORT 0
#define ISOLATOR_SWITCH_BIT  4

#if SCANDAL_ADDRESS_OVERRIDE == BMS_HIGHER_ADDRESS
#define MODULE_NUMBER_OFFSET 19
#endif

#if SCANDAL_ADDRESS_OVERRIDE == BMS_LOWER_ADDRESS
#define MODULE_NUMBER_OFFSET 0
#endif

#if SCANDAL_ADDRESS_OVERRIDE == BMS_HIGHER_ADDRESS
#define MODULE_NUMBER_OFFSET 19
#endif

extern byte PECbyte;
extern byte PECpacket;
extern byte PECpacketREAD;
extern byte packet[18];

LTC6803_Struct LTC6803_Stack[2]; //Stack of LTC6803 chips, bottom 
uint8_t	LTC6803_Stack_Size = 2;
uint8_t numCells;

uint8_t LTC6803_State;
uint8_t One_Wire_State;

int16_t vCell_Max;
int16_t vCell_Min;
uint16_t dischargeVec[2]; // Holds the modules which need to be discharged
int balanceState = BALANCE_INACTIVE;

int tCell_Max;
int tCell_Min;

void set_fan(uint8_t fanVal);
void set_buzzer(uint8_t buzzVal);
void set_isolator(uint8_t isoVal);

void sendData(void);

/* Do some general setup for clocks, LEDs and interrupts
* and UART stuff on the MSP430 */
void setup(void) {
  GPIO_Init();
  GPIO_SetDir(BASE_RED_LED_PORT, BASE_RED_LED_BIT, 1);
  GPIO_SetDir(BASE_YEL_LED_PORT, BASE_YEL_LED_BIT, 1);
  GPIO_SetDir(FAN_SWITCH_PORT, FAN_SWITCH_BIT, 1);
  GPIO_SetDir(BUZZER_SWITCH_PORT, BUZZER_SWITCH_BIT, 1);

  GPIO_SetDir(LTC6803_CS_PORT, LTC6803_CS_BIT, 1);
  //I set the SSP Chip select line to an output, I should be in an LTC6803_Init function

  // set up timer for onewire
  init_timer16(0, 10);

  SSP_init_struct SSP_Config;

  SSP_Config.DataSize=SSPCR0_DSS_8BIT; //8 Bits of data
  SSP_Config.FrameFormat=SSPCR0_FRF_SPI; //SPI Format
  SSP_Config.ClockPolarity=1;
  SSP_Config.ClockPhase=1;
  SSP_Config.ClockRate=119; //SET ME!
  SSP_Config.Slave=0;
  SSP_Config.ClockPrescale=2; //Greater or equal to 2, I must be

  //SSP_IOConfig(0);
  SSP_IOConfig(0); //Configure pins for SSP0
  //SSP0_Init(&SSP_Config); //Initialise SSP0 with the configuration desired
  SSP_new_Init(&SSP_Config, LPC_SSP0);
  init_crc8();

  static LTC6803_CFG_Struct LTC6803_Bot_CFG;
  static LTC6803_FLG_Struct LTC6803_Bot_FLG;
  static LTC6803_TMP_Struct LTC6803_Bot_TMP;
  static LTC6803_DGN_Struct LTC6803_Bot_DGN;

  static LTC6803_CFG_Struct LTC6803_Top_CFG;
  static LTC6803_FLG_Struct LTC6803_Top_FLG;
  static LTC6803_TMP_Struct LTC6803_Top_TMP;
  static LTC6803_DGN_Struct LTC6803_Top_DGN;
  
  dischargeVec[0] = 0;
  dischargeVec[1] = 0;

  LTC6803_Stack[0].LTC6803_CFG = &LTC6803_Bot_CFG;
  LTC6803_Stack[0].LTC6803_FLG = &LTC6803_Bot_FLG;
  LTC6803_Stack[0].LTC6803_TMP = &LTC6803_Bot_TMP;
  LTC6803_Stack[0].LTC6803_DGN = &LTC6803_Bot_DGN;


  LTC6803_Stack[1].LTC6803_CFG = &LTC6803_Top_CFG;
  LTC6803_Stack[1].LTC6803_FLG = &LTC6803_Top_FLG;
  LTC6803_Stack[1].LTC6803_TMP = &LTC6803_Top_TMP;
  LTC6803_Stack[1].LTC6803_DGN = &LTC6803_Top_DGN;
  
  // Disable discharging
  LTC6803_Stack[0].enableDischarge  = STOP_DISCHARGE;
  LTC6803_Stack[1].enableDischarge  = STOP_DISCHARGE;
  
  // Don't discharge any cells for now
  LTC6803_Stack[0].discharge_cells  = 0;
  LTC6803_Stack[1].discharge_cells  = 0x3FF;
  
  /* Defining how many modules in each pack there are to be monitored */
  #if SCANDAL_ADDRESS_OVERRIDE == BMS_LOWER_ADDRESS
  LTC6803_Stack[0].nCells      = 9;
  LTC6803_Stack[1].nCells      = 10;
  #endif
  
  #if SCANDAL_ADDRESS_OVERRIDE == BMS_HIGHER_ADDRESS
  LTC6803_Stack[0].nCells      = 10;
  LTC6803_Stack[1].nCells      = 10;
  #endif
  
  numCells = calculateTotalCellCount(LTC6803_Stack, LTC6803_Stack_Size);
} // setup

// onewire setup - called every time when taking a temp reading
void ow_setup(void) {
  // configure for gpio
  GPIO_SetDir(TX_PRT, TX_PIN, 1);         // Tx
  GPIO_SetDir(RX_PRT, RX_PIN, 0);         // Rx
  GPIO_SetDir(CSEL_PRT, CSEL_PIN, 1);     // active high
  GPIO_SetDir(TX_EN_PRT, TX_EN_PIN, 1);   // active high

  GPIO_SetFunction(TX_PRT, TX_PIN, 0x00, 0x00);
  GPIO_SetFunction(RX_PRT, RX_PIN, 0x00, 0x00);
  GPIO_SetFunction(CSEL_PRT, CSEL_PIN, 0x00, 0x00);
  GPIO_SetFunction(TX_EN_PRT, TX_EN_PIN, 0x00, 0x00);

  // probing the onewire bus
  GPIO_SetValue(CSEL_PRT, CSEL_PIN, 1);
}

// spi_setup - reconfigures GPIO for SPI
void spi_setup(void) {
  SSP_IOConfig(0); //Configure pins for SSP0

  GPIO_SetValue(CSEL_PRT, CSEL_PIN, 0);
}

/* This is your main function! You should have an infinite loop in here that
* does all the important stuff your node was designed for */
int main(void) {   
  /* Initialise UART */
  UART_Init(115200);

  LTC6803_CFG_Struct cfg_struct;
  int32_t cfg_status = CFG_MISMATCH;
  //uint8_t i; //Temporary loop variable 1
  //uint8_t j; //Temporary loop variable 2
  uint8_t yellow_led_status = 0;
  uint8_t red_led_status = 0;
  //uint16_t discVar=0;
  
  static uint8_t tempSensePos;
  uint8_t checkPos;
  uint8_t cellPos;
  uint8_t stackPos;
  int16_t vCell;
  int tCell;
  
  uint32_t fanTimer = 0;
  
  uint8_t numROMs;
  
  setup(); 
  

  /* Initialise Scandal, registers for the config messages, timesync messages and in channels */
  scandal_init();

  /* Set LEDs to known states */
  red_led(1);
  yellow_led(1);

  /* Wait until UART is ready */
  scandal_delay(100);

  sc_time_t one_sec_timer = sc_get_timer();
  sc_time_t datasend_timer = sc_get_timer();
  sc_time_t dischargeStart = sc_get_timer();
  
  /* DELETE ME, DEBUG CODE!!!! */
  /*
  while(1){
    handle_scandal();
    
   
    
    if(sc_get_timer() >= one_sec_timer) {
      one_sec_timer = one_sec_timer + 1000;
      yellow_led_status = !yellow_led_status;
      ow_setup();
      
      
      GPIO_SetValue(CSEL_PRT, CSEL_PIN, 1);
      GPIO_SetValue(TX_EN_PRT, TX_EN_PIN, 1);
      GPIO_SetValue(TX_PRT, TX_PIN, yellow_led_status);
      //GPIO_SetValue(CSEL_PRT, CSEL_PIN, 1); 
    }
  
  }
  */

  //readConfiguration(LTC6803_Stack, LTC6803_Stack_Size); 
  
  //Setting desired config settings
  cfg_struct.CELL10 = 1;
  cfg_struct.LVLPL = 0;
  cfg_struct.CDC = 7;//7
  cfg_struct.DCC = 0;//(cfg_struct.DCC ^ 7);
  cfg_struct.MC = 0; //0xFF8
  cfg_struct.VOV = 207;// =  (VUV (in mV) / 24 ) + 32, 207 = 4200mV
  cfg_struct.VUV = 145; // =  (VUV (in mV) / 24 ) + 31, 145 = 2640mV
  
  //printConfiguration(LTC6803_Stack, 0);
  setConfiguration(LTC6803_Stack, 0, &cfg_struct);
  setConfiguration(LTC6803_Stack, 1, &cfg_struct);
  
  writeConfiguration(LTC6803_Stack, LTC6803_Stack_Size);

  ow_setup();
  numROMs = FindDevices();
  readAllBoard();

  set_fan(1);

  while (1) {
    /* Update the timer */
    handle_scandal();

  /* Send a UART and CAN message and flash an LED every second */
    if(sc_get_timer() >= one_sec_timer) {
      one_sec_timer = one_sec_timer + 1000;
      
      dischargeVec[0] = 0;
      dischargeVec[1] = 0;
      
      //setDischarge(LTC6803_Stack, 0, 0);
      //setDischarge(LTC6803_Stack, 1, 0);
      //enableDischarge(LTC6803_Stack, 0, START_DISCHARGE);
      //enableDischarge(LTC6803_Stack, 1, START_DISCHARGE);
      
      
/*  LTC6803 Voltage Monitoring  */      

      // perform spi actions
      spi_setup();

      /* Check if the configuration of the chips is what we set */
      cfg_status = verifyConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
      
      if(cfg_status != CFG_MATCH){
        UART_printf(ANSI_RED"CONFIGURATION LOST - POSSIBLE WATCHDOG EVENT\n\r"ANSI_RESET);
        writeConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
        cfg_status = verifyConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
      }

      if((cfg_status == CFG_MATCH) && (balanceState == BALANCE_INACTIVE)){

        LTC6803_State = ERROR_NONE;
        readCellVoltages(LTC6803_Stack, LTC6803_Stack_Size);
        
        vCell_Max=retrieveCellVoltages(LTC6803_Stack, 0, 1);
        vCell_Min=retrieveCellVoltages(LTC6803_Stack, 0, 1);
        
        for(checkPos=1; checkPos<=numCells; checkPos++){
          calculateCellPos(&cellPos, &stackPos, checkPos, LTC6803_Stack, LTC6803_Stack_Size);
          vCell = retrieveCellVoltages(LTC6803_Stack, stackPos, cellPos);
          
          /* Maximum and Minimum voltage detection */
          if (vCell > vCell_Max){
            vCell_Max = vCell;
          }
          
          if (vCell < vCell_Min){
            vCell_Min = vCell;
          }
          
          /* Detecting of any of the voltages are beyond the warning thresholds and flagging */
          if((vCell > OVERVOLTAGE_WARNING) || (vCell < UNDERVOLTAGE_WARNING)){
            UART_printf(ANSI_RED"VOLTAGE WARNING THRESHOLD EXCEEDED\n\r"ANSI_RESET);
            LTC6803_State = ERROR_VOLTAGE_WARNING;
          }
          
          /* Detecting of any of the voltages are beyond the critical thresholds and flagging */
          if((vCell > OVERVOLTAGE_CRITICAL) || (vCell < UNDERVOLTAGE_CRITICAL)){
            UART_printf(ANSI_RED"VOLTAGE CRITICAL THRESHOLD EXCEEDED\n\r"ANSI_RESET);
            LTC6803_State = ERROR_VOLTAGE_CRITICAL;
          }
        }
        
        // Find battery cells which need discharging
        if ((LTC6803_State == ERROR_NONE) && ((vCell_Max - vCell_Min) > BALANCE_THRESHOLD) && ENABLE_BALANCE) {
            UART_printf (ANSI_YELLOW"Vmin = %d\n\r"ANSI_RESET, (int) vCell_Min);
            for (checkPos = 1; checkPos <= numCells; checkPos++) {    
                calculateCellPos(&cellPos, &stackPos, checkPos, LTC6803_Stack, LTC6803_Stack_Size);
                vCell = retrieveCellVoltages(LTC6803_Stack, stackPos, cellPos); 
                if (vCell > (vCell_Min + BALANCE_THRESHOLD)) {
                    dischargeVec[stackPos] = dischargeVec[stackPos] | (1<<(cellPos-1));
                    UART_printf (ANSI_YELLOW"Discharging: S = %d, C = %d, V = %dmV, Delta = %dmV\n\r"ANSI_RESET, (int) stackPos, (int) cellPos, (int) vCell, (int) vCell - (int) vCell_Min);

                }
            }

    UART_printf (ANSI_RED"dischargeVec[0] is %x, dischargeVec[1] is %x\n\r"ANSI_RESET, (int)dischargeVec[0], (int)dischargeVec[1]);

    
    
            enableDischarge(LTC6803_Stack, 0, START_DISCHARGE);     // Stack 0
            setDischarge(LTC6803_Stack, 0, dischargeVec[0]); 
            enableDischarge(LTC6803_Stack, 1, START_DISCHARGE);     // Stack 1
            setDischarge(LTC6803_Stack, 1, dischargeVec[1]);            
            writeConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
        
            balanceState = BALANCE_ACTIVE;
            dischargeStart = sc_get_timer();
        }else{
            enableDischarge(LTC6803_Stack, 0, STOP_DISCHARGE);     // Stack 0
            enableDischarge(LTC6803_Stack, 1, STOP_DISCHARGE);     // Stack 1
            //writeConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
        }
        
      }else if((cfg_status == CFG_MATCH) && (balanceState == BALANCE_ACTIVE)){
        
          
        if (sc_get_timer() > dischargeStart + DISCHARGE_TIME) {
            balanceState = BALANCE_INACTIVE;
            UART_printf (ANSI_GREEN"Ending Discharge\n\r"ANSI_RESET);

            enableDischarge(LTC6803_Stack, 0, STOP_DISCHARGE);     // Stack 0
            enableDischarge(LTC6803_Stack, 1, STOP_DISCHARGE);     // Stack 1
            writeConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
        }
      }else{
        
        UART_printf(ANSI_RED"LTC6803 RECONFIGURATION FAILED - COMMS ERROR\n\r"ANSI_RESET);
        LTC6803_State = ERROR_COMMS;
        balanceState = BALANCE_INACTIVE;
        enableDischarge(LTC6803_Stack, 0, STOP_DISCHARGE);     // Stack 0
        enableDischarge(LTC6803_Stack, 1, STOP_DISCHARGE);     // Stack 1
      }
      
/*  One Wire temperature sensing is done on a rotating basis */      
      

      ow_setup();
      One_Wire_State = ERROR_NONE;
      if(numROMs != numCells){/*numROMs != numCells*/ /* TODO: Errors hidden here, un hide them!!! */
        //UART_printf(ANSI_RED"1-WIRE DETECTION FAULT (%d/%d) - COMMS ERROR"ANSI_RESET, (int) numROMs, (int) numCells);
        numROMs = FindDevices(); /* Attempt to recover by detecting all the available ID's */
        //UART_printf(ANSI_RED" | NEW SEARCH DISCOVERED %d\n\r"ANSI_RESET, (int) numROMs);
        
        if(numROMs != numCells){
          One_Wire_State = ERROR_COMMS; /* If we have not yet recovered, flag an error */
        }else{
          readAllBoard(); /* Do a full read to speed up the recovery process, if we have recovered */
        }
      }
      
      tempSensePos = tempSensePos % numROMs;
      tempSensePos++; //sendPos now ranges from 1 up to numCells

      readOneBoard(tempSensePos); //sendPos
      
      /* Initialising the maximum and minimum */
      tCell_Max = getBoardTemp(1+MODULE_NUMBER_OFFSET);
      tCell_Min = getBoardTemp(1+MODULE_NUMBER_OFFSET);
      
      for(checkPos=1; checkPos<=numCells; checkPos++){
        tCell = getBoardTemp(tempSensePos+MODULE_NUMBER_OFFSET);
        
        /* Maximum and Minimum temperature detection */
        if (tCell > tCell_Max){
          tCell_Max = tCell;
        }
        
        if (tCell < tCell_Min){
          tCell_Min = tCell;
        }
        
        /* Initialisation Case */
        if(tCell == 85){
          UART_printf(ANSI_RED"UNINITIALISED TEMPERATURE DETECTED AT %d\n\r"ANSI_RESET, (int) checkPos);
          One_Wire_State = ERROR_COMMS;
          
        /* Communication error case */
        }else if(tCell == -1){
          if(One_Wire_State != ERROR_COMMS){ /* Only print once to avoid spamming too much */
            UART_printf(ANSI_RED"ONEWIRE COMMUNICATION ERROR - NO RESPONSE OR NO DATA FROM %d\n\r"ANSI_RESET, (int) checkPos);
          }
          One_Wire_State = ERROR_COMMS;
          
        /* Exceeding critical limit case */  
        }else if(tCell > TEMPERATURE_CRITICAL_LIMIT){
          UART_printf(ANSI_RED"TEMPERATURE CRITICAL LIMIT EXCEEDED\n\r"ANSI_RESET);
          One_Wire_State = ERROR_TEMPERATURE_WARNING;
          
        /* Exceeding warning limit */
        }else if(tCell > TEMPERATURE_WARN_LIMIT){
          UART_printf(ANSI_RED"TEMPERATURE WARNING LIMIT EXCEEDED\n\r"ANSI_RESET);
          One_Wire_State = ERROR_TEMPERATURE_WARNING;
        }
      }

      

/*   Error handling */
      

      /* Handling of LTC6803 State and warnings */
      if(LTC6803_State){
        // Stay on if there is a fault state flagged
        red_led_status = 0;
      }else{
        // Keep toggling if there are no errors
        red_led_status = !red_led_status;
      }
      
      GPIO_SetValue (BASE_RED_LED_PORT, BASE_RED_LED_BIT, red_led_status);

      
      /* Handling of One Wire State and warnings */
      
      if(One_Wire_State){
        // Stay on if there is a fault state flagged
        yellow_led_status = 0;
        numROMs = 0; // Clearing the numROMs value if we have an error to ensure we do a re-search and read on the next cycle
      }else{
        // Keep toggling if there are no errors
        yellow_led_status = !yellow_led_status;
      }
      
      GPIO_SetValue (BASE_YEL_LED_PORT, BASE_YEL_LED_BIT, yellow_led_status);
      
      /* Handling of combined warnings */
      
      /* Handling of Fan for over voltage  */
      
      if(tCell_Max >=TEMPERATURE_FAN_THRESHOLD){
        /* If the maximum detected temperature is above the fan threshold,
           Run the fan for 60 seconds */
        fanTimer = (uint32_t) sc_get_timer() + 60000;
      }
      
      if(fanTimer > (uint32_t) sc_get_timer()){
        set_fan(1);
      }else{  
        set_fan(0);
      }

    }
    
    if(sc_get_timer() >= datasend_timer) { 
      datasend_timer = datasend_timer + 250; //Primt number to rotate on
      sendData();
    }
  }
}

void sendData(void){
  static uint8_t sendPos=0;
  uint8_t stackPos;
  uint8_t cellPos;
 
  sendPos = sendPos % numCells;
  sendPos++; //sendPos now ranges from 1 up to numCells
  
  calculateCellPos(&cellPos, &stackPos, sendPos, LTC6803_Stack, LTC6803_Stack_Size);
  
  /*Priority, Channel Number, Value*/
  scandal_send_channel(TELEM_LOW, (uint16_t) sendPos, (uint32_t) retrieveCellVoltages(LTC6803_Stack, stackPos, cellPos));
  scandal_send_channel(TELEM_LOW, (uint16_t) (100+sendPos), (uint32_t) 1000*getBoardTemp(sendPos+MODULE_NUMBER_OFFSET));
#if DEBUG_SENDING
  UART_printf("Sending S = %d\tC = %d\tpos %d\tV=%d\tT = %d\t%d\n\r", stackPos, cellPos, sendPos , retrieveCellVoltages(LTC6803_Stack, stackPos, cellPos), (int) getBoardTemp(sendPos+MODULE_NUMBER_OFFSET), (int) sc_get_timer());
#endif
    
}

void set_fan(uint8_t fanVal){
  GPIO_SetValue(FAN_SWITCH_PORT, FAN_SWITCH_BIT, fanVal);
}

void set_buzzer(uint8_t buzzVal){
  GPIO_SetValue(BUZZER_SWITCH_PORT, BUZZER_SWITCH_BIT, buzzVal);
}

void set_isolator(uint8_t isoVal){
  GPIO_SetValue(ISOLATOR_SWITCH_PORT, ISOLATOR_SWITCH_BIT, isoVal);
}
