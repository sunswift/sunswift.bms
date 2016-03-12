/* --------------------------------------------------------------------------                                 
    LTC6803-3 Driver
    File name: ltc6803_3.c
    Author: Charith Perera
    Description: This is the driver file for the LTC6803-3 BMS chip
    it provides user level functions which provide the ability to read and write
    chip parameters such as configuration, cell voltages and other register groups.
    It depends on a low level SPI driver to handle the transfers, in this case
    it is SSP_Send and SSP_Receive.

    Created: 
    For PEC stuff: from endlesssphere
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

#include <arch/types.h>

#include <scandal/stdio.h>
#include <project/ansi_color.h>

#include <project/ltc6803_3.h>
#include <scandal/utils.h>
#include <project/driver_config.h>
#include <arch/ssp.h>
#include <arch/gpio.h>

static byte crc8_table[256];      // 8-bit table for PEC calc
static int made_table = 0;         // table made flag
byte packet[40]={0};               // used for PEC calc
byte packet2[40]={0};
byte PECbyte;                                                   // PEC of a byte
byte PECpacket;                // PEC of packet
byte PECpacketREAD;             // value that PECpacket should be as read from 6803


void init_crc8() {
  int z,j;
  byte cr;
  if (!made_table) {
      for (z = 0; z < 256; z ++) {
	cr = z;
	for (j = 0; j < 8; j ++) {
	    cr = (cr << 1) ^ ((cr & 0x80) ? 0x07 : 0);
	}
	crc8_table[z] = cr & 0xFF;
      }
      made_table = 1;
  }
}

void setPECbyte(byte m) {
  PECbyte = 0x41;                   // initialize PECbyte
  if (!made_table) {
      init_crc8();
  }
  PECbyte = crc8_table[(PECbyte) ^ m];
}


uint8_t calculatePEC(byte *input ,byte np) {            // np is number of bytes currently in input[]
  int z;
  int output;
  output = 0x41;                  // initialize PECpacket
  if (!made_table) {
      init_crc8();
  }
  for (z = 0; z < np; z ++) {
      output = crc8_table[(output) ^ input[z]];
  }
  return output;
}

/* Set the CS line low for the stack */
void initiateLine(void){
    setLTCPinFunctions();
    GPIO_SetValue(LTC6803_CS_PORT, LTC6803_CS_BIT, 0);
}

/* Set the CS line high for the stack */
void terminateLine(void){
    GPIO_SetValue(LTC6803_CS_PORT, LTC6803_CS_BIT, 1);
}

/* TODO: Set pin functions to those required for communication
   to LTC6803 in order to make sure it's not set for 1-wire mode*/
void setLTCPinFunctions(void){

}

/* Performs PEC Calculation and sends the data to the LTC6803 chip,
 Pass in the number of entries you have places into the packet register which
 are to be transmitted along with the calculated PEC byte.
 The number of bytes  you wish to read should be entered as the second argument.
 The third argument should be a pointer to where you want the output*/
int32_t PECReadWrite(byte *inArray, uint8_t numPEC, byte *outArray, uint8_t numRead){
  int32_t returnVal=PEC_SUCCESS; // equal to zero
  uint8_t readPEC;
#if DEBUG
  uint8_t i;
#endif
  
  //If we want to write anything, calculate the PEC and send it
  if(numPEC > 0){
    inArray[numPEC] = calculatePEC(inArray, numPEC);
    SSP_Send(0,packet,numPEC+1);
  }

  
  /* If we want to read anything, do the read and verify its PEC */
  if(numRead){
    SSP_Receive(0, outArray, numRead+1); //Read numRead bytes + PEC
    readPEC=calculatePEC(outArray, numRead); //Calculate the PEC for the numRead of bytes
    
    //If the PEC check failed, return -1 and print that we failed to verify the data
    if (readPEC != outArray[numRead]){ //readPEC != outArray[numRead]
      returnVal = PEC_FAILIURE; //-1
      
#if DEBUG
      UART_printf(ANSI_RED"PEC VERIFICATION FAILIURE!!!\r\n");
      for(i=0; i<numRead; i++){
        UART_printf("Byte %d = 0x%x\r\n", i, outArray[i]);
      }
      UART_printf("RCV PEC = 0x%x, CALC PEC = 0x%x\r\n"ANSI_RESET, outArray[numRead], readPEC);
    }else{
      UART_printf(ANSI_GREEN"PEC Passed! :)\r\n"ANSI_RESET);
      UART_printf(ANSI_GREEN"RCV PEC = 0x%x, CALC PEC = 0x%x\r\n"ANSI_RESET, outArray[numRead], readPEC);
#endif
    }
  }
  return returnVal;
}


/* If the PEC checks out, update cfg_struct with the configuration data. Return if the PEC passed or failed */
int32_t readConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackSize){
	int32_t returnVal=PEC_SUCCESS;
	int32_t pecStatus;
	int8_t stackPosition;
    LTC6803_CFG_Struct *cfg_struct=0;
	
	initiateLine();
    packet[0]=RDCFG;
    pecStatus=PECReadWrite(packet, 1, packet2, 0);
	
	for(stackPosition=0; stackPosition < stackSize; stackPosition++){
		cfg_struct=ltc_struct[stackPosition].LTC6803_CFG; //cfg_struct was being re-declared, check this works!
		pecStatus=PECReadWrite(packet, 0, packet2, 6);
		
		if(pecStatus==PEC_SUCCESS){
        cfg_struct->GPIO    =((packet2[0] & 0xE0) >> 5);
        cfg_struct->LVLPL   =((packet2[0] & 0x10) >> 4);
        cfg_struct->CELL10  =((packet2[0] & 0x08) >> 3);
        cfg_struct->CDC     =((packet2[0] & 0x07) >> 0);
        cfg_struct->DCC     =((packet2[2] & 0x0F) << 8) | ((packet2[1] & 0xFF) >> 0);
        cfg_struct->MC      =((packet2[3] & 0xFF) << 4) | ((packet2[2] & 0xF0) >> 4);
        cfg_struct->VUV     =((packet2[4] & 0xFF) >> 0);
        cfg_struct->VOV     =((packet2[5] & 0xFF) >> 0);
		}else{
			returnVal=PEC_FAILIURE;
		}
	}
	//returnVal=PECReadWrite(packet, 0, packet2, 6);

	terminateLine();
    return returnVal;
}

int32_t writeConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackSize){
	int32_t returnVal=PEC_SUCCESS;
	int32_t pecStatus;
	int8_t stackPosition;
    uint8_t tmp;
	
	initiateLine();
    packet[0]=WRCFG;
    PECReadWrite(packet, 1, packet2, 0);

    /* Iterate over each chip from the top down */
	for(stackPosition=(stackSize-1); stackPosition >= 0; stackPosition--){
		LTC6803_CFG_Struct *cfg_struct=ltc_struct[stackPosition].LTC6803_CFG;
		
		tmp  = ((cfg_struct->GPIO & 0x7) << 5);
		tmp |= ((cfg_struct->LVLPL & 0x1) << 4);
		tmp |= ((cfg_struct->CELL10 & 0x1) << 3);
		tmp |= ((cfg_struct->CDC & 0x7) << 0);
		packet[0] = tmp;
		
		packet[1] = ((cfg_struct->DCC & 0xFF) << 0);
		
		tmp  = ((cfg_struct->DCC & 0xF00) >> 8);
		tmp |= ((cfg_struct->MC & 0x0F) << 4);
		packet[2] = tmp;
		
		packet[3] = ((cfg_struct->MC & 0xFF0) >> 4);
		packet[4] = ((cfg_struct->VUV & 0xFF) >> 0);
		packet[5] = ((cfg_struct->VOV & 0xFF) >> 0);
		
		pecStatus=PECReadWrite(packet, 6, packet2, 0);
	}

    terminateLine();
    return returnVal;
}

int32_t verifyConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackSize){
    /*
    Steps:
        Create temporary cfg_struct
        Read lowest LTC into temporary cfg_struct
        Compare temporary cfg_struct to lowest local struct
        Flag as fail on mismatch or leave success
        Read next LTC into temporary cfg_struct
        Compare temporary cfg_struct with appropriate local struct
        Flag fail on a mismatch
        Return CFG_MATCH or CFG_MISMATCH
    */
    
    int32_t pecStatus;
    int8_t stackPosition;
    LTC6803_CFG_Struct *cfg_struct_store=0;
    LTC6803_CFG_Struct read_cfg_struct;
    
    initiateLine();
    packet[0]=RDCFG;
    pecStatus=PECReadWrite(packet, 1, packet2, 0);
    
    for(stackPosition=0; stackPosition < stackSize; stackPosition++){
        cfg_struct_store=ltc_struct[stackPosition].LTC6803_CFG; //cfg_struct was being re-declared, check this works!
        pecStatus=PECReadWrite(packet, 0, packet2, 6);
        
        if(pecStatus==PEC_SUCCESS){
            read_cfg_struct.GPIO    =((packet2[0] & 0xE0) >> 5);
            read_cfg_struct.LVLPL   =((packet2[0] & 0x10) >> 4);
            read_cfg_struct.CELL10  =((packet2[0] & 0x08) >> 3);
            read_cfg_struct.CDC     =((packet2[0] & 0x07) >> 0);
            read_cfg_struct.DCC     =((packet2[2] & 0x0F) << 8) | ((packet2[1] & 0xFF) >> 0);
            read_cfg_struct.MC      =((packet2[3] & 0xFF) << 4) | ((packet2[2] & 0xF0) >> 4);
            read_cfg_struct.VUV     =((packet2[4] & 0xFF) >> 0);
            read_cfg_struct.VOV     =((packet2[5] & 0xFF) >> 0);
            
            /* We do not want to compare GPIO as this is a GPIO line which can change...
            if(read_cfg_struct.GPIO != cfg_struct_store->GPIO){
                UART_printf("FAIL1 %d %d\r\n", read_cfg_struct.GPIO, cfg_struct_store->GPIO);
                terminateLine();
                return CFG_MISMATCH;
            }
            */
            if(read_cfg_struct.LVLPL != cfg_struct_store->LVLPL){
                UART_printf("FAIL2\r\n");
                terminateLine();
                return LVLPL_MISMATCH;
            }
            
            if(read_cfg_struct.CELL10 != cfg_struct_store->CELL10){
                UART_printf("FAIL3\r\n");
                terminateLine();
                return CELL10_MISMATCH;
            }
            
            if(read_cfg_struct.CDC != cfg_struct_store->CDC){
                UART_printf("FAIL4\r\n");
                terminateLine();
                return CDC_MISMATCH;
            }
            
            if(read_cfg_struct.DCC != cfg_struct_store->DCC){
                UART_printf("FAIL5\r\n");
                terminateLine();
                return DCC_MISMATCH;
            }
            
            if(read_cfg_struct.MC != cfg_struct_store->MC){
                UART_printf("FAIL6\r\n");
                terminateLine();
                return MC_MISMATCH;
            }
            
            if(read_cfg_struct.VUV != cfg_struct_store->VUV){
                UART_printf("FAIL7\r\n");
                terminateLine();
                return VUV_MISMATCH;
            }
            
            if(read_cfg_struct.VOV != cfg_struct_store->VOV){
                UART_printf("FAIL8\r\n");
                terminateLine();
                return VOV_MISMATCH;
            }
        
        }else{
            terminateLine();
            return PEC_FAILIURE;
        }
        

    }
    //returnVal=PECReadWrite(packet, 0, packet2, 6);

    terminateLine();
    return CFG_MATCH;

}

/* Accepts a stack array and stack position of where to store the desired
   configuration details as well as the desired configuration details
   Parameter 1: Input stack array
   Parameter 2: Desired position of the array to store configuration
   Parameter 3: Desired configuration details
   
   */
int32_t setConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackPosition, LTC6803_CFG_Struct *cfg_struct){
    
  LTC6803_CFG_Struct *cfg_struct_store=ltc_struct[stackPosition].LTC6803_CFG;
  
  //Store the desired configuration details in the desired position of the stack.
  cfg_struct_store->GPIO	= (cfg_struct->GPIO & 0x7);
  cfg_struct_store->LVLPL	= (cfg_struct->LVLPL & 0x1);
  cfg_struct_store->CELL10	= (cfg_struct->CELL10 & 0x1);
  cfg_struct_store->CDC		= (cfg_struct->CDC & 0x7);
  cfg_struct_store->DCC		= (cfg_struct->DCC & 0xFFF);
  cfg_struct_store->MC		= (cfg_struct->MC & 0xFFF);
  cfg_struct_store->VUV		= (cfg_struct->VUV & 0xFF);
  cfg_struct_store->VOV		= (cfg_struct->VOV & 0xFF);
  
  return 0; //Maybe change me into a void function
}

/* Print the configuration details of a given position on the stack. */
void printConfiguration(LTC6803_Struct ltc_struct[], uint8_t stackPosition){
    /* Input was (LTC6803_CFG_Struct *cfg_struct) */

    //Loading a pointer to our cfg_struct at stackPosition
    LTC6803_CFG_Struct *cfg_struct=ltc_struct[stackPosition].LTC6803_CFG;
    
    //Printing out our cfg_struct
    UART_printf("GPIO=%d\r\n", cfg_struct->GPIO);
    UART_printf("Level / Polling=%d\r\n", cfg_struct->LVLPL);
    UART_printf("10 Cell mode=%d\r\n", cfg_struct->CELL10);
    UART_printf("Comparater Duty Cycle=%d\r\n", cfg_struct->CDC);
    UART_printf("Discharge=%x\r\n", cfg_struct->DCC);
    UART_printf("Mask Interrupt=%x\r\n", cfg_struct->MC);
    UART_printf("Undervoltage=%d\r\n", cfg_struct->VUV);
    UART_printf("Overvoltage=%d\r\n", cfg_struct->VOV);
}


/* Enables or disables discharging from occurring on an LTC6803 IC

   Parameter 1: Input stack array
   Parameter 2: Desired position of chip in the stack
   Parameter 3: either "START_DISCHARGE" or "STOP_DISCHARGE" as defined in ltc6803.h
   Return: Nothing meaningful yet
   
   Example call: enableDischarge(LTC6803_Stack, 0, START_DISCHARGE); //Enables discharge on the bottom LTC6803

*/

int enableDischarge(LTC6803_Struct ltc_struct[], uint8_t stackPosition, uint8_t setting){

  // If we get either START_DISCHARGE or STOP_DISCHARGE, save those values
	if(setting == START_DISCHARGE){
    ltc_struct[stackPosition].enableDischarge = 1;
    ltc_struct[stackPosition].LTC6803_CFG->DCC = ltc_struct[stackPosition].discharge_cells;
	}else if(setting == STOP_DISCHARGE){
    ltc_struct[stackPosition].enableDischarge = 0;
    ltc_struct[stackPosition].LTC6803_CFG->DCC = 0;
  }

  return 0; // TODO: Change this to return something more meaningful
}


/* Sets the cells on a LTC6803 IC that will be discharged

   Parameter 1: Input stack array
   Parameter 2: Desired position of chip in the stack
   Parameter 3: Mask of the cells to be discharged where LSB = bottom module, up to 12 bits will be
                accepted depending on the CELL10 flag of the configuration struct
   Return: Nothing meaningful yet
   
   Example call: setDischarge(LTC6803_Stack, 0, 0x00F); // Discharge the bottom four modules of the bottom LTC6803

*/	

int setDischarge(LTC6803_Struct ltc_struct[], uint8_t stackPosition, uint16_t dischargeMask){

  // Saving the cell mask into discharge_cells and limiting the 
  if(ltc_struct[stackPosition].LTC6803_CFG->CELL10){
    ltc_struct[stackPosition].discharge_cells = dischargeMask & 0x03FF;
  }else{
    ltc_struct[stackPosition].discharge_cells = dischargeMask & 0x0FFF;
  }
  
  // if discharge is set, copy to DCC
  if(ltc_struct[stackPosition].enableDischarge){
    ltc_struct[stackPosition].LTC6803_CFG->DCC = ltc_struct[stackPosition].discharge_cells;
  }
  
  return 0; // TODO: Change this to return something more meaningful
}


/*
   Request and read the cell voltages for the stack, the values will be stored
   in the struct array.
   configuration details as well as the desired configuration details
   Parameter 1: Input stack array
   Parameter 2: Number of LTC6803 chips  in the stack
 */
int readCellVoltages(LTC6803_Struct ltc_struct[], uint8_t stackSize){
    int32_t returnVal=PEC_SUCCESS;
    int32_t pecStatus;
    int8_t stackPosition;
    int8_t CELL10_Stat;
    uint8_t i;
    
    initiateLine();
    packet[0]=STCVAD;
    PECReadWrite(packet, 1, packet2, 0);
    terminateLine();
    
    scandal_delay(30); //Wait 30ms for the conversion, which should take 21ms
    packet[0]=RDCV;
    initiateLine();
    PECReadWrite(packet, 1, packet2, 0);
    
    for(stackPosition=0; stackPosition < stackSize; stackPosition++){
        CELL10_Stat = ltc_struct[stackPosition].LTC6803_CFG->CELL10;
        
        /* The CELL10 configuration parameter changes how many bytes each chip
           sends us as we read the cell voltage register group
           Therefore we read and process the two cases seperately*/
        if(CELL10_Stat){
            pecStatus=PECReadWrite(packet, 0, packet2, 15);
            
            if (pecStatus == PEC_SUCCESS){
                for(i=0; i<= 4; i++){
                    ltc_struct[stackPosition].LTC6803_CV[0+2*i] =((packet2[1+3*i] & 0x0F) << 8) | ((packet2[0+3*i] & 0xFF) << 0);
                    ltc_struct[stackPosition].LTC6803_CV[1+2*i] =((packet2[2+3*i] & 0xFF) << 4) | ((packet2[1+3*i] & 0xF0) >> 4);
                }
            }else{ //Set returnVal to PEC_FAIL if pecStatus was not PEC_SUCCESS and set cell voltages to -1
                returnVal=PEC_FAILIURE;
                for(i=0; i< 12; i++){
                    ltc_struct[stackPosition].LTC6803_CV[i] = -1;
                }
            }
        
        }else{
            pecStatus=PECReadWrite(packet, 0, packet2, 18);
            
            if (pecStatus == PEC_SUCCESS){
                for(i=0; i<= 5; i++){
                    ltc_struct[stackPosition].LTC6803_CV[0+2*i] =((packet2[1+3*i] & 0x0F) << 8) | ((packet2[0+3*i] & 0xFF) << 0);
                    ltc_struct[stackPosition].LTC6803_CV[1+2*i] =((packet2[2+3*i] & 0xFF) << 4) | ((packet2[1+3*i] & 0xF0) >> 4);
                }
            }else{ //Set returnVal to PEC_FAIL if pecStatus was not PEC_SUCCESS and set cell voltages to -1
                returnVal=PEC_FAILIURE;
                for(i=0; i< 12; i++){
                    ltc_struct[stackPosition].LTC6803_CV[i] = -1;
                }
            }
        }
    }
    terminateLine();
    
    return returnVal;
}


/*
   Return the cell voltage for the desired cell in the stack in mV
   
   Parameter 1: Input stack array
   Parameter 2: Desired position of chip in the stack
   Parameter 3: Cell position on the chip  (1 to nCells)
   Return: Cell voltage in mV
 */
int16_t retrieveCellVoltages(LTC6803_Struct ltc_struct[], uint8_t stackPosition, uint8_t cellPosition){
    int16_t returnVal=0;
    
    //nCells goes from 1 up while cellPosition is from 0 to (nCells - 1)
    if ((cellPosition <= ltc_struct[stackPosition].nCells) & (cellPosition > 0)){
        returnVal = ltc_struct[stackPosition].LTC6803_CV[cellPosition-1];
        returnVal = returnVal - 512;
        returnVal = returnVal + returnVal/2;        
    }
    
    return returnVal;
    
}



int readFlags(LTC6803_FLG_Struct *flg_struct){
/* 
For reference:
    uint8_t FLGR0;
    uint8_t FLGR1;
    uint8_t FLGR2;
*/
    int32_t pec_status;
    packet[0]=RDFLG;
    initiateLine();
    pec_status=PECReadWrite(packet, 1, packet2, 3);
    terminateLine();
    
    if(pec_status == PEC_SUCCESS){
        flg_struct->FLGR0=packet2[0];
        flg_struct->FLGR1=packet2[1];
        flg_struct->FLGR2=packet2[2];
    }
    
    
    return pec_status;
}

//TODO: START TEMPERATURE CONVERSION, WAIT, THEN READ!!! Don't just read the registers...
int readTemps(LTC6803_TMP_Struct *tmp_struct){
/* 
For reference:
    uint16_t ETMP1;
    uint16_t ETMP2;
    uint16_t ITMP2;
    uint8_t  THSD;
*/
    int32_t pec_status;
    packet[0]=RDTMP;
    initiateLine();
    pec_status=PECReadWrite(packet, 1, packet2, 5);
    terminateLine();

    if(pec_status == PEC_SUCCESS){
        tmp_struct->ETMP1=((packet2[1] & 0x0F) << 4) | ((packet2[0] & 0xFF) << 0);
        tmp_struct->ETMP2=((packet2[2] & 0xFF) << 4) | ((packet2[1] & 0xF0) >> 4);
        tmp_struct->ITMP2=((packet2[4] & 0x0F) << 4) | ((packet2[3] & 0xFF) << 0);
        tmp_struct->THSD =((packet2[4] & 0x10) >> 4);
    }
    
    return pec_status;
}


//TODO: START DIAGNOSTIC CHECK, WAIT, THEN READ!!! Don't just read the registers...

int readDiagnostics_old(LTC6803_DGN_Struct *dgn_struct){
/* 
For reference:
    uint16_t REF; //Reference voltage test
    uint8_t MUXFAIL; //Mux failiure flag
    uint8_t REV; //Chip revision
*/
    int32_t pec_status;
    packet[0]=DAGN;
    initiateLine();
    pec_status=PECReadWrite(packet, 1, packet2, 0);
    
    scandal_delay(17);
    packet[0]=RDDGNR;
    pec_status=PECReadWrite(packet, 1, packet2, 3);

    
    terminateLine();

    //FIX ME!!! wrong everything below...
    if(0){
       
    }
    
    return pec_status;

}


int readDiagnostics(LTC6803_Struct ltc_struct[], uint8_t stackSize){
    int32_t returnVal=PEC_SUCCESS;
    int32_t pecStatus;
    int8_t stackPosition;
    //uint8_t i;
    LTC6803_DGN_Struct *diag_struct=0;

    
    initiateLine();
    packet[0]=DAGN;
    PECReadWrite(packet, 1, packet2, 0);
    terminateLine();
    
    scandal_delay(20); //Wait 20ms for diagnostics, they should take 16ms
    packet[0]=RDDGNR;
    initiateLine();
    PECReadWrite(packet, 1, packet2, 0);
    
    for(stackPosition=0; stackPosition < stackSize; stackPosition++){
      diag_struct=ltc_struct[stackPosition].LTC6803_DGN; //cfg_struct was being re-declared, check this works!
      pecStatus=PECReadWrite(packet, 0, packet2, 2);
        
      if (pecStatus == PEC_SUCCESS){
        diag_struct->REF     = ((packet2[1] & 0x0F) << 4) | ((packet2[0] & 0xFF) << 0);
        diag_struct->MUXFAIL = ((packet2[1] & 0x20) >> 5);
        diag_struct->REV     = ((packet2[1] & 0xC0) >> 6);
      }else{
        returnVal=PEC_FAILIURE;
      }
    }
    terminateLine();
    
    return returnVal;
}


int testRead(void){
    int32_t returnVal;
    int32_t i;
    int32_t imax=6;
    packet[0]=RDCFG;
    initiateLine();
    returnVal=PECReadWrite(packet, 1, packet2, imax);
    //terminateLine();
    
    for(i=0; i<=imax; i++){
        UART_printf("p%d = %x\r\n", (int) i, (int) packet2[i]);
    }
    
    //initiateLine();
    returnVal=PECReadWrite(packet, 0, packet2, imax);
    terminateLine();
    
    for(i=0; i<=imax; i++){
        UART_printf("p%d = %x\r\n", (int) i, (int) packet2[i]);
    }
   

   return 0;  
}


/*
  if(discVar){
    discVar = 0;
  }else{
    discVar = 0x0;//x3FF;
  }
  
  if(discVar != cfg_struct.DCC){
    cfg_struct.DCC = discVar;
    setConfiguration(LTC6803_Stack, 0, &cfg_struct);
    setConfiguration(LTC6803_Stack, 1, &cfg_struct);
    writeConfiguration(LTC6803_Stack, LTC6803_Stack_Size);
  }
*/
void timedDischarge(uint32_t cellMask, int32_t dischargeTime, LTC6803_CFG_Struct *cfg_struct, LTC6803_Struct ltc_struct[], uint8_t stackSize){
  uint32_t dischargeStep;
  

  
  while(dischargeTime >= 0){
    /* Figuring out how long we must discharge for */
    if(dischargeTime < 2000){
      dischargeTime = 0;
      dischargeStep = dischargeTime;
    }else{
      dischargeTime = dischargeTime - 2000;
      dischargeStep = 2000;
    }
    
  }
  
  cfg_struct->DCC = 0;
  setConfiguration(ltc_struct, 0, cfg_struct);
  setConfiguration(ltc_struct, 1, cfg_struct);
  writeConfiguration(ltc_struct, stackSize);
}
