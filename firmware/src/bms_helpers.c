#include <project/bms_helpers.h>
#include <project/onewire.h>

/* UNTESTED */

// array containing all temperatures, filled in when readAllBoard is called.
static int temperatures[NO_ROMS][2];

/*
   Return the number of cells present on the LTC6803-3 stack according to the configuration
   
   Parameter 1: Input stack array
   Parameter 2: Input stack size
   Return: Number of cells in stack
 */
int8_t calculateTotalCellCount(LTC6803_Struct ltc_struct[], uint8_t stackSize){
  int8_t TotalCellCount = 0;
  int8_t i;
  
  for(i=0; i<stackSize; i++){
    TotalCellCount = TotalCellCount + ltc_struct[i].nCells;
  }
  return TotalCellCount;
}

/*
   Return the cell voltage for the desired cell in the stack in mV
   
   Parameter 1: Input stack array
   Parameter 2: Desired position of chip in the stack (1 to nCells)
   Return: Cell voltage in mV

  
*/
void calculateCellPos(uint8_t* cellPos, uint8_t* stackPos, uint8_t cellNum, LTC6803_Struct ltc_struct[], uint8_t stackSize){
  uint8_t i;
  
  if(cellNum <= calculateTotalCellCount(ltc_struct, stackSize)){
    for(i=0; i<stackSize; i++){
      if(cellNum > ltc_struct[i].nCells){
        cellNum = cellNum - ltc_struct[i].nCells;
      }else{
          *cellPos = cellNum;
          *stackPos = i;
        return;
      }
    }
  }  
  
}

// get temperatures of all boards and issue warnings for overtemp and undertemp
void readAllBoard(void) {
    int i = 0;

    for(i = 1 ; i <= numROMs ; i++) {
      readOneBoard(i);
    /*
        temperatures[i][0] = tempSense(i) >> 16;
        temperatures[i][1] = tempSense(i) & 0x7F;
#if OW_DEBUG
        UART_printf("DEVICE ID: %d TEMP: %d\n", temperatures[i][0], temperatures[i][1]);
#endif
*/
        // issue board overtemp / undertemp warnings
        if(temperatures[i][1] < WRN_LOW_TEMP || temperatures[i][1] > WRN_HI_TEMP) {
            // do something on CAN
        }
    }
}

void readOneBoard(uint8_t i){
  //static uint8_t readBoard=1;
  
  //readBoard = readBoard % numROMs;
  //readBoard = readBoard + 1;
  
  int temperatureHold;
  
  temperatureHold = tempSense(i);
  temperatures[i][0] = temperatureHold >> 16;
  temperatures[i][1] = temperatureHold & 0x7F;
#if OW_DEBUG
  UART_printf("I: %d DEVICE ID: %d TEMP: %d\n", (int32_t) i, temperatures[i][0], temperatures[i][1]);
#endif
}

// get temperature of a certain board, note that readAllBoard() should be called
// beforehand to ensure temp data is the most recent
int getBoardTemp(int boardNo) {
    int i;

    for(i = 1 ; i <= numROMs ; i++) {
        if(temperatures[i][0] == boardNo) {
            return temperatures[i][1];
        }
    }

    return -1;
}

// returns the average temperatures of all of the boards
// readAllBoard() should be called beforehand to ensure temp data is recent
int getAverageTemp(void) {
    int avg = 0;
    int i;

    for(i = 1 ; i != numROMs + 1 ; i++) {
        avg += temperatures[i][1];
    }

    avg /= numROMs;

    return avg;
}

// sends data in a circular manner, call within the main loop and specify
// how many temp reading should be sent (amt)
void circularSend(int amt) {
    static int pos;
    int i = 0;

    while(i <= amt) {
        // send temperatures over CAN
        
        // if the end of the array has been reached, reset the counter
        if(pos == numROMs + 1) {
            pos = 1;
        }
    }
}
