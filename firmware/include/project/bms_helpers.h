#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>

#include <project/ltc6803_3.h>
#include <project/onewire.h>

#define WRN_HI_TEMP   80
#define WRN_LOW_TEMP    5

int8_t calculateTotalCellCount(LTC6803_Struct ltc_struct[], uint8_t stackSize);
int8_t calculateStackPos(uint8_t cellNum, LTC6803_Struct ltc_struct[], uint8_t stackSize);
void readAllBoard(void);
void readOneBoard(uint8_t i);
int getBoardTemp(int boardNo);
int getAverageTemp(void);
void circularSend(int amt);
void calculateCellPos(uint8_t* cellPos, uint8_t* stackPos, uint8_t cellNum, LTC6803_Struct ltc_struct[], uint8_t stackSize);

