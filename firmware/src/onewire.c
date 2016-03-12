/* onewire.c
 * An implementation of the onewire protocol for LPC11xx devices
 * Based on implementation specified in application Note 162 by 
 * Dallas Semiconductor / Maxim Semiconductor
 *
 * Note that onewire bus is multiplexed with SPI
 * 
 * Pin Assignments for BMS + 1-wire
 * PIO0_10 = SCK  = TX_EN = 1-wire input or output (high for output)
 * PIO0_2  = CSBI = CSEL  = SPI or 1-wire enable (1-wire high)
 * PIO0_9  = SDI  = TX    = 1-wire out
 * PIO0_8  = DSO  = RX    = 1-wire in
 *
 * Place romcodes into FoundROM if you want to hardcode it
 *
 * PROGRAMMING BOARD NUMBERS INTO THE SCRATCHPAD (T_h register)
 * Call writeSP with a SINGLE temperature sensor plugged in as
 * it issues a skip rom commans
 *
 * FINDING THE BOARD NUMBER OF A TEMPERATURE READ
 * The board number is returned as the 16 MSB of tempSense()'s return
 * value
 * i.e. for temperature = tempSense(x) | 0x7F
 * i.e. for board number = tempSense(x) >> 16
 */

#include <arch/timer16.h>
#include <arch/gpio.h>
#include <project/onewire.h>

static unsigned char ROM[8];
static int lastDiscrep;
static int doneFlag;
static unsigned char dowcrc;
static unsigned char FoundROM[NO_ROMS][8] = {
    //{0x10, 0xDD, 0xDC, 0x6C, 0x01, 0x08, 0x00, 0x55},
    //{0x10, 0xE7, 0xD0, 0x6c, 0x01, 0x08, 0x00, 0x56}
};
static unsigned char dscrc_table[] = {
0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
157,195, 33,127,252,162, 64, 30, 95, 1,227,189, 62, 96,130,220,
35,125,159,193, 66, 28,254,160,225,191, 93, 3,128,222, 60, 98,
190,224, 2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89, 7,
219,133,103, 57,186,228, 6, 88, 25, 71,165,251,120, 38,196,154,
101, 59,217,135, 4, 90,184,230,167,249, 27, 69,198,152,122, 36,
248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91, 5,231,185,
140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
202,148,118, 40,171,245, 23, 73, 8, 86,180,234,105, 55,213,139,
87, 9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

/* uDelay() - performs a blocking delay in usecs
 * note that context switching overheads add extra time to the delay
 */
inline void uDelay(int uSec) {
    LPC_TMR16B0->TCR = 0x02;
    LPC_TMR16B0->PR  = SystemCoreClock / 1000000;
    LPC_TMR16B0->IR  = 0xff;
    LPC_TMR16B0->MCR = 0x04;
    LPC_TMR16B0->MR0 = uSec;
    LPC_TMR16B0->TCR = 0x01;

    // blocking wait
    while (LPC_TMR16B0->TCR & 0x01);
}

/* ow_reset() - performs a 1-wire reset operation
 * returns 0 if a 1-wire device is present
 */
unsigned char ow_reset() {
    int8_t presence;

    GPIO_SetValue(TX_EN_PRT, TX_EN_PIN, 1);     // enable Tx
    GPIO_SetValue(TX_PRT, TX_PIN, 0);           // send value
    uDelay(480);

    GPIO_SetValue(TX_EN_PRT, TX_EN_PIN, 0);     // disable Tx
    uDelay(70);
    presence = GPIO_GetValue(RX_PRT, RX_PIN);

    // device is now reset
    uDelay(420);
    return presence;
}
 
/* read_bit() - reads a bit from the 1-wire bus
 * returns the bit read
 */
unsigned char read_bit() {
    // enable Tx
    GPIO_SetValue(TX_EN_PRT, TX_EN_PIN, 1);

    // pull low and let device pull up/down
    GPIO_SetValue(TX_PRT, TX_PIN, 0);
    GPIO_SetValue(TX_EN_PRT, TX_EN_PIN, 0);

    uDelay(15);
    return GPIO_GetValue(RX_PRT, RX_PIN);
}

/* write_bit() - writes a bit to the 1-wire bus */
void write_bit(char bit) {
    // set bus to low and write bit out to bus
    GPIO_SetValue(TX_EN_PRT, TX_EN_PIN, 1);
    GPIO_SetValue(TX_PRT, TX_PIN, 0);
    uDelay(1);
    GPIO_SetValue(TX_PRT, TX_PIN, bit);

    uDelay(100);
    GPIO_SetValue(TX_PRT, TX_PIN, 1);
}

/* write_byte() - writes multiple bytes to 1-wire bus */
void write_byte(char val) {
    int i = 0;

    for(i = 0 ; i < 8 ; i++) {
        write_bit((val >> i) & 0x01);
    }

    uDelay(120);
}


/* read_byte() - reads a byte off the 1-wire bus */
int8_t read_byte() {
    int retVal = 0;
    int i;
    
    for(i = 0 ; i < 8 ; i++) {
        if(read_bit()) {
            retVal |= 0x01 << i;
        }

        uDelay(120);
    }

    return retVal;
}

/* tempSense() - returns temperature (in celcius) of device ROM
 * to specify which device is read, pass in the index of the device (ROM)
 * in the foundROM array
 *
 * FINDING THE BOARD NUMBER OF A TEMPERATURE READ
 * The board number is returned as the 16 MSB of tempSense()'s return
 * value
 * i.e. for temperature = tempSense(x) | 0x7F
 * i.e. for board number = tempSense(x) >> 16
 */
int tempSense(int ROM) {
    char get[10];
    char temp_lsb, temp_msb;
    int k, i;
    
    ow_reset();
    write_byte(0x55);
    uDelay(10);

    for(i = 0 ; i < 8 ; i++) {
        write_byte(FoundROM[ROM][i]);
    }
    write_byte(0x44); // Start Conversion
    uDelay(102);
    
    ow_reset();
    write_byte(0x55);
    uDelay(10);
    for(i = 0 ; i < 8 ; i++) {
        write_byte(FoundROM[ROM][i]);
    }

    // Read Scratch Pad
    write_byte(0xBE);
    for (k=0;k<9;k++){
        get[k]=read_byte();
    }

    temp_msb = get[1]; // Sign byte + lsbit
    temp_lsb = get[0]; // Temp data plus lsb

    // shift to get whole degree
    if (temp_msb <= 0x80)
        temp_lsb = (temp_lsb/2);

    // mask all but the sign bit
    temp_msb = temp_msb & 0x80;
    
    // twos complement
    if (temp_msb >= 0x80)
        temp_lsb = (~temp_lsb)+1;

    // shift to get whole degree
    if (temp_msb >= 0x80)    
        temp_lsb = (temp_lsb/2);

    // add sign bit
    if (temp_msb >= 0x80)
        temp_lsb = ((-1)*temp_lsb);

    return (int)temp_lsb | get[2] << 16;
}

unsigned char First(void) {
    lastDiscrep = 0;
    // reset the rom search last discrepancy global
    doneFlag = FALSE;

    // call Next and return its return value
    return Next();
}

// NEXT
// The Next function searches for the next device on the 1-wire bus. If
// there are no more devices on the 1-wire then false is returned.
//
unsigned char Next(void) {
    unsigned char m = 1;    // ROM Bit index 
    unsigned char n = 0;    // ROM Byte index
    unsigned char k = 1;    // bit mask
    unsigned char x = 0;
    unsigned char discrepMarker = 0; // discrepancy marker
    unsigned char g;            // Output bit
    unsigned char nxt = FALSE;  // return value
    int flag;

    // reset the 1-wire
    flag = ow_reset();

    // reset the dowcrc
    dowcrc = 0;

    // no parts -> return false
    if(flag||doneFlag) {
        // reset the search
        lastDiscrep = 0;
        return FALSE;
    }

    // send SearchROM command    
    write_byte(0xF0);

    // for all eight bytes
    do {
        x = 0;
        if(read_bit()==1) x = 2;
        uDelay(120);
        if(read_bit()==1) x |= 1;
        uDelay(120);
        if(x ==3) break;
        else {
            // all devices coupled have 0 or 1
            if(x>0)
                // bit write value for search
                g = x>>1;
            else {
                // if this discrepancy is before the last
                // discrepancy on a previous Next then pick
                // the same as last time
                if(m<lastDiscrep) g = ((ROM[n]&k)>0);
                else g = (m==lastDiscrep);

                // if equal to last pick 1
                // if not then pick 0
                // if 0 was picked then record
                // position with mask k

                if (g==0) discrepMarker = m;
            }
            
            // isolate bit in ROM[n] with mask k
            if(g==1) ROM[n] |= k;
            else ROM[n] &= ~k;

            // ROM search write
            write_bit(g);

            // increment bit counter m
            m++;
            // and shift the bit mask k
            k = k<<1;

            // if the mask is 0 then go to new ROM
            if(k==0) {
                // accumulate the CRC
                ow_crc(ROM[n]);
                // byte n and reset mask
                n++;
                k++;
            }
        }
    } while(n<8);

    // if search was unsuccessful then
    // reset the last discrepancy to 0
    if(m<65||dowcrc) lastDiscrep=0;
    else {
        // search was successful, so set lastDiscrep,
        // lastOne, nxt;
        lastDiscrep = discrepMarker;
        doneFlag = (lastDiscrep==0);

        // indicates search is not complete yet, more parts remain
        nxt = TRUE;
    }
    return nxt;
}

// FIND DEVICES
uint8_t FindDevices(void) {
    unsigned char m;

    // Begins when a presence is detected
    if(!ow_reset()) {
        // Begins when at least one part is found
        if(First()) {
            numROMs=0;

            do {
                numROMs++;
                for(m=0;m<8;m++) {
                    // Identifies ROM number on found device
                    FoundROM[numROMs][m]=ROM[m];
                }
#if OW_DEBUG
                UART_printf("\nSENSOR = %d ROM = %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                    numROMs + 1, FoundROM[numROMs][0],FoundROM[numROMs][1],
                    FoundROM[numROMs][2],FoundROM[numROMs][3], FoundROM[numROMs][4],
                    FoundROM[numROMs][5],FoundROM[numROMs][6],FoundROM[numROMs][7]);
#endif
            } while (Next()&&(numROMs<=NO_ROMS)); //Continues until no additional devices are found
        }
    }
    
    return numROMs;
}

// crc function
unsigned char ow_crc( unsigned char x) {
dowcrc = dscrc_table[dowcrc^x];
return dowcrc;
}

// write to scratchpad of a SINGLE sensor
void writeSP(int brdNo) {
    int i;
    int get[8];

    ow_reset();
    write_byte(0xCC);
    write_byte(0x4E);
    write_byte(brdNo);
    write_byte(0x0);
    write_byte(0x7F);
    while(read_bit() == 0);

    ow_reset();
    write_byte(0xCC);
    write_byte(0x48);
    uDelay(120);

    ow_reset();
    write_byte(0xCC);
    uDelay(120);
    write_byte(0xBE);
    for (i = 0 ; i < 9 ; i++){
        get[i] = read_byte();
    }

    if(get[2] != brdNo || get[3] != 0 || get[4] != 0x7F) {
        UART_printf("BAD CONFIGURATION?\n");
    }
}

