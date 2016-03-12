/* GPIO port configuration */
#define TX_EN_PRT   0
#define TX_EN_PIN   10
#define TX_PRT      0
#define TX_PIN      9
#define RX_PRT      0
#define RX_PIN      8
#define CSEL_PRT    0
#define CSEL_PIN    2
#define NO_ROMS     21

#define OW_DEBUG    0
#define OW_ENABLE   1

/* function declarations */
unsigned char ow_reset(void);
unsigned char read_bit(void);
void write_bit(char bit);
void write_byte(char bytes);
int8_t read_byte(void);
inline void uDelay(int uSec);
void read_temperature(void);

int OWSearch(void);
int tempSense(int ROM);

unsigned char First(void);
unsigned char Next(void);
uint8_t FindDevices(void);
unsigned char ow_crc( unsigned char x);

void writeSP(int brdNo);

/* Globals */
int numROMs;
