#define BT_w 13
#define BT_h 15
const uint8_t BT_bits[] PROGMEM = {
  0x80, 0x01, 0xC0, 0x03, 0xC0, 0x0E, 0xC2, 0x1C, 0xCE, 0x0C, 0xDC, 0x07,
  0xF0, 0x03, 0xE0, 0x00, 0xF0, 0x03, 0xDC, 0x07, 0xCE, 0x1C, 0xC2, 0x1C,
  0x80, 0x0F, 0xC0, 0x03, 0xC0, 0x00, };

#define wifi_w 19
#define wifi_h 15
const uint8_t wifi_bits[] PROGMEM = {
  0xC0, 0x1F, 0x00, 0xF8, 0xFF, 0x00, 0xFC, 0xFB, 0x01, 0x1F, 0xE0, 0x03,
  0x0F, 0x00, 0x07, 0xC3, 0x1F, 0x06, 0xE0, 0x7F, 0x00, 0xF0, 0x7E, 0x00,
  0x38, 0xE0, 0x00, 0x10, 0x06, 0x00, 0x80, 0x0F, 0x00, 0x80, 0x0F, 0x00,
  0x80, 0x0F, 0x00, 0x80, 0x0F, 0x00, 0x00, 0x07, 0x00, };

#define indicator_horizonal_w 32
#define indicator_horizonal_h 7
const uint8_t indicator_horizonal_bits[] PROGMEM = {
  0xFE, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0xC0,
  0x03, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFE, 0xFF, 0xFF, 0x7F, };

//#define indicator_horizonal_filled_w 32
//#define indicator_horizonal_filled_h 5
const uint8_t indicator_horizonal_filled_bits[] PROGMEM = {
  0xFE, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFE, 0xFF, 0xFF, 0x7F, };

#define indicator_vertical_w 8
#define indicator_vertical_h 32
const uint8_t indicator_vertical_bits[] PROGMEM = {
  0x3C, 0x7E, 0x66, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42,
  0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42,
  0x42, 0x42, 0x42, 0x42, 0x42, 0x66, 0x7E, 0x3C, };

//#define indicator_vertial_filled_w 6
//#define indicator_vertial_filled_h 32
const uint8_t indicator_vertial_filled_bits[] PROGMEM = {
  0x3C, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E,
  0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E,
  0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x3C, };

#define battery_100_w 33
#define battery_100_h 13
const uint8_t battery_100_bits[] PROGMEM = {
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x07, 0x00,
  0x00, 0xE0, 0x00, 0xFB, 0x7C, 0x3E, 0xDF, 0x00, 0xFB, 0xFD, 0x7E, 0xFF,
  0x01, 0xFB, 0xFD, 0x7E, 0xFF, 0x01, 0xFB, 0xFD, 0x7E, 0xFF, 0x01, 0xFB,
  0xFD, 0x7E, 0xFF, 0x01, 0xFB, 0xFD, 0x7E, 0xFF, 0x01, 0xFB, 0x7C, 0x3E,
  0xDF, 0x00, 0x07, 0x00, 0x00, 0xE0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0xFE, 0xFF, 0xFF, 0xFF, 0x00,  };

#define battery_75_w 34
#define battery_75_h 13
const uint8_t battery_75_bits[] PROGMEM = {
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x01, 0x07, 0x00,
  0x00, 0xC0, 0x01, 0xFF, 0xFD, 0x7E, 0xC0, 0x01, 0xFF, 0xFD, 0x7E, 0xC0,
  0x01, 0xFF, 0xFD, 0x7E, 0xC0, 0x01, 0xFF, 0xFD, 0x7E, 0xC0, 0x01, 0xFF,
  0xFD, 0x7E, 0xC0, 0x01, 0xFF, 0xFD, 0x7E, 0xC0, 0x01, 0xFF, 0xFD, 0x7E,
  0xC0, 0x01, 0xF7, 0x78, 0x3C, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, };

#define battery_50_w 34
#define battery_50_h 13
const uint8_t battery_50_bits[] PROGMEM = {
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x07, 0x00,
  0x00, 0xC0, 0x01, 0xFF, 0xFD, 0x00, 0xC0, 0x01, 0xFF, 0xFD, 0x00, 0xC0,
  0x01, 0xFF, 0xFD, 0x00, 0xC0, 0x01, 0xFF, 0xFD, 0x00, 0xC0, 0x01, 0xFF,
  0xFD, 0x00, 0xC0, 0x01, 0xFF, 0xFD, 0x00, 0xC0, 0x01, 0xFF, 0xFD, 0x00,
  0xC0, 0x01, 0xF7, 0x78, 0x00, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, };


#define battery_25_w 34
#define battery_25_h 13
const uint8_t battery_25_bits[] PROGMEM = {
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x07, 0x00,
  0x00, 0xC0, 0x01, 0xFF, 0x01, 0x00, 0xC0, 0x01, 0xFF, 0x01, 0x00, 0xC0,
  0x01, 0xFF, 0x01, 0x00, 0xC0, 0x01, 0xFF, 0x01, 0x00, 0xC0, 0x01, 0xFF,
  0x01, 0x00, 0xC0, 0x01, 0xFF, 0x01, 0x00, 0xC0, 0x01, 0xFF, 0x01, 0x00,
  0xC0, 0x01, 0x07, 0x00, 0x00, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
  0xFE, 0xFF, 0xFF, 0xFF, 0x00, };

