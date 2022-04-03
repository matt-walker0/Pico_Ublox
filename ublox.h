#ifndef ublox_included
#define ublox_included

// Below structs defined by Ublox datasheet:
// https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf

// Lat/lon/height GPS
struct NAV_POSLLH {
  unsigned char cls; // msg class
  unsigned char id; // msg id
  unsigned short len; // len of inc. message
  unsigned long iTOW;
  long lng;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

// Status of GPS
struct NAV_STATUS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned char gpsFix;
  char flags;
  char fixStat;
  char flags2;
  unsigned long ttff;
  unsigned long msss;
};

// Speed info
struct NAV_VELNED {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long velN;
  long velE;
  long velD;
  unsigned long speed3d;
  unsigned long gnd_speed;
  long heading; // course over gnd (e-5)
  unsigned long sAcc; // speed accuracy 
  unsigned long cAcc; // course accuracy (e-5)
};

struct GPS_DATA {
  struct NAV_POSLLH navPosllh;
  struct NAV_STATUS navStatus; 
  struct NAV_VELNED navVelNED;
};

// Possible incoming messages
enum _ubxMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_VELNED,
};

bool gps_setup(uart_inst_t* uart_ID, uint8_t uart_tx_pin, uint8_t uart_rx_pin);
float UbloxLatitude();
float UbloxLongitude();
int32_t UbloxAltitude();
uint8_t UbloxVerticalAcc();
uint8_t UbloxHorizontalAcc();
uint8_t UbloxCourseAcc();
float UbloxHeading();
uint16_t UbloxGroundSpeed();
int16_t UbloxClimbRate();

//void GetStoredGPSData(struct AFCS *aircraft);
uint8_t gps_value(struct GPS_DATA *gps_data);

// This is a real global variable. Careful!
extern bool UART_NEW_DATA;

#endif