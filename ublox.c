
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "ublox.h"
#include "../../pins.h"

// Local functions
void OnUartIRQ();
void GetUbloxDataQuick();
void GetUbloxData();
bool WriteToUblox(const unsigned char buffer[], uint8_t length_array, uint8_t line_end);
bool UbloxGoodWrite();
void UbloxClearUART();

// NAV POSLLH, STATUS, VELNED all on  (iForce2D GPS video)
const unsigned char UBLOX_MSGS[] = {
  // Disable NMEA
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-nav_LLH off
//  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
//  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on
 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x01,0x00,0x00,0x00,0x00,0x23,0x2E, //NAV-VELNED on

  // Rates
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

// STANDARD ENGINE CONFIG, EXCEPT FOR <2G FLIGHT MODE (derived from ublox micro centre)
const unsigned char UBLOX_NAV_ENGINE[] = {
 0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x07,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,
 0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,
 0x00,0x00,0x00,0x00,0x00,0x00,0x4E,0xFD,
};

const unsigned char UBX_HEADER[]        = {0xB5,0x62};
const unsigned char NAV_POSLLH_HEADER[] = {0x01,0x02};
const unsigned char NAV_STATUS_HEADER[] = {0x01,0x03};
const unsigned char NAV_VELNED_HEADER[] = {0x01,0x12};


// Global variables
bool UART_NEW_DATA = false; // True if new data arrived. This is EXTERN in header (global across files!)

// Function wide variables
struct GPS_DATA gps;
uart_inst_t* uart_id;
union GPS_UNION { // Join multiple messages, used as we don't know which one we are receiving at the beginning.
  struct NAV_POSLLH navPosllh;
  struct NAV_STATUS navStatus;
  struct NAV_VELNED navVelNED;
} ubx_msg;



// Sets up Ublox device, returns false is successful.
bool gps_setup(uart_inst_t* uart_ID, uint8_t uart_tx_pin, uint8_t uart_rx_pin) {
	uart_id = uart_ID;
	
	// Initialise UART 
	gpio_set_function(uart_tx_pin, GPIO_FUNC_UART);
	gpio_set_function(uart_rx_pin, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    //uart_set_hw_flow(uart_id, false, false);

	// ~~~ GPS CONFIG ~~~
	// Set up I2C UBlox GPS
	printf("Ublox6M -> ");
	absolute_time_t start_time = get_absolute_time();

	// Only way to check if UART (GPS) is alive, is to watch for interrupt
	while(uart_is_readable(uart_id) == false); { 
		if(absolute_time_diff_us(start_time, get_absolute_time()) > 2000000) { // 2 seconds we wait
			printf("ERR\n");
			return(true);
		}
	}

	// Write the rest of the config, with ack-rec
	WriteToUblox(UBLOX_NAV_ENGINE, sizeof(UBLOX_NAV_ENGINE), sizeof(UBLOX_NAV_ENGINE));
	WriteToUblox(UBLOX_MSGS, sizeof(UBLOX_MSGS), 16);
	
	
	if(uart_is_readable_within_us(uart_id,100000) != true) {
		printf("ERR\n");
		return(true); 
	}
	printf("OK\n");

	// ~~~ IRQ handler ~~~
	printf("GPS IRQ -> ");
	// Select correct interrupt for the UART we are using
    int UART_IRQ = uart_id == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, OnUartIRQ);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart_id, true, false);
	printf("OK\n");

	return(false); // All good to go  :)
}


// Variable size byte array to write (upto 255), length of that array, and any line ends (len array if not)
bool WriteToUblox(const unsigned char buffer[], uint8_t length_array, uint8_t line_end) {
	uint8_t i=0;
	char rx_buff[8];
	
	// Clears any RX buffered data
	UbloxClearUART();

	// Send each config byte, only continues once confirmation achieved.
	while (i<length_array) {
		uart_putc_raw(uart_id, buffer[i]);

		if((i+1) % line_end == 0 && i != 0) { // End of a line (assume char line, true for all expect last line, doesnt matter)
			if(UbloxGoodWrite() == true) { // confirmation byte start
				i++;
			}
			else {
				// No confirmation? Resend the line.
				printf("WARN: Bad GPS write, line: %d\n",i/line_end);
				i = (i+1) - line_end;
				UbloxClearUART();
			}
		}
		else {
			i++;
		}
		//sleep_ms(1); // Allow GPS to read without overload FIFO buffer
	}
}


// Used to scan incoming data for setting write confirmation: B5,62,5,1
bool UbloxGoodWrite() {
	uint8_t rx_buff[4];
	uint8_t ublox_byte[1];
	absolute_time_t start_read = get_absolute_time();
	while(absolute_time_diff_us(start_read, get_absolute_time()) < 50000) { // 50ms
		if(uart_is_readable(uart_id) == true) { // prevent blocking read unless data present
			uart_read_blocking(uart_id,ublox_byte,1);
			rx_buff[0] = rx_buff[1];
			rx_buff[1] = rx_buff[2];
			rx_buff[2] = rx_buff[3];
			rx_buff[3] = ublox_byte[0];
			//printf("rx from gps: %x, %x, %x, %x\n",rx_buff[0],rx_buff[1],rx_buff[2],rx_buff[3]);
			if(rx_buff[0] ==0xB5 && rx_buff[1] ==0x62 && rx_buff[2] ==0x05 && rx_buff[3] ==0x01) {
				return(true); // ^ ACK-REC 
			}
			if(rx_buff[0] ==0xB5 && rx_buff[1] ==0x62 && rx_buff[2] ==0x05 && rx_buff[3] ==0x00) {
				return(false); // ^ ACK-NAK
			}
		}
	}
	return(false); // timeout with ACK-REC
}

// Used to flush FIFO buffer
void UbloxClearUART() {
	uint8_t temp_buff[1];
	while(uart_is_readable(uart_id) == true) {
		uart_read_blocking(uart_id,temp_buff,1);
	}
}


// Fuctions to return current values stored within the GPS struct (posllh/velned/status sub-structs)
float UbloxLatitude() {
	return(gps.navPosllh.lat/10000000.0f);
}
float UbloxLongitude() {
	return(gps.navPosllh.lng/10000000.0f);
}
int32_t UbloxAltitude() {
	return(gps.navPosllh.height);
}
uint8_t UbloxVerticalAcc() {
	if(gps.navPosllh.vAcc/1000 > 255) { return(255); }
 	else { return(gps.navPosllh.vAcc/1000); }
}
uint8_t UbloxHorizontalAcc() {
	if(gps.navPosllh.hAcc/1000 > 255) { return(255); }
 	else { return(gps.navPosllh.hAcc/1000); }
}
uint8_t UbloxCourseAcc() {
	if(gps.navVelNED.cAcc/1000 > 255) { return(255); }
 	else { return(gps.navPosllh.hAcc/1000); }	
}
float UbloxHeading() {
	return(gps.navVelNED.heading/10000000.0f);
}
uint16_t UbloxGroundSpeed() {
	return(gps.navVelNED.gnd_speed);
}
int16_t UbloxClimbRate() {
	return(gps.navVelNED.velD);
}


// Fires upon UART interrupt
void OnUartIRQ() {
	// Disable interrupt (re-enabled on data read complete)
	uart_set_irq_enables(uart_id, false, false);

	// Now deal with incoming data (expect many more bytes than just this one)
	// By using high speed transfer we IRQ and transfer ALL incoming data before resuming previous activity.
	GetUbloxDataQuick();

	// Enable interrupt (re-enabled on data read complete)
	uart_set_irq_enables(uart_id, true, false);
}


void CalcChecksum(uint8_t CK[2], uint8_t msg_size) {
	// Pass reference to two uint8's, these are then set using the
	// Ublox checksum algorithm: 16-Bit Fletcher Algorithm
	CK[0] = 0; CK[1] = 0;
  	for (uint8_t i = 0; i < msg_size; i++) {
    	CK[0] = CK[0] + ((unsigned char*)(&ubx_msg))[i];
    	CK[1] = CK[0] + CK[1];
  	}
}


// Compares the first two bytes of the ubx_msg struct with a specific message header.
// Returns true if the two bytes match.
bool compareMsgHeader(const unsigned char msgHeader[2]) {
	// convert ubx_message to char array and compare first two bytes
	if(((unsigned char*)(&ubx_msg))[0] == msgHeader[0] && ((unsigned char*)(&ubx_msg))[1] == msgHeader[1]) {
		return(true);
	}
	return(false);
}


/*
~~~~~~~~~~~~~ UBbox6m Read Function ~~~~~~~~~~~~~
 - Designed to work with slow 9600 baud rate.
 - Designed to only read one (or a couple) of bytes at a time, IRQs must be completed quickly!
 - Based upon IForce2D's Ublox binary algorithm: https://www.youtube.com/watch?v=TwhCX0c8Xe0
 - Message types are predefined in header file, taken from Ublox6m datasheet
*/

// Stored between algorithm calls:
int fpos = 0;
uint8_t checksum[2];
int payloadSize = 10; // temp before set to msg payload size
uint8_t currentMsgType;

// Fills the GPS struct where confirmed messages have been received
void GetUbloxDataQuick() {
	// UART is readable at-least once, due to IRQ, but we should execute quicker than incoming data
	// Therefore data packet should be broken into seperate GetUbloxDataQuick calls.
	while(uart_is_readable(uart_id) == true) {

		char c = uart_getc(uart_id);
		//printf("pos:%d,hex:%x\n",fpos,c);

		if(fpos < 2) {
			// Check for first two header constant values (0xB5, 0x62)
			if ( c == UBX_HEADER[fpos] ) {
				fpos++;
			}
			else {
				fpos = 0;
			}
		}
		else {      
		// If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
		// and we are now reading in the bytes that make up the payload.
		// Place the incoming byte into the ubx_msg struct. The position is fpos-2 because
		// the struct does not include the initial two-byte header (UBX_HEADER).
			if((fpos-2) < payloadSize) {
				((unsigned char*)(&ubx_msg))[fpos-2] = c;
			}
			fpos++;
			if (fpos == 4) {
			// We have just received the second byte of the message type header, 
			// so now we can check to see what kind of message it is.
				if(compareMsgHeader(NAV_POSLLH_HEADER)) {
					printf("inc PosLLH\n");
					currentMsgType = MT_NAV_POSLLH;
					payloadSize = sizeof(struct NAV_POSLLH);
				}
				else if(compareMsgHeader(NAV_STATUS_HEADER)) {
					printf("inc STATUS\n");
					currentMsgType = MT_NAV_STATUS;
					payloadSize = sizeof(struct NAV_STATUS);
				}
				else if(compareMsgHeader(NAV_VELNED_HEADER)) {
					printf("inc NED\n");
					currentMsgType = MT_NAV_VELNED;
					payloadSize = sizeof(struct NAV_VELNED);
				}
				else {
					printf("Unknown msg\n");
					// unknown message type, bail
					fpos = 0;
					continue;
				}
			}
			if ( fpos == (payloadSize+2) ) {
			// All payload bytes have now been received, so we can calculate the 
			// expected checksum value to compare with the next two incoming bytes.
				CalcChecksum(checksum, payloadSize);					
			}
			else if ( fpos == (payloadSize+3) ) {
			// First byte after the payload, ie. first byte of the checksum.
			// Does it match the first byte of the checksum we calculated?
				if ( c != checksum[0] ) {
					fpos = 0;
					printf("bad ck1\n");
				}
			}
			else if ( fpos == (payloadSize+4) ) {
			// Second byte after the payload, ie. second byte of the checksum.
			// Does it match the second byte of the checksum we calculated?
				fpos = 0; // Reset regardless of match
				if ( c == checksum[1] ) {
					// Checksum matches, we have a valid message.
					// Append GPS data struct with good data
					if(currentMsgType == MT_NAV_POSLLH) {
						printf("POSLLH\n");
						gps.navPosllh = ubx_msg.navPosllh;
						//printf("LLH: ");
					}
					else if (currentMsgType == MT_NAV_STATUS) {
						printf("STATUS\n");
						gps.navStatus = ubx_msg.navStatus;
						//printf("STATUS: ");
					}
					else if (currentMsgType == MT_NAV_VELNED) {
						printf("VELNED\n");
						gps.navVelNED = ubx_msg.navVelNED;
					}
					// Set flag
					UART_NEW_DATA = true;
				}
				else {
					printf("bad ck2\n");
				}

			}
			else if ( fpos > (payloadSize+4) ) {
				// We have now read more bytes than both the expected payload and checksum 
				// together, so something went wrong. Reset to beginning state and try again.
				printf("Too many bytes\n");
				fpos = 0;
			}
		}
	}
}



// // Fills the GPS struct where confirmed messages have been received
// void GetUbloxData() {
// // IForce2D ublox binary algorithm: https://www.youtube.com/watch?v=TwhCX0c8Xe0
// 	int fpos = 0;
// 	uint8_t checksum[2];
//   	int payloadSize = 10; // temp before set to msg payload size
// 	// Message type set to none
//   	static uint8_t currentMsgType = MT_NONE;
// 	uint8_t counter;
	
// 	while(uart_is_readable_within_us(uart_id, 1000) == true) {

// 		char c = uart_getc(uart_id);
// 		//printf("pos:%d,hex:%x\n",fpos,c);

// 		if(fpos < 2) {
// 			// Check for first two header constant values (0xB5, 0x62)
// 			if ( c == UBX_HEADER[fpos] ) {
// 				fpos++;
// 			}
// 			else {
// 				fpos = 0;
// 			}
// 		}
// 		else {      
// 		// If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
// 		// and we are now reading in the bytes that make up the payload.
// 		// Place the incoming byte into the ubx_msg struct. The position is fpos-2 because
// 		// the struct does not include the initial two-byte header (UBX_HEADER).
// 			if((fpos-2) < payloadSize) {
// 				((unsigned char*)(&ubx_msg))[fpos-2] = c;
// 			}
// 			fpos++;
// 			if (fpos == 4) {
// 			// We have just received the second byte of the message type header, 
// 			// so now we can check to see what kind of message it is.
// 				if(compareMsgHeader(NAV_POSLLH_HEADER)) {
// 					printf("inc PosLLH\n");
// 					currentMsgType = MT_NAV_POSLLH;
// 					payloadSize = sizeof(struct NAV_POSLLH);
// 				}
// 				else if(compareMsgHeader(NAV_STATUS_HEADER)) {
// 					printf("inc STATUS\n");
// 					currentMsgType = MT_NAV_STATUS;
// 					payloadSize = sizeof(struct NAV_STATUS);
// 				}
// 				else if(compareMsgHeader(NAV_VELNED_HEADER)) {
// 					printf("inc NED\n");
// 					currentMsgType = MT_NAV_VELNED;
// 					payloadSize = sizeof(struct NAV_VELNED);
// 				}
// 				else {
// 					printf("Unknown msg\n");
// 					// unknown message type, bail
// 					fpos = 0;
// 					continue;
// 				}
// 			}
// 			if ( fpos == (payloadSize+2) ) {
// 			// All payload bytes have now been received, so we can calculate the 
// 			// expected checksum value to compare with the next two incoming bytes.
// 				CalcChecksum(checksum, payloadSize);					
// 			}
// 			else if ( fpos == (payloadSize+3) ) {
// 			// First byte after the payload, ie. first byte of the checksum.
// 			// Does it match the first byte of the checksum we calculated?
// 				if ( c != checksum[0] ) {
// 					fpos = 0;
// 					printf("bad ck1\n");
// 				}
// 			}
// 			else if ( fpos == (payloadSize+4) ) {
// 			// Second byte after the payload, ie. second byte of the checksum.
// 			// Does it match the second byte of the checksum we calculated?
// 				fpos = 0; // Reset regardless of match
// 				if ( c == checksum[1] ) {
// 					// Checksum matches, we have a valid message.
// 					// Append GPS data struct with good data
// 					if(currentMsgType == MT_NAV_POSLLH) {
// 						printf("POSLLH\n");
// 						gps.navPosllh = ubx_msg.navPosllh;
// 						//printf("LLH: ");
// 					}
// 					else if (currentMsgType == MT_NAV_STATUS) {
// 						printf("STATUS\n");
// 						gps.navStatus = ubx_msg.navStatus;
// 						//printf("STATUS: ");
// 					}
// 					else if (currentMsgType == MT_NAV_VELNED) {
// 						printf("VELNED\n");
// 						gps.navVelNED = ubx_msg.navVelNED;
// 					}
// 				}
// 				else {
// 					printf("bad ck2\n");
// 				}

// 			}
// 			else if ( fpos > (payloadSize+4) ) {
// 				// We have now read more bytes than both the expected payload and checksum 
// 				// together, so something went wrong. Reset to beginning state and try again.
// 				printf("Too many bytes\n");
// 				fpos = 0;
// 			}
// 		}
// 	}
// }
