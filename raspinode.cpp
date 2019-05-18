/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello, world!", that
 * will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <wiringPi.h>
#include <lmic.h>
#include <hal.h>
#include <local_hal.h>
#include <wiringSerial.h>

// LoRaWAN Application identifier (AppEUI)
// Not used in this example
static const u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8]  = { 0x42, 0x42, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
static const u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
static const u1_t APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0xffffffff ; // <-- Change this address for every node!

//
//
int gpsfd;

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEi (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, NWKSKEY, 16);
}

u4_t cntr=0;
u1_t mydata[] = {"Hello, world!                               "};
static osjob_t sendjob;

// Pin mapping
lmic_pinmap pins = {
  .nss = 6,
  .rxtx = UNUSED_PIN, // Not connected on RFM92/RFM95
  .rst = 0,  // Needed on RFM92/RFM95
  .dio = {7,4,5}
};

void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          fprintf(stdout, "Event EV_TXCOMPLETE, time: %d\n", millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              fprintf(stdout, "Data Received!\n");
          }
          break;
       default:
          break;
    }
}

// fake gps
static void getGPSData() {

  unsigned long start = millis();

  fprintf(stdout, "getGPSData");

//  do
//  {
//    while (mySerial.available()) { gps.encode(mySerial.read()); }
//  } while (millis() - start < 1000); CHANGE IMPLEMENTATION
}

static void do_send(osjob_t* j){
      time_t t=time(NULL);
      fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(&t));
      // Show TX channel (channel numbers are local to LMIC)
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      fprintf(stdout, "OP_TXRXPEND, not sending");
    } else {
      unsigned char mydata[17];
      unsigned long int age, hdop, cnt;
      int year;
      int month, day, hour, minute, second, hundredths;

      float flat,flon,falt,fcourse,fkmph;

      falt = 1000000.00;
      cnt = 0;
      while (( falt > 900000.00 ) and ( cnt < 30 )) {
        //fprintf(stdout,cnt);
        fprintf(stdout," ");
        //getGPSData();
        //gps.f_get_position(&flat, &flon, &age);
        //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
        //hdop = gps.hdop();
        //falt = gps.f_altitude();
        delay(2000);
        cnt++;
      }

      /*
     if (( falt < 900000.00 ) && ( falt > -1000.00 )) {
        // pack date in an integer

        unsigned long int datetime = year - 2000;
        datetime = (datetime * 100) + month;
        datetime = (datetime * 100) + day;
        datetime = (datetime * 100) + hour;
        datetime = (datetime * 100) + minute;

        mydata[0] = flat >> 24;
        mydata[1] = flat >> 16;
        mydata[2] = flat >> 8;
        mydata[3] = flat;

        mydata[4] = flon >> 24;
        mydata[5] = flon >> 16;
        mydata[6] = flon >> 8;
        mydata[7] = flon;
 
        mydata[8] = falt >> 24;
        mydata[9] = falt >> 16;
        mydata[10] = falt >> 8;
        mydata[11] = falt;

        mydata[12] = datetime >> 24;
        mydata[13] = datetime >> 16;
        mydata[14] = datetime >> 8;
        mydata[15] = datetime;
 
        mydata[16] = hdop;
*/
        mydata[17]='\0';
        int myPort = 0;  // maybe should be 1
        LMIC_setTxData2(myPort, (xref2u1_t) &mydata, sizeof(mydata), 1);
      }
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime()+sec2osticks(20), do_send);
}

void gpsdump( float flat, float flon, float falt, float fcourse, float fkmph, unsigned long age, unsigned long datetime, unsigned long hdop )
{
  unsigned long chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
}

void setup() {
  // LMIC init
  wiringPiSetup();

  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, (u1_t*)NWKSKEY, (u1_t*)APPSKEY);

////////////////////////////////////////////////////////////////////////////////
// from original code
//  // Disable data rate adaptation
//  LMIC_setAdrMode(0);
//  
//  // Disable beacon tracking
//  LMIC_disableTracking ();
//
//  // Stop listening for downstream data (periodical reception)
//  LMIC_stopPingable();
//
////////////////////////////////////////////////////////////////////////////////

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Get data from gps
    gpsfd = serialOpen("/dev/ttyAMA0", 9600);

    // Start job
    do_send(&sendjob);
}

void loop() {

do_send(&sendjob);

while(1) {

  if  ( gpsfd == -1 ) {
    // try to connect again
    gpsfd = serialOpen("/dev/ttyAMA0", 9600);
  }

  if ( gpsfd > -1 ) {
    // get the data
    while (serialDataAvail( gpsfd ) {
      fprintf(stdout, serialGetChar( gpsfd ));
    }
  }
  os_runloop();
//  os_runloop_once();
  }
}

int main() {
  setup();

  while (1) {
    loop();
  }
  return 0;
}

