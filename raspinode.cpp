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
 * Pins set in lmic_pinmap
 *    For Dragino Lora & GPS shield for Raspberry PI
 *
 * Channel set in setup
 *    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
 *
 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <wiringPi.h>
#include <lmic.h>
#include <hal.h>
#include <local_hal.h>
#include <wiringSerial.h>
extern "C" 
{
#include <gps.h>
}

// Pin mapping
lmic_pinmap pins = {
  .nss = 6,
  .rxtx = UNUSED_PIN, // Not connected on RFM92/RFM95
  .rst = 0,  // Needed on RFM92/RFM95
  .dio = {7,4,5}
};

// LoRaWAN configuration
// See http://thethingsnetwork.org/wiki/AddressSpace
#include "config.conf"

//
//
FILE *logfd;
loc_t gpsdata;

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
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

static osjob_t sendjob;

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

static void gpsdump( float latitude, float longitude, float speed, float altitude, float course ) {
    logfd = fopen("/tmp/gps.log", "a+");

    fprintf(logfd, "%lf %lf %lf %lf %lf\n", latitude, longitude, speed, altitude, course);
    fflush(logfd);
    fclose(logfd);
}

static void do_send(osjob_t* j){
    time_t t=time(NULL);
    fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(&t));
    // Show TX channel (channel numbers are local to LMIC)
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
        fprintf(stdout, "OP_TXRXPEND, not sending");
        return;
    }
    unsigned char mydata[18];
    unsigned long int age, hdop, cnt;
    int year, month, day, hour, minute, second, hundredths;

    float flat,flon,falt,fcourse,fspeed;

    falt = 1000000.00;
    cnt = 0;
    while (( falt > 900000.00 ) and ( cnt < 10 )) {
        fprintf(stdout, "cnt=%d\n", cnt);
        // get GPS data
        gps_location(&gpsdata);

        flat = gpsdata.latitude;
        flon = gpsdata.longitude;
        fspeed = gpsdata.speed;
        falt = gpsdata.altitude;
        fcourse = gpsdata.course;

        //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
        //hdop = gps.hdop();
        delay(2000);
        cnt++;
    }

    if (( falt < 900000.00 ) && ( falt > -1000.00 )) {
        // pack date in an integer
        gpsdump( flat, flon, fspeed, falt, fcourse );

        unsigned long int datetime = year - 2000;
        datetime = (datetime * 100) + month;
        datetime = (datetime * 100) + day;
        datetime = (datetime * 100) + hour;
        datetime = (datetime * 100) + minute;

        mydata[0] = (int)flat >> 24;
        mydata[1] = (int)flat >> 16;
        mydata[2] = (int)flat >> 8;
        mydata[3] = (int)flat;

        mydata[4] = (int)flon >> 24;
        mydata[5] = (int)flon >> 16;
        mydata[6] = (int)flon >> 8;
        mydata[7] = (int)flon;
 
        mydata[8] = (int)falt >> 24;
        mydata[9] = (int)falt >> 16;
        mydata[10] = (int)falt >> 8;
        mydata[11] = (int)falt;

        mydata[12] = datetime >> 24;
        mydata[13] = datetime >> 16;
        mydata[14] = datetime >> 8;
        mydata[15] = datetime;
 
        mydata[16] = hdop;

        mydata[17]='\0';
        int myPort = 0;  // maybe should be 1
        LMIC_setTxData2(myPort, (xref2u1_t) &mydata, sizeof(mydata), 1); 
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(20), do_send);
}

void setup() {
    fprintf(stdout,"Start setup\n");

    gps_init();

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

    fprintf(stdout,"Start send\n");
    // Start job
    do_send(&sendjob);
    fprintf(stdout,"Done  setup\n");
}

void loop() {

do_send(&sendjob);

while(1) {

  fprintf(stdout, "loop\n");
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

