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
 * Only 1 channel in setup
 *    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
 *
 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <unistd.h>
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

//
// Unions needed for message construction

union u_latitude {
    float         flat;
    unsigned char clat[4];
};

union u_longitude {
    float         flon;
    unsigned char clon[4];
};

union u_speed {
    float         fspeed;
    unsigned char cspeed[4];
};

union u_altitude {
    float         falt;
    unsigned char calt[4];
};

union u_course  {
    float         fcourse;
    unsigned char ccourse[4];
};

union u_datetime {
    unsigned long int idatetime;
    unsigned char cdatetime[8];
};


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

static void gpsdump( float latitude, float longitude, float speed, float altitude, float course ) {
    logfd = fopen("/tmp/gps.log", "a+");
    fprintf(logfd, "%lf %lf %lf %lf %lf\n", latitude, longitude, speed, altitude, course);
    fflush(logfd);
    fclose(logfd);
    fprintf(stdout, "%lf %lf %lf %lf %lf\n", latitude, longitude, speed, altitude, course);
}

static void do_send(osjob_t* j){

    fprintf(stdout,"do_send\n");
    time_t t=time(NULL);
    fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(&t));

    // Show TX channel (channel numbers are local to LMIC)
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
        fprintf(stdout, "OP_TXRXPEND, not finished, so not sending\n");
        return;
    }
    char mydata[18];
    unsigned long int age, hdop, cnt;
    int year, month, day, hour, minute, second, hundredths;
    u_latitude lat;
    u_longitude lon;
    u_altitude alt;
    u_course course;
    u_speed speed;
    u_datetime datetime;

    alt.falt = 1000000.00;
    cnt = 0;
    while (( alt.falt > 900000.00 ) and ( cnt < 10 )) {
        fprintf(stdout, "cnt=%d\n", cnt);
        // get GPS data
        gps_location(&gpsdata);
        fprintf(stdout, "got data\n");

        lat.flat = gpsdata.latitude;
        lon.flon = gpsdata.longitude;
        speed.fspeed = gpsdata.speed;
        alt.falt = gpsdata.altitude;
        course.fcourse = gpsdata.course;

        //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
        //hdop = gps.hdop();

        year = 2019;
        month = 5;
        day = 19;
        hour = 16;
        minute = 0;
        hdop = 0; //gps.hdop();
        sleep(cnt);
        cnt++;
    }

    if (( alt.falt < 900000.00 ) && ( alt.falt > 0.00 )) {
        // pack date in an integer
        gpsdump( lat.flat, lon.flon, speed.fspeed, alt.falt, course.fcourse );

        datetime.idatetime = year - 2000;
        datetime.idatetime = (datetime.idatetime * 100) + month;
        datetime.idatetime = (datetime.idatetime * 100) + day;
        datetime.idatetime = (datetime.idatetime * 100) + hour;
        datetime.idatetime = (datetime.idatetime * 100) + minute;

        lat.flat = 10.000000;
        lon.flon = 0.000001;
        alt.falt = 5.555555;

        mydata[0] = lat.clat[0];
        mydata[1] = lat.clat[1];
        mydata[2] = lat.clat[2];
        mydata[3] = lat.clat[3];

        mydata[4] = lon.clon[0];
        mydata[5] = lon.clon[1];
        mydata[6] = lon.clon[2];
        mydata[7] = lon.clon[3];
 
        mydata[8] = alt.calt[0];
        mydata[9] = alt.calt[1];
        mydata[10] = alt.calt[2];
        mydata[11] = alt.calt[3];

        mydata[12] = datetime.cdatetime[0];
        mydata[13] = datetime.cdatetime[1];
        mydata[14] = datetime.cdatetime[2];
        mydata[15] = datetime.cdatetime[3];
 
        mydata[16] = hdop;


        mydata[17]='\0';
        fprintf(stdout, "Send gps data\n");
    }
    else
    {
        fprintf(stdout, "No gps data\n");
    }
    for (int i = 0; i< 17; i++ ){
        mydata[i] = i;
    }

    // no payload if port 0 is defined
    fprintf(stdout,"LMIC_setTxData2 %s\n", (char*)&mydata);
    LMIC_setTxData2(1, (xref2u1_t) &mydata, sizeof(mydata), 0);
}

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

            // Schedule a timed job to run at the given timestamp (absolute system time)
            // every 2 minutes
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(30), do_send);
            break;
         default:
            break;
    }
}


void setup() {
    gps_init();

    // LMIC init
    wiringPiSetup();

    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Set static session parameters. Instead of dynamically establishing a session 
    // by joining the network, precomputed session parameters are be provided.
    LMIC_setSession (0x1, DEVADDR, (u1_t*)NWKSKEY, (u1_t*)APPSKEY);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_disableChannel(1);
    LMIC_disableChannel(2);
    LMIC_disableChannel(3);
    LMIC_disableChannel(4);
    LMIC_disableChannel(5);
    LMIC_disableChannel(6);
    LMIC_disableChannel(7);
    LMIC_disableChannel(8);
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
}

void loop() {
    do_send(&sendjob);
    sleep(2); // wait for first message to go

    while(1) {
        os_runloop();
//        os_runloop_once();
    }
}

int main() {
    setup();

    while (1) {
        loop();
    }
    return 0;
}

