#include <Arduino.h>
// LMIC
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// OLED to display data available then uncomment the next line 
//define oled_output_on 

#ifdef oled_output_on
#include <U8x8lib.h>
// OLED Pins on TTGo V1 please adapt to your needs on different hardware
#define OLED_SCL 15 // GPIO 15
#define OLED_SDA 4 // GPIO 4
#define OLED_RST 16 // GPIO 16
// defines the display type  we use
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST);
#endif

// LoRa Pins check out the datasheet of your controller to adapt this pins to your needs. This is for TTGo V1
#define LoRa_RST 14 // GPIO 14
#define LoRa_CS 18 // GPIO 18
#define LoRa_DIO0 26 // GPIO 26
#define LoRa_DIO1 33 // GPIO 33
#define LoRa_DIO2 32 // GPIO 32


// LoRaWAN NwkSKey, network session key, fill with your own

static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key

static const u1_t PROGMEM APPSKEY[16] ={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


// LoRaWAN end-device address (DevAddr)

static const u4_t DEVADDR =0x00000000;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//Ultrasonic Sensor Nr1 connection to Mikrocontroller TTGo V1

int u1_trigPin=13;  //connect Pin 13 with Trigger
int u1_echoPin=12;  //connect Pin 12 with Echo
long u1_duration=0;
long u1_cm=0;
int u1_counter;
bool u1_schrankefrei = true;

//Ultrasonic Sensor Nr2 connection to Mikrocontroller TTGo V1
int u2_trigPin=2;
int u2_echoPin=17;
long u2_duration=0;
long cm2=0;
int u2_counter;
bool u2_schrankefrei = true;

int volleBreite=300; //set physical maximum of your counting distance
unsigned long time_to_react = 500; // waiting time in ms after interruption of sensor 1 to see interruption at sensor 2

unsigned long lastprint=millis();
unsigned long abort_time; //gets filled with millis()+time_to_react when 1st sensor reacts
static char mydata[14 + 1]; // "Packet = " + max(65536)

#ifdef oled_output_on
static char abstand1[15];
static char abstand2[15];
#endif

//static uint16_t packetNumber = 0;
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1800;
// Pin mapping
const lmic_pinmap lmic_pins = {
.nss = LoRa_CS,
.rxtx = LMIC_UNUSED_PIN,
.rst = LoRa_RST,
.dio = { LoRa_DIO0, LoRa_DIO1, LoRa_DIO2 },
};
void do_send(osjob_t* j){

//put incoming and leaving people in an array
static uint16_t payloadA[2];
//for network testing replace the counter variables with an integer.     
payloadA[0] = u1_counter;
payloadA[1] = u2_counter;
// Check if there is not a current TX/RX job running
if (LMIC.opmode & OP_TXRXPEND) {
Serial.println(F("OP_TXRXPEND, not sending"));
} else {
// Prepare upstream data transmission at the next possible time.
//send data
sprintf(mydata, "sendep1 = %5u", u2_counter);
LMIC_setTxData2(1, (xref2u1_t)payloadA, sizeof(payloadA), 0);
Serial.println(F("Packet queued"));
// packetNumber++;
//set counter to zero after sending data. 
u1_counter=0;
u2_counter=0;
}
// Next TX is scheduled after TX_COMPLETE event.
}
void onEvent(ev_t ev) {
Serial.print(os_getTime());
Serial.print(": ");
switch(ev) {
case EV_SCAN_TIMEOUT:
Serial.println(F("EV_SCAN_TIMEOUT"));
break;
case EV_BEACON_FOUND:
Serial.println(F("EV_BEACON_FOUND"));
break;
case EV_BEACON_MISSED:
Serial.println(F("EV_BEACON_MISSED"));
break;
case EV_BEACON_TRACKED:
Serial.println(F("EV_BEACON_TRACKED"));
break;
case EV_JOINING:
Serial.println(F("EV_JOINING"));
break;
case EV_JOINED:
Serial.println(F("EV_JOINED"));
break;
case EV_RFU1:
Serial.println(F("EV_RFU1"));
break;
case EV_JOIN_FAILED:
Serial.println(F("EV_JOIN_FAILED"));
break;
case EV_REJOIN_FAILED:
Serial.println(F("EV_REJOIN_FAILED"));
break;
case EV_TXCOMPLETE:
Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
if (LMIC.txrxFlags & TXRX_ACK)
Serial.println(F("Received ack"));
if (LMIC.dataLen) {
Serial.println(F("Received "));
Serial.println(LMIC.dataLen);
Serial.println(F(" bytes of payload"));
}
// Schedule next transmission
os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
break;
case EV_LOST_TSYNC:
Serial.println(F("EV_LOST_TSYNC"));
break;
case EV_RESET:
Serial.println(F("EV_RESET"));
break;
case EV_RXCOMPLETE:
// data received in ping slot
Serial.println(F("EV_RXCOMPLETE"));
break;
case EV_LINK_DEAD:
Serial.println(F("EV_LINK_DEAD"));
break;
case EV_LINK_ALIVE:
Serial.println(F("EV_LINK_ALIVE"));
break;
default:
Serial.println(F("Unknown event"));
break;
}
}
void setup()
{
// array mydata helps to display data on oled
sprintf( mydata,"rein= %5u", u1_counter); 

Serial.begin(9600); //115200
Serial.println(F("Starting System"));

#ifdef oled_output_on
// set up the display
u8x8.begin();
u8x8.setFlipMode(1);
u8x8.setPowerSave(0);
#endif

pinMode(u1_trigPin,OUTPUT);
pinMode(u1_echoPin,INPUT);
pinMode(u2_trigPin,OUTPUT);
pinMode(u2_echoPin,INPUT);
// LMIC init
os_init();
// Reset the MAC state. Session and pending data transfers will be discarded.
LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
// On AVR, these values are stored in flash and only copied to RAM
// once. Copy them to a temporary buffer here, LMIC_setSession will
// copy them into a buffer of its own again.
uint8_t appskey[sizeof(APPSKEY)];
uint8_t nwkskey[sizeof(NWKSKEY)];
memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
// If not running an AVR with PROGMEM, just use the arrays directly
LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

// Set up the channels used by the Things Network, which corresponds
// to the defaults of most gateways. Without this, only three base
// channels from the LoRaWAN specification are used, which certainly
// works, so it is good for debugging, but can overload those
// frequencies, so be sure to configure the full frequency range of
// your network here (unless your network autoconfigures them).
// Setting up channels should happen after LMIC_setSession, as that
// configures the minimal channel set.
// NA-US channels 0-71 are configured automatically
LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band

// disable channels (only use channel 0 - 868.1 MHz - for my single channel gateway!!!)
for (int channel = 1; channel <= 8; channel++) {
LMIC_disableChannel(channel);
}

// Disable link check validation
LMIC_setLinkCheckMode(0);

// TTN uses SF9 for its RX2 window.
LMIC.dn2Dr = DR_SF9;

// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
LMIC_setDrTxpow(DR_SF7, 14);

// Start job
do_send(&sendjob);

  
/*
#ifdef oled_output_on
u8x8.setFont(u8x8_font_8x13_1x2_f);
u8x8.drawString(0, 0, " rein raus");
sprintf( mydata,"%5u %5u", u1_counter, u2_counter);
u8x8.drawString(0, 2, mydata);
#endif
*/
}

//get data ultrasonic 1
int read_ultra1()
{
digitalWrite(u1_trigPin, LOW);
delayMicroseconds(5);
digitalWrite(u1_trigPin, HIGH);
delayMicroseconds(20);
digitalWrite(u1_trigPin, LOW);
u1_duration = pulseIn(u1_echoPin, HIGH);
u1_cm = (u1_duration/2) / 29.1;
return u1_cm; 
}

//get data ultrasonic 2
int read_ultra2()
{ 
digitalWrite(u2_trigPin, LOW);
delayMicroseconds(5);
digitalWrite(u2_trigPin, HIGH);
delayMicroseconds(20);
digitalWrite(u2_trigPin, LOW);
u2_duration = pulseIn(u2_echoPin, HIGH);
cm2 = (u2_duration/2) / 29.1;
return cm2;
}

//count people
void count()
{ 
int x=read_ultra2();
int y=read_ultra1();

//display movement  
#ifdef oled_output_on
if ((millis()-lastprint)>500)
{
u8x8.setFont(u8x8_font_8x13_1x2_f);

u8x8.drawString(0, 0, " rein raus");
sprintf( mydata,"%5u %5u", u1_counter, u2_counter);
u8x8.drawString(0, 2, mydata);

sprintf(abstand1, " U1: %5ucm", y);
u8x8.drawString(0, 4, abstand1);

sprintf(abstand2, " U2: %5ucm", x);
u8x8.drawString(0, 6, abstand2);

lastprint=millis();
}
#endif
//ultrasonic 1 interrupted
if (y<=volleBreite && u1_schrankefrei==true)
{
Serial.println("bewegung1");
abort_time=millis()+time_to_react;
u1_schrankefrei=false; //ultrasonic 1 is interupted 
bool bewegung1 = true; 
while(bewegung1 == true)
{
int s2=read_ultra2();
//die Schleife wartet darauf, dass s2 wackelt aber für immer soll es auch nicht warten, also else gut überlegen
if(s2<volleBreite)
{
Serial.println("rein");
u1_counter++; //wenn es wackelt 
bewegung1 = false; //schleife abbrechen
#ifdef oled_output_on
u8x8.clearDisplay();
u8x8.setFont(u8x8_font_open_iconic_arrow_8x8);
u8x8.drawString(0, 0, "RR");
#endif
delay(100);
#ifdef oled_output_on
u8x8.clearDisplay();
#endif

break; 
}

if(millis()>abort_time)
{
bewegung1 = false;
Serial.println("zu lange auf s2 gewartet");
break;
} 
}
}
if(y >= volleBreite)
{
u1_schrankefrei = true;
}
//Bewegung an Schranke 2
if(x<=volleBreite && u2_schrankefrei==true)
{ 
abort_time=millis()+time_to_react;
u2_schrankefrei=false; //die Schranke2 ist besetzt
Serial.println("bewegung2");
bool bewegung2 = true;
while(bewegung2 == true)
{
int s1=read_ultra1();
//die Schleife wartet darauf, dass s2 wackelt aber für immer soll es auch nicht warten, also else gut überlegen
if(s1<volleBreite)
{
Serial.println("raus");
u2_counter++; //count a leaving person
bewegung2 = false; //leave loop

#ifdef oled_output_on
u8x8.clearDisplay();
u8x8.setFont(u8x8_font_open_iconic_arrow_8x8);
//u8x8.drawGlyph(0,0,81);
u8x8.drawString(0, 0, "QQ");
#endif
delay(100);
#ifdef oled_output_on
u8x8.clearDisplay();
#endif
break;
}
if(millis()>abort_time)
{
bewegung2 = false;
Serial.println("zu lange auf s1 gewartet");
break;
} 
}
} 

if (x >= volleBreite)
{
u2_schrankefrei=true;
}
//no counting
}

void loop()
{
count();
os_runloop_once();
}

