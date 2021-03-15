#include <Arduino.h>
// LMIC
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// OLED
//define oled_output_on //auskommentieren, falls OLED-Output nicht gewünscht ist

#ifdef oled_output_on
#include <U8x8lib.h>
// OLED Pins
#define OLED_SCL 15 // GPIO 15
#define OLED_SDA 4 // GPIO 4
#define OLED_RST 16 // GPIO 16
// define the display type that we use
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST);
#endif

// LoRa Pins
#define LoRa_RST 14 // GPIO 14
#define LoRa_CS 18 // GPIO 18
#define LoRa_DIO0 26 // GPIO 26
#define LoRa_DIO1 33 // GPIO 33
#define LoRa_DIO2 32 // GPIO 32


// LoRaWAN NwkSKey, network session key
//static const PROGMEM u1_t NWKSKEY[16] = { 0xB6, 0x1B, 0xCE, 0x39, 0xE8, 0x62, 0x06, 0x14, 0xBF, 0x1F, 0x74, 0x6F, 0x63, 0xB4, 0xAA, 0x02 };
//novembercount vergeben
//static const PROGMEM u1_t NWKSKEY[16] = { 0x9D, 0xC4, 0xB2, 0x88, 0x3D, 0x7D, 0xB1, 0x28, 0x0B, 0x78, 0x08, 0xBD, 0x80, 0xA5, 0x87, 0x2C };
//p_zwei vergeben
static const PROGMEM u1_t NWKSKEY[16] = { 0xC3, 0x60, 0x5C, 0x94, 0x7F, 0xDF, 0xFD, 0x53, 0x3F, 0xDE, 0x18, 0xAA, 0x2A, 0x8C, 0xC5, 0xEA };
//sensor1
//static const PROGMEM u1_t NWKSKEY[16] = { 0x18, 0x36, 0x85, 0x14, 0xB4, 0x84, 0xEE, 0x67, 0x8B, 0xF8, 0x17, 0x3E, 0x30, 0xFD, 0x2B, 0x4E };
//pax4
//static const PROGMEM u1_t NWKSKEY[16] = { 0x91, 0x9F, 0xC2, 0x92, 0x17, 0x16, 0x94, 0xD5, 0x96, 0xD3, 0xB6, 0xBA, 0xC8, 0x6D, 0x58, 0x72 };
//arthur
//static const PROGMEM u1_t NWKSKEY[16] = { 0xA2, 0x27, 0x79, 0x78, 0xF9, 0x17, 0xC7, 0xC4, 0xF2, 0x9C, 0x15, 0x1E, 0x1B, 0xAA, 0x38, 0x22 };

// LoRaWAN AppSKey, application session key
//static const u1_t PROGMEM APPSKEY[16] = { 0xE4, 0xC7, 0x92, 0x47, 0xC5, 0x30, 0x1B, 0x25, 0xD1, 0xDF, 0x76, 0x1A, 0xA3, 0x8A, 0x3C, 0x5C }; 
//novembercount
//static const u1_t PROGMEM APPSKEY[16] = { 0x9C, 0xF3, 0x12, 0xA3, 0x27, 0xE7, 0x24, 0x46, 0x96, 0x64, 0x1E, 0xF9, 0x1D, 0xD5, 0x37, 0xB0 };
//p_zwei
static const u1_t PROGMEM APPSKEY[16] ={ 0x62, 0x83, 0x1E, 0x3D, 0xDF, 0xF9, 0x31, 0xEC, 0x08, 0x03, 0x02, 0xF4, 0xBF, 0x85, 0x5D, 0xBA };
//sensor1
//static const u1_t PROGMEM APPSKEY[16] = { 0x2F, 0x31, 0xA3, 0x60, 0xF2, 0xFC, 0xC9, 0x11, 0x6E, 0xF4, 0xF0, 0xF3, 0x1E, 0x6D, 0x68, 0x4B };
//pax4
//static const u1_t PROGMEM APPSKEY[16] = { 0xC7, 0xEB, 0x74, 0x52, 0x48, 0xB3, 0x8D, 0x3C, 0x64, 0x23, 0x0D, 0x85, 0x2E, 0x91, 0x38, 0x5E };
//arthur
//static const u1_t PROGMEM APPSKEY[16] = { 0xBE, 0x27, 0x2D, 0xCE, 0x68, 0xE9, 0x3A, 0x4C, 0xFB, 0x79, 0x6F, 0xE3, 0x16, 0xAC, 0x9B, 0xFD };


// LoRaWAN end-device address (DevAddr)
//static const u4_t DEVADDR = 0x260110A8;//0x26011771 ;
//novembercount
//static const u4_t DEVADDR =0x2601394B;
//p_zwei
static const u4_t DEVADDR =0x26011B92;
//sensor1
//static const u4_t DEVADDR = 0x2601130B;
//pax4
//static const u4_t DEVADDR = 0x26011771;
//arthur
//static const u4_t DEVADDR = 0x260117AC;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//UltraschallNr1
int u1_trigPin=13;
int u1_echoPin=12;
long u1_duration=0;
long u1_cm=0;
int u1_counter;
bool u1_schrankefrei = true;

//UltraschallNr2
int u2_trigPin=2;
int u2_echoPin=17;
long u2_duration=0;
long cm2=0;
int u2_counter;
bool u2_schrankefrei = true;

int volleBreite=300; //in cm
unsigned long time_to_react = 500; //in ms

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

//Array der Daten
static uint16_t payloadA[2];
payloadA[0] = u1_counter;//zum Testen hier feste Werte einsetzen
payloadA[1] = u2_counter;//
// Check if there is not a current TX/RX job running
if (LMIC.opmode & OP_TXRXPEND) {
Serial.println(F("OP_TXRXPEND, not sending"));
} else {
// Prepare upstream data transmission at the next possible time.
//Anzahl
sprintf(mydata, "sendep1 = %5u", u2_counter);
LMIC_setTxData2(1, (xref2u1_t)payloadA, sizeof(payloadA), 0);
Serial.println(F("Packet queued"));
// packetNumber++;
//nach jedem Senden, die counter auf 0 setzen. Menschen pro 10 Minuten die reinrausgehen
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
// init packet counter
sprintf( mydata,"rein= %5u", u1_counter); //aenderung

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

void count()
{ 
int x=read_ultra2();
int y=read_ultra1();

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
//Bewegung an Schranke 1
if (y<=volleBreite && u1_schrankefrei==true)
{
Serial.println("bewegung1");
abort_time=millis()+time_to_react;
u1_schrankefrei=false; //die Schranke 1 ist besetzt 
bool bewegung1 = true; //Kontrollbedingung der while schleife
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
u2_counter++; //wenn es wackelt 
bewegung2 = false; //schleife abbrechen

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
//Serial.println("nix bewegt"); 
}

void loop()
{
count();
os_runloop_once();
}

