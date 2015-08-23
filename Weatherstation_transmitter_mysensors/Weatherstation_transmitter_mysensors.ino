/*
  Wireless weather station transmitter - An Arduino sketch to send weather
  station data obtained from a MySensors network to an Alecto DKW-2012 
  wireless weather station display.

  Created by Ivo Pullens, Emmission, 2015 -- www.emmission.nl

  Based on the work of SevenW (https://github.com/SevenW)

  Library dependencies:
  * MySensors 1.5 (https://github.com/mysensors/Arduino/tree/master)
  * JeeLib        (https://github.com/jcw/jeelib)
  * Time          (https://github.com/PaulStoffregen/Time)
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define RF69_COMPAT 1 // define this to use the RF69 driver i.s.o. RF12 

#include <JeeLib.h>
#include <SPI.h>
#include <MySensor.h>  
#include <Time.h>
#include "Weatherstation_types.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

const uint8_t preamble    = 0xff;
const uint8_t id          = 216;
const uint8_t mysns_id    = 80;

#define MAX_PACKET_SIZE_BYTES (11)
#define SERIAL_BAUD           (115200)
#define LED_PIN               (9)
#define MYSNS_RF24_CE_PIN     (7)
#define MYSNS_RF24_CS_PIN     (8)

#define RETRY_TIMEOUT_MS          (5UL*1000UL)         // 5 seconds
#define CLOCK_UPDATE_TIMEOUT_MS   (60UL*60UL*1000UL)   // 1 hour
#define DISPLAY_UPDATE_TIMEOUT_MS (1UL*60UL*1000UL)    // 1 minute  (display blanks after 6:30 without data)

static const mysensor_data sensors[] = { V_TEMP, V_HUM, V_WIND, V_GUST, V_RAIN };

MyTransportNRF24 radio(MYSNS_RF24_CE_PIN, MYSNS_RF24_CS_PIN, RF24_PA_LEVEL);
MyHwATMega328 hw; 
MySensor gw(radio, hw);



static void activityLed (bool on) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

static void print_hex( uint8_t v )
{
  Serial.print(v >> 4, HEX);
  Serial.print(v & 0x0F, HEX);
}

static void print_hex( uint8_t const * data, uint8_t len )
{
  while (len-- > 0)
  {
    print_hex(*data++);
    Serial.print(" ");
  }
}

#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_DATAMOD         0x02
#define REG_BRMSB           0x03
#define REG_FRFMSB          0x07
#define REG_IRQFLAGS1       0x27
#define REG_IRQFLAGS2       0x28
#define REG_PREAMPSIZE      0x2D
#define REG_SYNCCONFIG      0x2E
#define REG_PKTCONFIG1      0x37
#define REG_PAYLOADLEN      0x38

#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x04
#define MODE_RECEIVER       0x10
#define MODE_TRANSMITTER    0x0C

#define IRQ1_MODEREADY      0x80
#define IRQ1_RXREADY        0x40

#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_FIFOOVERRUN    0x10
#define IRQ2_PACKETSENT     0x08
#define IRQ2_PAYLOADREADY   0x04

static void writeReg (uint8_t addr, uint8_t value) {
  RF69::control(addr | 0x80, value);
}

static uint8_t readReg (uint8_t addr) {
  return RF69::control(addr, 0);
}

static void setMode (uint8_t mode) {
  writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | mode);
}

static void setFrequency (uint32_t hz) {
  // accept any frequency scale as input, including KHz and MHz
  // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
  while (hz < 100000000)
    hz *= 10;
  uint32_t frf = (hz << 2) / (32000000L >> 11);
  writeReg(REG_FRFMSB, frf >> 10);
  writeReg(REG_FRFMSB + 1, frf >> 2);
  writeReg(REG_FRFMSB + 2, frf << 6);
}

inline uint16_t swap16(const uint16_t v)
{
  return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF);
}

static void sendPacket(uint8_t const * data, int len, uint8_t repeat = 3)
{
  print_hex(data, len); Serial.println("");

  while (repeat-- > 0)
  {
    // Start with 1 byte 'silence'
    writeReg(REG_FIFO, 0);
    
    uint16_t tx = 0;
    uint8_t b = 16;
    for (uint8_t i = 0; i < len; ++i)
    {
      uint8_t mask = 0x80;
      do
      {
        uint8_t code;
        uint8_t l;
        if (data[i] & mask)
        {
          // Send 1, encode as 100
          code = 0b100;
          l = 3;        
        }
        else
        {
          // Send 0, encode as 11100
          code = 0b11100;
          l = 5;
        }
        b -= l;
        tx |= uint16_t(code) << b;
  
        if (b <= 7)
        {
          tx = swap16(tx);
          writeReg(REG_FIFO, tx);
          tx &= 0xFF00;
          b += 8;
        }
        mask >>= 1;
      } while ( mask != 0 );
    }
    if (b != 16)
    {
      // Send remaining bits
      writeReg(REG_FIFO, tx >> 8);
    }
    
    // send after filling FIFO (RFM should be in sleep or standby)
    activityLed(true);
    setMode(MODE_TRANSMITTER);
   
    while ((readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0) {};
  
    setMode(MODE_STANDBY);
    activityLed(false);

    delay(100);
  }
}

static void ook_initialize()
{
  writeReg(REG_OPMODE, 0x00);         // OpMode = sleep
  writeReg(REG_DATAMOD, 0x08);        // DataModul = packet mode, OOK
  writeReg(REG_PREAMPSIZE, 0x00);     // PreambleSize = 0 NO PREAMBLE
  writeReg(REG_SYNCCONFIG, 0x00);     // SyncConfig = sync OFF
  writeReg(REG_PKTCONFIG1, 0x80);     // PacketConfig1 = variable length, advanced items OFF
  writeReg(REG_PAYLOADLEN, 0x00);     // PayloadLength = 0, unlimited
  setFrequency(868280);

  const uint32_t br = 32000000L / 2000;   // 2000bit/s
  writeReg(REG_BRMSB, br >> 8);
  writeReg(REG_BRMSB + 1, br);
}

// Ref: https://github.com/SevenW/wirelessweather/blob/master/WeatherStationFSKv2/WeatherStationFSKv2.ino
static uint8_t crc8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;

  // Indicated changes are from reference CRC-8 function in OneWire library
  while (len--) {
    uint8_t inbyte = *data++;
    uint8_t i;
    for (i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
      crc <<= 1; // changed from right shift
      if (mix) crc ^= 0x31;// changed from 0x8C;
      inbyte <<= 1; // changed from right shift
    }
  }
  return crc;
}

void sendSensorPacket( const uint8_t id, const SensorData& data )
{
  uint8_t msg[11];
  Serial.print("Temp:"); Serial.print(data.temp_c); 
  Serial.print("  Hum:"); Serial.print(data.hum_rh); 
  Serial.print("  Wind:"); Serial.print(data.wind_kmh); 
  Serial.print("  Gust:"); Serial.print(data.gust_kmh); 
  Serial.print("  Rain:"); Serial.print(data.rain_cum_mm); 
  Serial.print("  Dir:"); Serial.print((uint8_t)data.wind_dir); 
  Serial.println("");
    
  msg[0] = preamble;
  msg[1] = (MsgType_Sensor << 4) | (id >> 4);
  uint16_t tmp = (data.temp_c*10)+400.5;
  msg[2] = (id << 4) | ((tmp >> 8) & 0x03);
  msg[3] = tmp;
  msg[4] = data.hum_rh + 0.5;
  msg[5] = (data.wind_kmh/1.07917) + 0.5;
  msg[6] = (data.gust_kmh/1.07917) + 0.5;
  uint16_t rain = (data.rain_cum_mm / 0.3) + 0.5;
  msg[7] = rain >> 8;
  msg[8] = rain;
  msg[9] = data.wind_dir;
  msg[10] = crc8(&msg[1], 9);

  sendPacket(msg, ARRAY_SIZE(msg));
}

static uint8_t dec2bcd( const uint8_t v )
{
  return ((v / 10) << 4) | (v % 10);
}

void sendDcfPacket( const uint8_t id, const DcfData& data )
{
  uint8_t msg[11];
  msg[0] = preamble;
  msg[1] = (MsgType_DCF << 4) | (id >> 4);
  msg[2] = (id << 4) | 0x0a;
  msg[3] = dec2bcd(data.hour);// | 0x80;
  msg[4] = dec2bcd(data.min);
  msg[5] = dec2bcd(data.sec);
  msg[6] = dec2bcd(data.year);
  msg[7] = dec2bcd(data.mon);// | 0x40;
  msg[8] = dec2bcd(data.day);
  msg[9] = 0x45;
  msg[10] = crc8(&msg[1], 9);

  sendPacket(msg, ARRAY_SIZE(msg));
}

static bool time_received;
void receiveTime(unsigned long time)
{
  Serial.print("Rx time: "); Serial.println(time);
  setTime(time);
  time_received = true;
}

uint8_t get_sns_idx( const uint8_t sns )
{
  for (uint8_t i = 0; i < ARRAY_SIZE(sensors); ++i)
  {
    if (sensors[i] == sns)
      return i;
  }
  return 0;
}


//static unsigned long last_update;
static SensorData sns_data;
static bool sns_forcetx = false;    // Trigger immediate send of sensor data
static uint8_t sns_updated = 0;     // Keeps track of which sensor was updated since last mark.
void incomingMessage(const MyMessage &message)
{
  float v = atof( message.data );
  switch (message.type)
  { 
    case V_TEMP: sns_data.temp_c = v;      break;
    case V_HUM:  sns_data.hum_rh = v;      break;
    case V_WIND: sns_data.wind_kmh = v;    break;
    case V_GUST: sns_data.gust_kmh = v;    break;
    case V_RAIN: sns_data.rain_cum_mm = v; break;  
    default:
      return;
  }
  sns_updated |= 1 << get_sns_idx(message.type);
}

void setup () {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Weatherstation transmitter");

  rf12_initialize(0x81, RF12_868MHZ, 212, 1600);

  ook_initialize();

  gw.begin( incomingMessage, mysns_id );

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Weatherdisplay", "1.0");
}

void statemachine_time(void)
{
  static uint8_t state = 0;
  static boolean timeReceived = false;
  static unsigned long timeout;

  switch (state)
  {
    case 0:
      // Request time from controller. 
      Serial.println("Receive time");
      time_received = false;
      gw.requestTime(receiveTime);
      timeout = millis() + RETRY_TIMEOUT_MS;
      state = 1;
      break;
    case 1:
      if (time_received)
      {
        Serial.println("Got time");
        time_t t = now();
        DcfData dcf = { hour(t), minute(t), second(t), year(t)-2000, month(t), day(t) };
        sendDcfPacket( id, dcf );
        timeout = millis() + CLOCK_UPDATE_TIMEOUT_MS;
        state = 2;
      }
      else if (millis() >= timeout)
      {
        Serial.println("Failed to retrieve time. Retry.");
        state = 0;
      }
      break;
    case 2:
      // Wait until it's time to update the clock again.
      if (millis() >= timeout)
      {
        state = 0;
      }
      break;
  }
}

void statemachine_snsrx(void)
{
  static uint8_t state  = 0;
  static uint8_t idx_sns;
  static unsigned long timeout;
    
  switch (state)
  {
    case 0:
      // First request all values once and wait for a response to be received.
      // When finished, the gateway knows which values we're interested in and thus
      // has created a subscription for each of our values.
      idx_sns = 0;
      sns_updated = 0;
      state = 1;
      break;  
    case 1:
      if (idx_sns >= ARRAY_SIZE(sensors))
      {
        // All sensor values have been updated & subscribed to
        Serial.println("All sensors subscribed");
        state = 3;
      } else {
        // Update next sensor value
        Serial.print("Request update (subscribe to) "); Serial.println(sensors[idx_sns]);
        gw.request( 0, sensors[idx_sns] ); 
        timeout = millis() + RETRY_TIMEOUT_MS;
        state = 2;
      }
      break;
    case 2:
      // Wait until we receive a value for the sensor we've currently requested
      // (and not a value pushed for a sensor we've subscribed to before)
      if (sns_updated & (1 << idx_sns))
      {
        Serial.println("Got data (subscribed)");
        ++idx_sns;
        state = 1;
      } else if (millis() >= timeout) {
        // No value received. Try again.
        Serial.println("No data, retry");
        state = 1;
      }
      break;
    case 3:
      // Send sensor data to display.
      sns_forcetx = true;
      // Keep track of which sensor got updated
      sns_updated = 0;
      state = 4;
      break;
    case 4:
      if (sns_updated)
      {
        // At least one sensor got udated. Wait a short while before sending,
        // as updates usually come in batches.
        timeout = millis() + RETRY_TIMEOUT_MS;  // Retry timeout seems to be a sane value
        state = 5;
      }
      break;
    case 5:
      if (millis() >= timeout) {
        state = 3;
      }
      break;
  }
}

void statemachine_snstx(void)
{
  static uint8_t state  = 0;
  static unsigned long timeout;
    
  switch (state)
  {
    case 0:
      Serial.println("Wait for sensor update, or timeout");
      timeout = millis() + DISPLAY_UPDATE_TIMEOUT_MS;
      sns_forcetx = false;
      state = 1;
      break;  
    case 1:
      if (sns_forcetx || (millis() >= timeout))
      {
        // Send sensor data to display.
        sendSensorPacket( id, sns_data );
        state = 0;
      }
  }
}

void loop () {
  gw.process();
  statemachine_time();
  statemachine_snsrx();
  statemachine_snstx();
}
