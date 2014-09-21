#define LEAN_AND_MEAN 1
#define WIFI 1
#define PID 0

#include <SPI.h>

/********** constants ************/

#define HEATER_0_PIN 2
#define SEAT_SENSOR 3
#define TEMP_0_PIN A1
#define GAS_SENSOR A2
#define TEMP_TARGET 200

#define WLAN_SSID       "Hero_Center"           // cannot be longer than 32 characters!
#define WLAN_PASS       "homeforstartups"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define WEBSITE ("wc.noun.ly")

#define DEBUG(a) Serial.print(a)
#define DEBUGLN(a) Serial.println(a)

/*********** globals for wifi ***************/
#if WIFI

//#include <SPI.h>
#include <Adafruit_CC3000.h>
#include <ccspi.h>

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   7  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

//#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
#endif
/*********** globals for screen *************/

#include "U8glib.h"

U8GLIB_NHD_C12864 u8g(13, 11, 12, 9, 8);	// SPI Com: SCK = 13, MOSI = 11, CS = 10^H2, A0 = 9, RST = 8
#define U8G_BACKLIGHT 7

/**************** Temperature ************/

#define WRITE(p,v) digitalWrite(p,v?HIGH:LOW)
#define SET_OUTPUT(p) pinMode(p, OUTPUT)
#define OVERSAMPLENR 16
#define SOFT_PWM_SCALE 0

void disable_heater()
{
  DEBUGLN("heaters off");
  WRITE(HEATER_0_PIN, 0);
}
  
#if PID

#define DEFAULT_Kp 22.0
#define DEFAULT_Ki 1.08
#define DEFAULT_Kd 114
#define PID_dT ((OVERSAMPLENR * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term

// Thermistor lookup table for Marlin
// ./createTemperatureLookupMarlin.py --rp=33000 --t1=25:65000 --t2=36:44000 --t3=101:6500.0 --num-temps=10
// Steinhart-Hart Coefficients: 0.000404613294425998, 0.000245193517270764,  1.70555946515395e-07
//#define NUMTEMPS 10
const short temptable[][2] PROGMEM = {
   {(short)(851.92*OVERSAMPLENR), 100}, // v=4.15977814909 r=6665.57688569 res=-0.277880915648 C/count
   {(short)(802.71*OVERSAMPLENR), 80}, // v=3.91947105932 r=9097.51711466 res=-0.216791023701 C/count
   {(short)(740.38*OVERSAMPLENR), 65}, // v=3.61512169793 r=12641.6170151 res=-0.173153503992 C/count
   {(short)(663.74*OVERSAMPLENR), 57}, // v=3.24089886076 r=17911.8016603 res=-0.143383639679 C/count
   {(short)(573.50*OVERSAMPLENR), 46}, // v=2.80030692097 r=25922.1127028 res=-0.124955612042 C/count
   {(short)(473.34*OVERSAMPLENR), 40}, // v=2.31124268139 r=38390.1665664 res=-0.116443231701 C/count
   {(short)(370.10*OVERSAMPLENR), 28}, // v=1.80711418898 r=58305.7962836 res=-0.117702068082 C/count
   {(short)(272.45*OVERSAMPLENR), 16}, // v=1.33032272828 r=91030.0541311 res=-0.130310829047 C/count
   {(short)(188.26*OVERSAMPLENR), 4}, // v=0.919262252789 r=146491.760376 res=-0.158510107352 C/count
   {(short)(122.11*OVERSAMPLENR), -8}  // v=0.596224613689 r=243741.34246 res=-0.211133208338 C/count
};

# define HEATER_0_TEMPTABLE temptable
# define HEATER_0_TEMPTABLE_LEN (sizeof(HEATER_0_TEMPTABLE)/sizeof(*HEATER_0_TEMPTABLE))

int target_temperature = 0;
int current_temperature_raw = 0;
float current_temperature = 0.0;
float Kp=DEFAULT_Kp;
float Ki=(DEFAULT_Ki*PID_dT);
float Kd=(DEFAULT_Kd/PID_dT);

static unsigned char soft_pwm;

//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

//static cannot be external:
static float temp_iState = 0;
static float temp_dState = 0;
static float pTerm;
static float iTerm;
static float dTerm;
static float pid_error;
static float temp_iState_min;
static float temp_iState_max;
static bool pid_reset;

//#define PGM_RD_W(x)   (short)pgm_read_word(&(x))

static float analog2temp(int raw) {
    float celsius = 0;
    uint8_t i;
    
    for (i=1; i<HEATER_0_TEMPTABLE_LEN; i++)
    {
      if ((HEATER_0_TEMPTABLE[i][0]) > raw)
      {
        celsius = (HEATER_0_TEMPTABLE[i-1][1]) +
          (raw - (HEATER_0_TEMPTABLE[i-1][0])) *
          (float)((HEATER_0_TEMPTABLE[i][1]) - (HEATER_0_TEMPTABLE[i-1][1])) /
          (float)((HEATER_0_TEMPTABLE[i][0]) - (HEATER_0_TEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == HEATER_0_TEMPTABLE_LEN) 
      celsius = (HEATER_0_TEMPTABLE[i-1][1]);

    return celsius;
}


void tp_init()
{
#if (MOTHERBOARD == 80) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1))
  //disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
  MCUCR=(1<<JTD);
  MCUCR=(1<<JTD);
#endif

  #if defined(HEATER_0_PIN) && (HEATER_0_PIN > -1)
    SET_OUTPUT(HEATER_0_PIN);
  #endif

  // Set analog inputs
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN;
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8);
    #endif
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);

  // Wait for temperature measurement to settle
  delay(250);
}


// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned char temp_state = 8;
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_0;

  if(pwm_count == 0){
    soft_pwm_0 = soft_pwm;
    if(soft_pwm_0 > 0) {
      WRITE(HEATER_0_PIN,1);
    } else WRITE(HEATER_0_PIN,0);
  }
  if(soft_pwm_0 < pwm_count) {
      WRITE(HEATER_0_PIN,0);
  }

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;
  
  switch(temp_state) {
    case 0: // Prepare TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      //lcd_buttons_update();
      temp_state = 1;
      break;
    case 1: // Measure TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      temp_state = 2;
      break;
    case 2: // Prepare TEMP_BED
      //lcd_buttons_update();
      temp_state = 3;
      break;
    case 3: // Measure TEMP_BED
      temp_state = 4;
      break;
    case 4: // Prepare TEMP_1
      //lcd_buttons_update();
      temp_state = 5;
      break;
    case 5: // Measure TEMP_1
      temp_state = 6;
      break;
    case 6: // Prepare TEMP_2
      //lcd_buttons_update();
      temp_state = 7;
      break;
    case 7: // Measure TEMP_2
      temp_state = 0;
      temp_count++;
      break;
    case 8: //Startup, delay initial temp reading a tiny bit so the hardware can settle.
      temp_state = 0;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  }

  if(temp_count >= OVERSAMPLENR) // 8 * 16 * 1/(16000000/64/256)  = 131ms.
  {
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
      current_temperature_raw = raw_temp_0_value;
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
  }
}


/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
    current_temperature = analog2temp(current_temperature_raw);
    //Reset the watchdog after we know we have a temperature measurement.
    //watchdog_reset();

    //CRITICAL_SECTION_START;
    temp_meas_ready = false;
    //CRITICAL_SECTION_END;
}


void PID_autotune(float temp, int ncycles)
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  DEBUGLN("PID Autotune start");

  disable_heater(); // switch off all heaters.

  soft_pwm = (PID_MAX)/2;
  bias = d = (PID_MAX)/2;
  
  for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = current_temperature;

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) {
          heating=false;
          soft_pwm = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(PID_MAX)-20);
            if(bias > (PID_MAX)/2) d = (PID_MAX) - 1 - bias;
            else d = bias;

            DEBUG(" bias: "); DEBUGLN(bias);
            DEBUG(" d: "); DEBUGLN(d);
            DEBUG(" min: "); DEBUGLN(min);
            DEBUG(" max: "); DEBUGLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              DEBUG(" Ku: "); DEBUGLN(Ku);
              DEBUG(" Tu: "); DEBUGLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              DEBUGLN(" Classic PID ");
              DEBUG(" Kp: "); DEBUGLN(Kp);
              DEBUG(" Ki: "); DEBUGLN(Ki);
              DEBUG(" Kd: "); DEBUGLN(Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;
              SERIAL_PROTOCOLLNPGM(" Some overshoot ");
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;
              SERIAL_PROTOCOLLNPGM(" No overshoot ");
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              */
            }
          }
          soft_pwm = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      }
    }
    if(input > (temp + 20)) {
      DEBUGLN("PID Autotune failed! Temperature too high");
      return;
    }
    if(millis() - temp_millis > 2000) {
      int p;
      p=soft_pwm;
      DEBUG("ok T:");

      DEBUGLN(input);
      DEBUG(" @:");
      DEBUGLN(p);

      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      DEBUGLN("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      DEBUGLN("PID Autotune finished! Put the last Kp, Ki and Kd constants from above into Configuration.h");
      return;
    }
    //lcd_update();
  }
}


void updatePID()
{
  temp_iState_max = PID_INTEGRAL_DRIVE_MAX / Ki;
}
#endif

/************** globals ************/

static short gasSensorValue = 0;
static short seatSensorValue = 0;

#if WIFI
Adafruit_CC3000_Client host;
#endif

/************** code ************/

void draw(void) 
{  
  u8g.drawStr( 0, 20, "Gas:");
  u8g.drawStr( 0, 40, String(gasSensorValue).c_str());
  u8g.drawStr( 100, 20, "Seat:");
  u8g.drawStr( 100, 40, String(seatSensorValue).c_str());
}

void showScreen(const char* text)
{
  u8g.firstPage();  
  do {
    u8g.drawStr(0, 20, text);
  } while (u8g.nextPage());
}

/*
void showScreen(const __FlashStringHelper* text)
{
  u8g.firstPage();  
  do {
    u8g.drawStr(0, 20, text);
  } while (u8g.nextPage());
}
*/

void setup(void) 
{
  pinMode(HEATER_0_PIN, OUTPUT);
  pinMode(SEAT_SENSOR, INPUT);
  
  disable_heater();
  
  Serial.begin(9600);

  // flip screen
  u8g.setRot180();
  // highest contrast  
  u8g.setContrast(0);
  
  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
#ifndef LEAN_AND_MEAN
  else if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
#endif
  
  //u8g.setFont(u8g_font_unifont);
  u8g.setFont(u8g_font_8x13);

  showScreen("Smart Toilet");

#if PID
  tp_init();

  PID_autotune(36.0, 5);
#endif

#if WIFI
  /* Initialise the module */
  if (!cc3000.begin())
  {
    DEBUGLN("cc3000.begin() failed");
    while(1);
  }
  
  showScreen(WLAN_SSID);

  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) 
  {
    DEBUGLN("Failed to connect");
    while(1);
  }

  showScreen("DHCP...");

  while (!cc3000.checkDHCP())
  {
    delay(200); // ToDo: Insert a DHCP timeout!
  }

  showScreen(WEBSITE);

  uint32_t destIP = 0;
  while (destIP == 0) {
    if (! cc3000.getHostByName(WEBSITE, &destIP)) {
      //showScreen( F("Couldn't resolve!"));
      DEBUGLN("Could not resolve");
    }
    delay(500);
  }
  
#if 1
  host = cc3000.connectUDP(destIP, 5417);
#else
  host = cc3000.connectTCP(destIP, 5417);
  if (host.connected())  
    DEBUGLN("Connected...");
  else
    DEBUGLN("NOT Connected...");
#endif

#endif
}


void loop() 
{
  // read the value from the sensors
  gasSensorValue = analogRead(GAS_SENSOR);
  seatSensorValue = digitalRead(SEAT_SENSOR);
  short rawtemp = analogRead(TEMP_0_PIN);
  
  static short rawavg;
  rawavg = (rawavg + rawtemp + 1) >> 1;
  DEBUGLN(rawavg);

  static boolean heating = false;
  if (heating)
  {
    if (rawavg > TEMP_TARGET) {
      WRITE(HEATER_0_PIN, 0);
      heating = false;
    }
  }
  else
  {
    if (rawavg < TEMP_TARGET) {
      WRITE(HEATER_0_PIN, 1);
      heating = true;
    }
  }

  String json = "{\"wc\":\"hackathon\",\"heat0\":"+String(rawavg)+",\"gas\":" + String(gasSensorValue) + ",\"seat\":" + String(seatSensorValue) + "}\r\n";  
  host.fastrprint(json.c_str());

  // picture loop
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );    
  
  // rebuild the picture after some delay
  delay(200);
}

