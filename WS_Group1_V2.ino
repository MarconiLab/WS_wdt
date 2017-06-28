//Weather station. TPH, rain, wind speed and direction.
//Anemometer, rain gauge and Wind Vane datasheet
//https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf
//Updating to ThingSpeak, Ubidots and Weather Underground
//using GPRSbee board. Use GPRSBee library v.1.3
//By RJC 

//libraries
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <math.h>

//SODAQ Mbili libraries
#include <Sodaq_BMP085.h>
#include <Sodaq_SHT2x.h>
#include <Sodaq_DS3231.h>
#include <Sodaq_PcInt.h>
#include <GPRSbee.h>
#include "Sodaq_wdt.h"

//Network constants GPRS
#define APN "www.xlgprs.net"
#define APN_USERNAME "xlgprs"
#define APN_PASSWORD "proxl"

//SpeakThings constants
#define URL "api.thingspeak.com/update"
#define URL "184.106.153.149/update"
#define WRITE_API_KEY "8H63LZ69PBNMO0HB"
//#define HOST_PORT   (80)

//update RTC
#define TIME_URL "149.210.205.161/time"
#define TIME_ZONE 7.0
#define TIME_ZONE_SEC (TIME_ZONE * 3600)

//Seperators
#define FIRST_SEP "?"
#define OTHER_SEP "&"
#define LABEL_DATA_SEP "="

//Data labels, cannot change for ThingSpeak
#define LABEL1 "field1"
#define LABEL2 "field2"
#define LABEL3 "field3"
#define LABEL4 "field4"
#define LABEL5 "field5"
#define LABEL6 "field6"
#define LABEL7 "field7"
#define LABEL8 "field8"

//The sleep length in seconds (MAX 86399)
#define SLEEP_PERIOD 60 //every minute to take readings
#define SEND_PERIOD 30   //how many minutes to post to TS
int post_seq,int_seq = 0; //sequence to post 

//RTC Interrupt RTC pin 
#define RTC_PIN A7
//RTC manually configuration, uncomment rtc.setDateTime(dt);
//year, month, date, hour, min, sec and week-day(starts from 0 and goes to 6)
//writing any non-existent time-data may interfere with normal operation of the RTC.
//Take care of week-day also.*/
DateTime dt(2016,8, 30, 12, 21, 00,2 );

//Sensors configuration
//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLT_PIN A6
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

//Use analog pin A0 for the light sensor
#define LIGHT_PIN A4 
#define WD_PIN  A2  //Wind direction
#define RAIN_PIN 4  //Rain tipping soft int 5!
#define WS_PIN 10   //Wind speed on pin 10 as (physical interruption 2)

//wind and rain sensors interruption
volatile int rpmcount = 0;//see http://arduino.cc/en/Reference/Volatile
int rpm = 0;

//Anemometer
#define WIND_SPEED 2.4/60 //from datasheet 2.4kmh=1.492MPH
#define KMH2MPH 1.60934

//Rain tipping funnel size
#define SIZE_FUNNEL 0.2794 //from datasheet 0.011''=0.2794mm

//Rain in one hour
#define NUM_BINS 60 //if updated every minute =60
volatile int cnt[NUM_BINS] = {0}; //vector with NUM_BINS positions
int idx = 0;

//Rain in 24h
#define NUM_BINS1 24 //counts 24 hours = 24
volatile int cnt1[NUM_BINS1] = {0}; //vector with NUM_BINS positions
int idx1 = 0;

//Wind in 5 minutes and gust
#define NUM_BINS2 5 // minutes
volatile int cnt2[NUM_BINS2] = {0}; //vector with NUM_BINS positions
volatile int windgustdirection_5m[NUM_BINS2] = {0}; //vector with NUM_BINS positions
int idx2 = 0;
int windgust_5m = 0; // [past 5 minutes wind gust]
int windgustdir_5m; // [0-360 past 5 minutes wind gust direction]

//variables used for interruptions
volatile boolean rtc_flag = true;
volatile uint8_t * pcmsk2 = digitalPinToPCMSK(RAIN_PIN);

//Data header
#define DATA_HEADER "Date Time, TempSHT21, PressureBMP, HumiditySHT21, Light, DP, BatVoltage, Wind direction, Wind speed, Wind speed avg, Rain mm, Rain 1h mm, Rain 1Day mm"

//TPH BMP sensor
Sodaq_BMP085 bmp;

//variable for sync
boolean syncc=true;

void setup() 
{
  //Initialise the serial connection
  Serial.begin(9600);
  Serial.println("Starting Weather Station by RJC");
    
  //Initialise sensors
  setupSensors();
    
  //Setup GPRSbee
  setupComms();
  
  //Setup sleep mode
  setupSleep();
  
  //Echo the data header to the serial connection
  Serial.println(DATA_HEADER);
  delay(1000);

  // Enable WDT
  sodaq_wdt_enable(WDT_PERIOD_8X);

  //Sync time if voltage is >3500mV
  int Batt = getRealBatteryVoltage() * 1000.0;
  if (Batt>3300) syncRTCwithServer();
    
  //Uncomment to take readings at the begining
  //post_seq = SEND_PERIOD-1;
}

void loop() 
{
    // If the WDT interrupt has been triggered
  if (sodaq_wdt_flag) {
    sodaq_wdt_flag = false;
    sodaq_wdt_reset();
    
    Serial.print("WDT interrupt has been triggered");
    
  } 
  if (rtc_flag) {
    sodaq_wdt_reset();
    rtc_flag = false;
     Serial.println("RTC interrupt has been triggered");
    //Schedule the next wake up pulse timeStamp + SLEEP_PERIOD
    RTCsleep();
    
    //Take readings
    takeReading(getNow());
    delay(100);
    rtc_flag = false;    
  }
  
  //Sleep
  systemSleep();  
}

void setupSensors()
{
  //Initialise the wire protocol for the TPH sensors
  Wire.begin();
  
  //Initialise the TPH BMP sensor
  bmp.begin();

  //Initialise the DS3231 RTC
  rtc.begin();
  //rtc.setDateTime(dt); //Adjust date-time as defined 'dt' above

  //Power sensor column GROVEPWR=22
  pinMode(GROVEPWR, OUTPUT);
  digitalWrite(GROVEPWR, LOW);
  
  //WindSpeed Analog Input
  //pinMode(WD_PIN, INPUT);
  
  //Interruption wind and rain
  pinMode(WS_PIN, INPUT_PULLUP);
  attachInterrupt(2,rpm_int,RISING);
  
  pinMode(RAIN_PIN, INPUT_PULLUP);
  PcInt::attachInterrupt(RAIN_PIN,rain_int);
  //attachInterruptMbili();
}

void setupComms()
{
  //Start Serial1 the Bee port
  Serial1.begin(9600);
  
 // This is the code to initialize the GPRSbee on a Autonomo with las Version
  gprsbee.initAutonomoSIM800(Serial1, BEEDTR, -1, BEECTS, 400);

  //uncomment this line to debug the GPRSbee with the serial monitor
  gprsbee.setDiag(Serial);
  
  //This is required for the Switched Power method
  //gprsbee.setPowerSwitchedOnOff(true); 
}

 void setupSleep()
{
  pinMode(RTC_PIN, INPUT_PULLUP);
  PcInt::attachInterrupt(RTC_PIN, wakeISR);
  
  //Set the sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void detachInterruptMbili()
{
  *pcmsk2 &= ~_BV(digitalPinToPCMSKbit(RAIN_PIN));
}

void attachInterruptMbili()
{
  *pcmsk2 |= _BV(digitalPinToPCMSKbit(RAIN_PIN));
}

void RTCsleep()
{
  //Schedule the next wake up pulse timeStamp + SLEEP_PERIOD
  DateTime wakeTime(getNow() + SLEEP_PERIOD);
  //rtc.enableInterrupts(wakeTime.hour(), wakeTime.minute(), 0);//wakeTime.second());

  if (syncc==true){
    // synchro to send at 0 or 30 min
    DateTime nowsync(getNow());
    if(nowsync.minute()<30){post_seq=nowsync.minute();}else{post_seq=(30-(60-nowsync.minute()));}
    Serial.print("Now minutes=");Serial.print(nowsync.minute());
    Serial.print(", Next sync=");
    Serial.println(30-post_seq);
    syncc=false;
  }
  rtc.enableInterrupts(wakeTime.hour(), wakeTime.minute(), 0);//wakeTime.second());
    
  //The next timed interrupt will not be sent until this is cleared
  rtc.clearINTStatus();
}

void systemSleep()
{

  // Wait till the output has been transmitted
  Serial.flush();
  Serial1.flush();

  //ADCSRA &= ~_BV(ADEN);         // ADC disabled
  disableper();
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  /*
   * This code is from the documentation in avr/sleep.h
   */
  cli();
  
  // Only go to sleep if there was no watchdog interrupt.
  if (!sodaq_wdt_flag)
  {
    // Power on LED before sleep
    //digitalWrite(SLEEP_LED, HIGH);
    
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
  sei();

  //ADCSRA |= _BV(ADEN);          // ADC enabled
  enableper();
}

void disableper(){
  ADCSRA &= ~(1<<ADEN);            //Disable ADC
  DIDR0 = 0x3F;                    //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D);   //Disable digital input buffer on AIN1/0
  ACSR = (1<<ACD);                 //Disable the analog comparator
  power_twi_disable();
  power_spi_disable();
  power_usart0_disable();
  power_timer0_disable();     //Needed for delay_ms
  power_timer1_disable();
  }
  
void enableper(){
  ADCSRA |= _BV(ADEN);
  DIDR0 = 0x00; 
  DIDR1 = 0x00;
  ACSR = 0x00;                 
  power_twi_enable();
  power_spi_enable();
  power_usart0_enable();
  power_timer0_enable();     //Needed for delay_ms
  power_timer1_enable();
  }


/* From the datasheet we adjusted the values for a 3V divider, using a 10K
 * resistor, the check values are the following:
 * --------------------+------------------+-------------------------------
 * Direction (Degrees)  Resistance (Ohms)  Voltage (mV)       ADC
 *     0                   33k                 2532.55        785 
 *     22.5                6.57k               1308.44        406
 *     45                  8.2k                1486.81        461
 *     67.5                891                 269.97         84
 *     90                  1k                  300.00         93
 *     112.5               688                 212.42         66
 *     135                 2.2k                595.08         184
 *     157.5               1.41k               407.80         126
 *     180                 3.9k                925.89         287
 *     202.5               3.14k               788.58         244
 *     225                 16k                 2030.76        630
 *     247.5               14.12k              1930.84        599
 *     270                 120k                3046.15        944
 *     292.5               42.12k              2666.84        827
 *     315                 64.9k               2859.41        886
 *     337.5               21.88k              2264.86        702
 * --------------------+------------------+-------------------------------
 */
int get_wind_direction(int DIR_PIN)
// read the wind direction sensor, return heading in degrees
{
  int adc = averageAnalogRead(DIR_PIN); // get the current reading from the sensor
   //Serial.println(adc);
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order!
  
  if (adc < 66) return (360-113);
  if (adc < 84) return (360-68);
  if (adc < 93) return (360-90);
  if (adc < 126) return (360-158);
  if (adc < 184) return (360-135);
  if (adc < 244) return (360-203);
  if (adc < 287) return (360-180);
  if (adc < 406) return (360-23);
  if (adc < 461) return (360-45);
  if (adc < 599) return (360-248);
  if (adc < 630) return (360-225);
  if (adc < 702) return (360-338);
  if (adc < 785) return (360-0);
  if (adc < 827) return (360-293);
  if (adc < 886) return (360-315);
  if (adc < 944) return (360-270);
  return (-1); // error, disconnected?
}

void takeReading(uint32_t ts)
{
  //Read sensor values
  //Rain tipping bucket
  int rain = cnt[idx];
  float rainmm = rain/2*SIZE_FUNNEL; //divided by 2 because of soft change int
  float rainin = rainmm/25.4;

  //rain in one hour is the sum of 60 minutes  
  unsigned long rainh = RainHour();
  float rainhmm = rainh/2*SIZE_FUNNEL;
  float rainhin=rainhmm/25.4;

  //rain in one day is the sum of 24 hours
  unsigned long raind = RainDay(rainh);
  float raindmm = raind*SIZE_FUNNEL;
  float raindin=raindmm/25.4;

    //wind average and guts in 5 minutes 
  float wind_5m = wind5m();
  float wind_5m_kmph = wind_5m *WIND_SPEED;
  float mph_5m = wind_5m *KMH2MPH;
  float windgustmph_5m = windgust_5m *KMH2MPH;

  post_seq++;
  Serial.println(post_seq);
  if (post_seq == SEND_PERIOD){
    post_seq = 0;
    float Temp = SHT2x.GetTemperature();
    float TempF = Temp*9/5+32;
    float Pres = bmp.readPressure() / 100;
    float Presin = Pres/33.8638866667;
    float Hum = SHT2x.GetHumidity();
  
   
    //Calculate Dew Point
    double gamma = log(Hum / 100) + ((17.625 * Temp) / (243.04 + Temp));
    double dp = 243.04 * gamma / (17.625 - gamma);
    double dpf = dp *9/5+32;
    
    //Read the voltage
    int mv = getRealBatteryVoltage() * 1000.0;
    
    //Instant RPM
    rpm = cnt2[idx2];  /* Con  , note: this works for one interruption per full rotation. For two interrups per full rotation use rpmcount * 30.*/
    float kmh = rpm *WIND_SPEED;
    float mph = kmh *KMH2MPH;
    String rpmS = String(rpm);
    
    digitalWrite(GROVEPWR, HIGH);//D22 power on Column
    delay(100);
    //wind direction
    int dir = averageAnalogRead(WD_PIN);
    int degree = get_wind_direction(dir);
    
    //light sensor
    int Light = averageAnalogRead(LIGHT_PIN);
    //Calculate the resistance from the sensor, from datasheet R->Lux
    float rSensor=(float)(1023-Light)*10 / Light; 
    float Lux = 350 * pow(rSensor, -1.43); //L = 350 * R^-1.43  where R is in kΩ.
    //https://forum.arduino.cc/index.php?topic=331679.0
    //float Lux =10000.0 / pow((rSensor*10.0),(4.0/3.0)); //Approximation from datasheet http://www.edaboard.com/thread278855.html
    
    digitalWrite(GROVEPWR, LOW);//D22 power off Column
  
    //Print to serial all the sensors data
    Serial.print(getDateTime());Serial.print(",");
    Serial.print(Temp);Serial.print(",");
    Serial.print(Pres);Serial.print(",");
    Serial.print(Hum);Serial.print(",");
    Serial.print(Light);Serial.print(",");
    Serial.print(dp);Serial.print(",");
    Serial.print(mv);Serial.print(",");
    Serial.print(degree);Serial.print(",");
    Serial.print(kmh);Serial.print(",");
    Serial.print(wind_5m_kmph);Serial.print(",");
    Serial.print(rainmm);Serial.print(",");
    Serial.print(rainhmm);Serial.print(",");
    Serial.println(raindmm);
  
    Serial.println("Sending to ThingSpeak");
    //Get the data record as a URL
    String url = createDataURL(String(Temp),String(Pres), String(Hum), String(Light), String(mv), String(degree), String(wind_5m_kmph), String(rainhmm));
    
    //Send it over the GPRS connection if voltage is >3500mV
    if (mv>3300) sendURLData(url);
    
    rpmcount = 0; // Restart counters
    Serial.print("Sleeping in low-power mode for ");
    Serial.print(SLEEP_PERIOD/60.0*SEND_PERIOD);
    Serial.println(" minutes");
  }
  
}

String getDateTime()
{
  String dateTimeStr;
  
  //Create a DateTime object from the current time
  DateTime dt(rtc.makeDateTime(rtc.now().getEpoch()));

  //Convert it to a String
  dt.addToString(dateTimeStr);
  
  return dateTimeStr;  
}

uint32_t getNow()
{
  return rtc.now().getEpoch();
}

float getRealBatteryVoltage()
{
  uint16_t batteryVoltage = analogRead(BATVOLT_PIN);
  return (ADC_AREF / 1023.0) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage;
}

//calculate wind average in 10 minutes and gusts
float wind5m(){
  int pos2,j2;
  float Wind_5m;
  // sum up the bins
  Wind_5m = 0;
  windgust_5m = 0;
  for (pos2=0; pos2<NUM_BINS2; pos2++)
  {
    Wind_5m += cnt2[pos2]; 
    if(cnt2[pos2] > windgust_5m)
    {
      windgust_5m = cnt2[pos2];
      windgustdir_5m = windgustdirection_5m[pos2];
    }
  }
    
   Wind_5m=Wind_5m/NUM_BINS2; 
 
   /* Serial.print("idx2=");Serial.println(idx2);
    Serial.print("Wind5m="); //uncomment to see the vector
    for (j2=0; j2<NUM_BINS2; j2++)
    {
      Serial.print( cnt2[j2] );Serial.print(",");
    }
    Serial.print("windavg5m=");Serial.print(Wind_5m);
    Serial.print(",windgust_5m=");Serial.println(windgust_5m);*/
    
  idx2 = (idx2 + 1) % NUM_BINS2; //idx from 0 to 9
  cnt2[idx2] = 0;
 
   
  return Wind_5m;
  }


//calculate rain in the last hour
unsigned long RainHour(){
  int pos,j;
  unsigned long Rainh;
  // sum up the bins
    Rainh = 0;
    for (pos=0; pos<NUM_BINS; pos++)
    {
      Rainh += cnt[pos]; 
    }
    idx = (idx + 1) % NUM_BINS; //idx from 0 to 9
    //Serial.print("idx=");Serial.println(idx);
    cnt[idx] = 0;
    /*Serial.print("Rainhour="); //uncomment to see the vector
    for (j=0; j<NUM_BINS; j++)
    {
      Serial.print( cnt[j] );Serial.print(",");
    }
    Serial.println();*/
    return Rainh;
}

//calculate rain in the last 24 hours
unsigned long RainDay(unsigned long rainhour){
  int pos1,j1;
  unsigned long Raind;
  cnt1[idx1]=rainhour;
  // sum up the bins
  Raind = 0;
  for (pos1=0; pos1<NUM_BINS1; pos1++)
  {
    Raind += cnt1[pos1]; 
  }
  if (idx==NUM_BINS-1){
    idx1 = (idx1 + 1) % NUM_BINS1;
    cnt[idx1] = 0;
  }
  /*Serial.print("idx1=");Serial.println(idx1);//uncomment to see the vector
  Serial.print("Rainday=");
  for (j1=0; j1<NUM_BINS1; j1++)
  {
    Serial.print( cnt1[j1] );
    Serial.print(",");
  }
  Serial.println();*/
  return Raind;
}

//Takes an average of readings on a given pin. Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
  //Serial.println(runningValue);
  return runningValue;
}

// Interruptions RTC, WindSpeed and Tipping Bucket
void wakeISR()
{
  rtc_flag = true;
  //Serial.println("it");
}

void rpm_int(){ /* this code will be executed every time the interrupt 0 (pin2) gets low.*/
  //rpmcount++;
  cnt2[idx2]++; //to count total in hours
  //Serial.println("i1");
  //rtc_flag = false;
}

void rain_int(){ /* this code will be executed every time the interrupt TIPPIN changes.*/
  //rpmtipcount++;
  cnt[idx]++; //to count total in hours
  //Serial.println("i2");
  //rtc_flag =false;
}

String createDataURL(String T, String P, String H, String L, String V, String WD, String WS, String R)
{
    //Construct data URL
    String url = URL;
  
    //Add key followed by each field
    url += String(FIRST_SEP) + String("key");
    url += String(LABEL_DATA_SEP) + String(WRITE_API_KEY);
 
    url += String(OTHER_SEP) + String(LABEL1);
    url += String(LABEL_DATA_SEP) + String(T);
 
    url += String(OTHER_SEP) + String(LABEL2);
    url += String(LABEL_DATA_SEP) + String(P);
 
    url += String(OTHER_SEP) + String(LABEL3);
    url += String(LABEL_DATA_SEP) + String(H);
 
    url += String(OTHER_SEP) + String(LABEL4);
    url += String(LABEL_DATA_SEP) + String(L);

    url += String(OTHER_SEP) + String(LABEL5);
    url += String(LABEL_DATA_SEP) + String(V);

    url += String(OTHER_SEP) + String(LABEL6);
    url += String(LABEL_DATA_SEP) + String(WD);
    
    url += String(OTHER_SEP) + String(LABEL7);
    url += String(LABEL_DATA_SEP) + String(WS);
    
    url += String(OTHER_SEP) + String(LABEL8);
    url += String(LABEL_DATA_SEP) + String(R);
   return url;  
}

void sendURLData(String url)
{
    char result[20] = "";
    gprsbee.doHTTPGET(APN, APN_USERNAME, APN_PASSWORD, url.c_str(), result, sizeof(result));
  
    Serial.println("Received: " + String(result));
}

void syncRTCwithServer()
{
    char buffer[20];
   
    if (gprsbee.doHTTPGET(APN, APN_USERNAME, APN_PASSWORD, TIME_URL, buffer, sizeof(buffer)))
    {
        Serial.println("HTTP GET: " + String(buffer)); 
         
        //Convert the time stamp to unsigned long
        char *ptr;
        uint32_t newTs = strtoul(buffer, &ptr, 0);
  
        // Add the timezone difference plus a few seconds 
        // to compensate for transmission and processing delay
        newTs += TIME_ZONE_SEC;
 
        // If conversion was successful
        if (ptr != buffer) 
        {
            //Get the old time stamp
            uint32_t oldTs = rtc.now().getEpoch();
            int32_t diffTs = abs(newTs - oldTs);

            //If time is more than 30s off, update
            if (diffTs > 30) 
            {
                //Display old and new time stamps
                Serial.print("Updating RTC, old=" + String(oldTs));
                Serial.println(" new=" + String(newTs));

                //Update the rtc
                rtc.setEpoch(newTs);
            }
        }
    }
}
