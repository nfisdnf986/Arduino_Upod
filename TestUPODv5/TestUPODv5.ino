#include <SPI.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <RTC_DS3231.h>
#include <SoftwareSerial.h>
#include <mcp3424.h>
#include <Adafruit_GPS.h>
#include <FileIO.h> //Yun SD

//IMPORTANT: MODEL MUST HAVE A 2-digit ID number
#define model "UPODXY" //UPOD model indicator
#define first_date 12 //this the possition of the first date character in 'current_file'
                     //s.t. the date is represented as ...MMDD...
                     //eg. "HELLO*_MMDDYYYY" would require first_date=7
char current_file[]="/mnt/sda1/SSMMDDYY.txt";  //make sure to check first_date value 

SoftwareSerial ss(8,9);

Adafruit_GPS GPS(&ss);
RTC_DS3231 RTC;
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2(B1001001);
//Quadstat ADC instances and variables
mcp3424 alpha_one;
mcp3424 alpha_two;
float alpha_value[8];
//BMP Temp and Pressure Variables
SFE_BMP180 BMP;
double T, P, p0, a;
char status;
//SHT2 Temp and Humidity Variables
#define SHT2x_address 64
const byte mask = B11111100;
const byte temp_command = B11100011;
const byte hum_command = B11100101;
byte TEMP_byte1, TEMP_byte2, TEMP_byte3;
byte HUM_byte1, HUM_byte2, HUM_byte3;
byte check1, check2;
unsigned int temperature_board, humidity_board;
float temperature_SHT, humidity_SHT;
//Weather Station Variables
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
byte windspdavg[120]; //120 bytes to keep track of 2 minute average
const byte WDIR = A0; //Wind direction
#define WIND_DIR_AVG_SIZE 120
int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
//GPS variables
boolean usingInterrupt = false;
void useInterrupt(boolean);
uint32_t timer = millis();
//holders for the ads ADCs
int ADC1[4];
int ADC2[4];
//Interrupt routine (called by the hardware interrupts, not by the main code)
void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
	if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
	{
		lastWindIRQ = millis(); //Grab the current time
		windClicks++; //There is 1.492MPH for each click per second.
	}
}
void setup() {
  Serial.begin(9600);
  GPS.begin(4800);
  ads1.begin();
  ads2.begin();
  BMP.begin();
  RTC.begin();
  Bridge.begin();  //Yun SD
  FileSystem.begin(); //Yun SD
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  //Need to incorporate debounce for anemometer reed switch.
  //Buffer time of a few hundred milliseconds is all that's needed
  attachInterrupt(4, wspeedIRQ, FALLING); //anemometer reed switch on pin 7--> interrupt# 4
  
  DateTime now = RTC.now(); //Get time from RTC
  current_file[first_date]=now.month()/10+'0';
  current_file[first_date+1]=now.month()%10+'0';
  current_file[first_date+2]=now.day()/10+'0';
  current_file[first_date+3]=now.day()%10+'0';
  current_file[first_date+4]=((now.year()/10)%100)%10+'0';
  current_file[first_date+5]=((now.year()%1000)%100)%10+'0';
  //fill 'current_file' with SS of UPOD<SS> model number
  current_file[10]=model[4]; //get the UPOD model number from defined string at header
  current_file[11]=model[5];

  File myFile = FileSystem.open(current_file, FILE_APPEND);
  if(!myFile){
    while(!myFile){ //Is there a better way to do this? Test/modify later
      File myFile = FileSystem.open(current_file, FILE_APPEND);
      digitalWrite(11,HIGH);
      delay(500);
      digitalWrite(11,LOW);
      delay(500);
  }
 }
  if (myFile){
      myFile.println("RTC Date/Time, SHT Temp, SHT Humidity, BMP Temp, BMP Pressure, Wind Count, Wind Direction, Fig210 Heater ADC val(0-65535), Fig210 Sensor ADC val(0-65535), Fig280 Heater ADC val(0-65535), Fig280 Sensor ADC val(0-65535), BL Mocoon sensor ADC(0-65535), ADC2 Channel 2 - EMPTY, e2vO3 Heater ADC value (0-65535), e2vO3 Sensor ADC value (0-65535), CO2 ADC value (0-4095), Quad_Aux1, Quad_Main1, Quad_Aux2, Quad_Main2, Quad_Aux3, Quad_Main3, Quad_Aux4, Quad_Main4");
      myFile.close();
  }

  alpha_one.GetAddress('G','F'); //user defined address for the alphasense pstat array (4-stat)
  alpha_two.GetAddress('H','H') ;

  ss.println("$PTNLSNM,0101,02"); //Address for RMC and GGA outputs. $PTNLSNM,0021,02 for GGA and ZTG
  delay(500);
  ss.println("$PTNLSNM,0101,02"); 
  useInterrupt(true);
}
void loop() {
  DateTime now = RTC.now(); //Get time from RTC
  //This is an inefficient way of modifying the name of the .txt file everyday.
  //It's ok in setup but not in the main loop.
  //Should use an RTC daily interrupt. Will modify later. -Drew
  current_file[first_date]=now.month()/10+'0';
  current_file[first_date+1]=now.month()%10+'0';
  current_file[first_date+2]=now.day()/10+'0';
  current_file[first_date+3]=now.day()%10+'0';
  current_file[first_date+4]=((now.year()/10)%100)%10+'0';
  current_file[first_date+5]=((now.year()%1000)%100)%10+'0';
  //Get NMEA sentence from GPS
  if (! usingInterrupt) { 
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
  }
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  //Get Wind Direction
  int  wind_dir = get_wind_direction();
  //Get Wind Speed...
  //Get SHT data
  get_SHT2x();
  //Get BMP data
  status = BMP.startTemperature();
  if (status != 0)
  {
    Serial.println(status);
    delay(status);
    status = BMP.getTemperature(T);
    status = BMP.startPressure(3);
    if (status != 0)
    {
      delay(status);
      status = BMP.getPressure(P, T);
    }
    else //if good temp; but can't compute P
    {
      P = -99;
    }
  }
  else //if bad temp; then can't compute temp or pressure
  {
    T = -99;
    P = -99;
  }
  //GET CO2 data
  float co2 = getS300CO2();
  //Get Quadstat data
  alpha_value[0]=alpha_one.GetValue(1);
  alpha_value[1]=alpha_one.GetValue(2);
  alpha_value[2]=alpha_one.GetValue(3);
  alpha_value[3]=alpha_one.GetValue(4);
  alpha_value[4]=alpha_two.GetValue(1);
  alpha_value[5]=alpha_two.GetValue(2);
  alpha_value[6]=alpha_two.GetValue(3);
  alpha_value[7]=alpha_two.GetValue(4);

  // approximately every 2 seconds or so, print out the current stats
  // Should put the following IF statement before the myFile.print(GPS data). Else leave GPS blank.
  //Maybe there's a better way to do this... Not quite sure yet how to best manage SD writes of
  //asyncronous events like GPS and windspeed.
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    File myFile = FileSystem.open(current_file, FILE_APPEND);
    if (myFile){
      myFile.print(GPS.day, DEC);
      myFile.print('/');
      myFile.print(GPS.month, DEC);
      myFile.print("/20");
      myFile.print(GPS.year, DEC);
      myFile.print(GPS.hour, DEC);
      myFile.print(':');
      myFile.print(GPS.minute, DEC);
      myFile.print(':');
      myFile.print(GPS.seconds, DEC);
      myFile.print('.');
      myFile.print(GPS.milliseconds);
      myFile.print((int)GPS.fix);
      myFile.print((int)GPS.fixquality); 
      myFile.print(GPS.latitudeDegrees, 4);
      myFile.print(", "); 
      myFile.print(GPS.longitudeDegrees, 4);
      myFile.print(GPS.altitude);
      myFile.print((int)GPS.satellites);
      myFile.print(now.year(), DEC);
      myFile.print('/');
      myFile.print(now.month(), DEC);
      myFile.print('/');
      myFile.print(now.day(), DEC);
      myFile.print(' ');
      myFile.print(now.hour(), DEC);
      myFile.print(':');
      myFile.print(now.minute(), DEC);
      myFile.print(':');
      myFile.print(now.second(), DEC);
      myFile.print(", ");
      myFile.print(temperature_SHT);
      myFile.print(", ");
      myFile.print(humidity_SHT);
      myFile.print(", ");
      myFile.print(T);
      myFile.print(", ");
      myFile.print(P);
      myFile.print(", ");
      //myFile.print(wind_count);
      myFile.print(", ");
      myFile.print(wind_dir);
      for (int i = 0; i < 4; i++){
        ADC1[i] = ads1.readADC_SingleEnded(i);
        //0.1875 mV per bit. The default gain on ADC is +/-6.144 volts.
        //The 16 bit ADC (1 drops out, acutally 15) corresponds to 32,786 possible output values(2^15).
        //V1[i] = ADC1[i] * 0.1875;
        myFile.print(ADC1[i]);
        myFile.print(", ");
    }
      for (int i = 0; i < 4; i++)
      {
        ADC2[i] = ads2.readADC_SingleEnded(i);
//        V2[i] = ADC2[i] * 0.1875;
        myFile.print(ADC2[i]);
        myFile.print(", ");
      }
      for(int i=0;i<8;i++){
        myFile.print(alpha_value[i]);
        myFile.print(", ");
        Serial.print(alpha_value[i]);
        Serial.print(", ");
      }
      myFile.println(co2);
      myFile.close();
    }

     else if(!myFile){
       //Include LED status indicator here that loops until myFile true
       Serial.println("Error opening datalog.txt"); 
     }
  }
  //Indicator LEDs shows that code is going thru Main loop
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);
  delay(2000);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
}
void get_SHT2x()
{
  Wire.beginTransmission(SHT2x_address);
  Wire.write(temp_command);
  check1 = Wire.endTransmission();

  Wire.requestFrom(SHT2x_address, 3);

  TEMP_byte1 = Wire.read();
  TEMP_byte2 = Wire.read();
  TEMP_byte3 = Wire.read();

  Wire.beginTransmission(SHT2x_address);
  Wire.write(hum_command);
  check2 = Wire.endTransmission();

  Wire.requestFrom(SHT2x_address, 3);
  HUM_byte1 = Wire.read();
  HUM_byte2 = Wire.read();
  HUM_byte3 = Wire.read();

  humidity_board = ( (HUM_byte1 << 8) | (HUM_byte2) & mask ); //HUM_byte1 shifted left by 1 byte, (|) bitwise inclusize OR operator
  temperature_board = ( (TEMP_byte1 << 8) | (TEMP_byte2) & mask );

  humidity_SHT = ((125 * (float)humidity_board) / (65536)) - 6.00;
  temperature_SHT = ((175.72 * (float)temperature_board) / (65536)) - 46.85;
}
float getS300CO2()
{
  int i = 1;
  long reading;
  float CO2val;
  Wire.beginTransmission(0x31);
  Wire.write(0x52);
  Wire.endTransmission();
  Wire.requestFrom(0x31, 7);
  while (Wire.available())
  {
    byte val = Wire.read();
    if (i == 2)
    {
      reading = val;
      reading = reading << 8;
    }
    if (i == 3)
    {
      reading = reading | val;
    }
    i = i + 1;
  }
  CO2val = reading / 4095.0 * 5000.0;
  CO2val = reading;
  return CO2val;
}

//// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//Returns the instataneous wind speed 
float get_wind_speed()//Will be modified from Sparkfun's Example
{
	float deltaTime = millis() - lastWindCheck; //750ms

	deltaTime /= 1000.0; //Covert to seconds

	float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

	windClicks = 0; //Reset and start watching for new wind
	lastWindCheck = millis();

	windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

	/* Serial.println();
	 Serial.print("Windspeed:");
	 Serial.println(windSpeed);*/

	return(windSpeed);
}
int get_wind_direction() //Will be modified from Sparkfun's Example
{
	unsigned int adc;

	adc = analogRead(WDIR); // get the current reading from the sensor

	// The following table is ADC readings for the wind direction sensor output, sorted from low to high.
	// Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
	// Note that these are not in compass degree order! See Weather Meters datasheet for more information.

	if (adc < 380) return (113);
	if (adc < 393) return (68);
	if (adc < 414) return (90);
	if (adc < 456) return (158);
	if (adc < 508) return (135);
	if (adc < 551) return (203);
	if (adc < 615) return (180);
	if (adc < 680) return (23);
	if (adc < 746) return (45);
	if (adc < 801) return (248);
	if (adc < 833) return (225);
	if (adc < 878) return (338);
	if (adc < 913) return (0);
	if (adc < 940) return (293);
	if (adc < 967) return (315);
	if (adc < 990) return (270);
	return (-1); // error, disconnected?
}
void calcWeather(){
        //Calc winddir
	int winddir = get_wind_direction();

	//Calc windspeed
	float windspeedmph = get_wind_speed();
        //Calc windspdmph_avg2m
	float temp = 0;
	for(int i = 0 ; i < 120 ; i++)
		temp += windspdavg[i];
	temp /= 120.0;
	float windspdmph_avg2m = temp;
        //Calc winddir_avg2m, Wind Direction
	//You can't just take the average. Google "mean of circular quantities" for more info
	//We will use the Mitsuta method because it doesn't require trig functions
	//And because it sounds cool.
	//Based on: http://abelian.org/vlf/bearings.html
	//Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
	long sum = winddiravg[0];
	int D = winddiravg[0];
	for(int i = 1 ; i < WIND_DIR_AVG_SIZE ; i++)
	{
		int delta = winddiravg[i] - D;
		if(delta < -180)
			D += delta + 360;
		else if(delta > 180)
			D += delta - 360;
		else
			D += delta;
		sum += D;
	}
        int winddir_avg2m = sum / WIND_DIR_AVG_SIZE;
	if(winddir_avg2m >= 360) winddir_avg2m -= 360;
	if(winddir_avg2m < 0) winddir_avg2m += 360;
}
