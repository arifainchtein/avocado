//this is the firmware for Cleo
//
//
#include <RTCInfoRecord.h>
#include <WPSSensorRecord.h>
#include <SPI.h>
#include <SD.h>
#include <TimeManager.h>
#include <GeneralFunctions.h>
#include <SecretManager.h>
#include <SDCardManager.h>
#include <OneWire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <totp.h>

//
// Device Specific Librarieas
//
#include <rgb_lcd.h>
#include <ChainableLED.h>
#include "DHT.h"


//
// Platform Variables
//
/
// current view index
// Controls what the user sees in the lcdnow
// it starts with a value of 99 which means is locked
int currentViewIndex=0;

boolean showingAct=false;


String toReturn="";
//
// the virtual micrcntroller
boolean isHost=true;
String currentIpAddress="No IP";
String currentSSID="No SSID";
int delayTime=1;

boolean inPulse=false;



unsigned long time;
unsigned long lastClickMillis;
unsigned long millisToLock=300000;
long poweredDownInLoopSeconds;
unsigned long lastActionTime=0;

boolean inputingUnlockSequence=false;
unsigned char unlockSequence=0;
int unlockSequenceCurrentPosition=0;
String systemKey="MySecretKey";
String command="";

//
// the wps variables
long secondsToTurnPowerOff = 30;
long secondsToNextPiOn=90L;
long currentSecondsToPowerOff=0L;
boolean wpsCountdown=false;
boolean wpsSleeping=false;
boolean inWPS=false;
float minWPSVoltage=12.4;
float enterWPSVoltage=12.6;
float exitWPSVoltage=12.8;

long wpsCountDownStartSeconds=0L;
//
// the virtual micrcntroller
// the //lcd at @Sento:Internal:Actuators:LCD

boolean notInPulse=true;
//boolean showSensorData=true;
int secondsForCommandsToBeExecuted=4;
int currentCommandSecondsCounter=0;
volatile int f_wdt=1;
char *faultData;
long secondsToForcedWPS=60L;
long wpsAlertTime=0L;
int wpsPulseFrequencySeconds=60;
long lastWPSRecordSeconds=0L;
boolean waitingForWPSConfirmation=false;
boolean pauseDuringWPS=false;
long currentSleepStartTime=0L;
long lastWPSStartUp=0L;

float capacitorVoltage= 0;
#define LOCK_CAPACITOR_PIN A2

int SHARED_SECRET_LENGTH=27;

int currentHour=0;
int currentDay=0;
int currentMonth=0;
int currentYear=0;
float dailyMinBatteryVoltage=0;
float dailyMaxBatteryVoltage=0;

float dailyMinBatteryCurrent=0;
float dailyMaxBatteryCurrent=0;
float dailyBatteryOutEnergy=0;
float dailyPoweredDownInLoopSeconds=0;

float hourlyBatteryOutEnergy=0;
float hourlyPoweredDownInLoopSeconds=0;



//
const char *DAILY_STATS_TIMESTAMP="Daily Timestamp";
const char *DAILY_MINIMUM_BATTERY_VOLTAGE="Daily Minimum Battery Voltage";
const char *DAILY_MAXIMUM_BATTERY_VOLTAGE="Daily Maximum Battery Voltage";
const char *DAILY_MINIMUM_BATTERY_CURRENT="Daily Minimum Battery Current";
const char *DAILY_MAXIMUM_BATTERY_CURRENT="Daily Maximum Battery Current";
const char *DAILY_ENERGY="Daily Energy";
const char *DAILY_POWERED_DOWN_IN_LOOP_SECONDS="Hourly Powered Down In Loop Seconds";
const char *HOURLY_ENERGY="Hourly Energy";
const char *HOURLY_POWERED_DOWN_IN_LOOP_SECONDS="Hourly Powered Down In Loop Seconds";
const char *HOURLY_OPERATING_IN_LOOP_SECONDS="Hourly Operating In Loop Seconds";

const char *MAXIMUM_VALUE="Max";
const char *MINIMUM_VALUE="Min";
const char *AVERAGE_VALUE="Avg";

const char *UNIT_NO_UNIT =" ";
const char *UNIT_VOLT ="Volt";
const char *UNIT_SECONDS="seconds";
const char *UNIT_MILLI_AMPERES ="mA";
const char *UNIT_MILLI_AMPERES_HOURS ="mAh";
const char *UNIT_PERCENTAGE ="%";
const char *FORCED_PI_TURN_OFF ="Forced Pi Turn Off";
const char *BATTERY_VOLTAGE_BEFORE_PI_ON ="Battery Voltage Before Turning Pi On";
const char *BATTERY_VOLTAGE_ATER_PI_ON="Battery Voltage After Turning Pi On";
const char *BATTERY_VOLTAGE_DIFFERENTIAL_AFTER_PI_ON ="Battery Voltage Differential After Turning Pi On";
const char *PI_TURN_OFF ="Pi Turn Off";

const char *LIFE_CYCLE_EVENT_FORCED_START_WPS ="Forced Start WPS";
const char *LIFE_CYCLE_MANUAL_SHUTDOWN    ="Manual Shutdown";
const char *LIFE_CYCLE_EVENT_START_WPS    ="Start WPS";
const char *LIFE_CYCLE_EVENT_END_WPS     ="End WPS";
const char *LIFE_CYCLE_EVENT_START_COMMA ="Start Comma";
const char *LIFE_CYCLE_EVENT_END_COMMA ="End Comma";
const char *LIFE_CYCLE_EVENT_SETUP_COMPLETED="Setup Method Completed";

const int LIFE_CYCLE_EVENT_AWAKE_VALUE=3;
const int LIFE_CYCLE_EVENT_WPS_VALUE=2;
const int LIFE_CYCLE_EVENT_COMMA_VALUE=1;
String operatingStatus ="Normal";

const char  *WPSSensorDataDirName="WPSSensr";
const char  *LifeCycleDataDirName="LifeCycl";
const char  *RememberedValueDataDirName  = "RememVal";
const char  *unstraferedFileName ="Untransf.txt";
char remFileName[10];

char sensorDirName[10];
char lifeCycleFileName[10];
long shutDownRequestedseconds= 0L;
boolean shuttingDownPiCountdown=false;
boolean manualShutdown=false;
boolean waitingManualPiStart=false;
GeneralFunctions generalFunctions;
TimeManager timeManager(generalFunctions, Serial);
SecretManager secretManager(timeManager);
SDCardManager sdCardManager(timeManager, generalFunctions, Serial);

//
// End of Platform Variables
//


D//
// Device Specific Variables
//








//
// the ambient sensor
//
#define DHTPIN A0     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
float ambientTemperature = 0;
float ambientHumidity = 0;




//
// current sensor
#define CURRENT_SENSOR A3
float amplitude_current;               //amplitude current
float effective_value;       //effective current
boolean powerSupplyOn=false;


//
// the led
//
ChainableLED leds(2, 3, 4);
//
// the order for the leds is:
//
// 5 - Sento Bore Pump Status
// 4 - Ra Battery Voltage
// 3 - Sento
// 2 - Tlaloc
// 1 - Ra
// 0 - Identity
//
String ledStatusLine1[]={"","Ra","Sento","Tlaloc"};
String ledStatusLine2[]={"","","",""};
String pulseStartTime="";
String pulseStopTime="";
String horseNameSecretKey[]={"","","","","","","","","","","","","","","","","","","",""};
//
// the lcd at @Cleo:Internal:Actuators:LCD
//
rgb_lcd lcd;


long debounceDelay = 60;    // the debounce time; increase if the output flickers

//
// two sets of variables one each for the select button
// and one for the action button
//
int selectPin = 4;
//int selectButtonValue=0;
//int selectButtonState;             // the current reading from the input pin
//int selectLastButtonState = LOW;   // the previous reading from the input pin
//long selectLastDebounceTime = 0;  // the last time the output pin was toggled


int actionPin = 5;
//int actionButtonValue=0;
//int actionButtonState;             // the current reading from the input pin
//int actionLastButtonState = LOW;   // the previous reading from the input pin
//long actionLastDebounceTime = 0;  //


int PI_POWER_PIN=8;






void hourlyTasks(long time, int previousHour ){

	sdCardManager.storeRememberedValue(time,HOURLY_ENERGY, hourlyBatteryOutEnergy, UNIT_MILLI_AMPERES_HOURS);
	sdCardManager.storeRememberedValue(time,HOURLY_POWERED_DOWN_IN_LOOP_SECONDS, hourlyPoweredDownInLoopSeconds, UNIT_SECONDS);
	sdCardManager.storeRememberedValue(time,HOURLY_OPERATING_IN_LOOP_SECONDS, 3600-hourlyPoweredDownInLoopSeconds, UNIT_SECONDS);
	hourlyBatteryOutEnergy=0;
	hourlyPoweredDownInLoopSeconds=0;
}

/*
 * this function is called at the beginning of a new day
 */
void dailyTasks(long time, int yesterdayDate, int yesterdayMonth, uint16_t yesterdayYear ){
	//
	// move whatever is in untrasferred to the correct date
	boolean result = sdCardManager.readUntransferredFileFromSDCardByDate( 1,false, RememberedValueDataDirName,yesterdayDate, yesterdayMonth, yesterdayYear );
	result = sdCardManager.readUntransferredFileFromSDCardByDate( 1,false, WPSSensorDataDirName,yesterdayDate, yesterdayMonth, yesterdayYear);
	result = sdCardManager.readUntransferredFileFromSDCardByDate( 1,false, LifeCycleDataDirName,yesterdayDate, yesterdayMonth, yesterdayYear);
	long yesterdayDateSeconds = timeManager.dateAsSeconds(yesterdayYear,yesterdayMonth,yesterdayDate, 0, 0, 0);
	sdCardManager.storeRememberedValue(time,DAILY_STATS_TIMESTAMP, yesterdayDateSeconds, UNIT_NO_UNIT);
	sdCardManager.storeRememberedValue(time,DAILY_MINIMUM_BATTERY_VOLTAGE, dailyMinBatteryVoltage, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,DAILY_MAXIMUM_BATTERY_VOLTAGE, dailyMaxBatteryVoltage, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,DAILY_MINIMUM_BATTERY_CURRENT, dailyMinBatteryCurrent, UNIT_MILLI_AMPERES);
	sdCardManager.storeRememberedValue(time,DAILY_MAXIMUM_BATTERY_CURRENT, dailyMaxBatteryCurrent, UNIT_MILLI_AMPERES);
	sdCardManager.storeRememberedValue(time,DAILY_ENERGY, dailyBatteryOutEnergy, UNIT_MILLI_AMPERES_HOURS);
	sdCardManager.storeRememberedValue(time,DAILY_POWERED_DOWN_IN_LOOP_SECONDS, dailyPoweredDownInLoopSeconds, UNIT_SECONDS);
	dailyMinBatteryVoltage = 9999.0;
	dailyMaxBatteryVoltage = -1.0;
	dailyMinBatteryCurrent = 9999.0;
	dailyMaxBatteryCurrent = -1.0;
	dailyBatteryOutEnergy=0.0;
	dailyPoweredDownInLoopSeconds=0.0;

}

void monthlyTasks(long time){

}

void yearlyTasks(long time){

}



float calculateCurrent(){
	int sensorValue;             //value read from the sensor
	int sensorMax = 0;
	uint32_t start_time = millis();
	while((millis()-start_time) < 100)//sample for 1000ms
	{
		sensorValue = analogRead(CURRENT_SENSOR);
		if (sensorValue > sensorMax)
		{
			//record the maximum sensor value
			sensorMax = sensorValue;
		}
	}

	//the VCC on the Grove interface of the sensor is 5v
	amplitude_current=(float)(sensorMax-512)/1024*5/185*1000000;
	effective_value=amplitude_current/1.414;
	return effective_value;
}

float getBatteryVoltage(){
	long  sensorValue=analogRead(batteryVoltagePin);
	long  sum=0;
	for(int i=0;i<10;i++)
	{
		sum=sensorValue+sum;
		sensorValue=analogRead(batteryVoltagePin);
		delay(2);
	}
	sum=sum/10;
	float value =(10*sum*4.980/1023.00);
	return value;
}

void initializeWDT(){
	/*** Setup the WDT ***/

	/* Clear the reset flag. */
	MCUSR &= ~(1<<WDRF);

	/* In order to change WDE or the prescaler, we need to
	 * set WDCE (This will allow updates for 4 clock cycles).
	 */
	WDTCSR |= (1<<WDCE) | (1<<WDE);

	/* set new watchdog timeout prescaler value */
	WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */

	/* Enable the WD interrupt (note no reset). */
	WDTCSR |= _BV(WDIE);

}

/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
	//lcd.setCursor(0, 1);
	//lcd.print("Waking Up") ;
	//wdt_reset();


	if(f_wdt == 0)
	{
		f_wdt=1;
	}
	else
	{
		//Serial.println("WDT Overrun!!!");
	}
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterArduinoSleep(void)
{
	digitalWrite(PI_POWER_PIN, LOW);



	wdt_reset();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
	//sleep_enable();
	long currentSleepSeconds = timeManager.getCurrentTimeInSeconds();
	/* Now enter sleep mode. */
	sleep_mode();

	/* The program will continue from here after the WDT timeout*/

	//
	// check the voltage of the battery, if its higher than
	// the min for wps then go into wps,
	// otherwise go back to comma
	//
	long lastSleepSeconds = timeManager.getCurrentTimeInSeconds()-currentSleepSeconds ;
	poweredDownInLoopSeconds+=lastSleepSeconds;
	float batteryVoltage = getBatteryVoltage();
	if(batteryVoltage>minWPSVoltage){
		// STORE a lifecycle comma exit record
		long now = timeManager.getCurrentTimeInSeconds();
		//sdCardManager.storeLifeCycleEvent(now, LIFE_CYCLE_EVENT_END_COMMA, LIFE_CYCLE_EVENT_COMMA_VALUE);
		lcd.display();
		lcd.setRGB(255,255,0);
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Out of Comma");
		lcd.setCursor(0,1);
		lcd.print(batteryVoltage);
		lcd.print("V ");
		lcd.print(lastSleepSeconds);
		lcd.print("V ");

		operatingStatus="WPS";
		currentSleepStartTime = now;
		wpsSleeping=true;
		inWPS=true;
		sleep_disable(); /* First thing to do is disable sleep. */
		/* Re-enable the peripherals. */
		power_all_enable();
	}else{
		lcd.display();
		lcd.setRGB(255,0,0);
		lcd.clear();
		lcd.print(batteryVoltage);
		lcd.print("V");
		delay(500);
		lcd.noDisplay();
		lcd.setRGB(0,0,0);
		enterArduinoSleep();
	}

}

/***************************************************
 *  Name:        pauseWPS
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: This method is similar to enterSleep except that is called
 *               during the time where the voltage is whithin the wps range
 *               and is a way to save power
 *               it is different than the comma because it does not recursively
 *               call itself and does not write lifecycle events
 *
 ***************************************************/
void pauseWPS(void)
{
	digitalWrite(PI_POWER_PIN, LOW);
	lcd.noDisplay();
	lcd.setRGB(0,0,0);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
	sleep_enable();

	long currentSleepSeconds = timeManager.getCurrentTimeInSeconds();
	/* Now enter sleep mode. */
	sleep_mode();

	/* The program will continue from here after the WDT timeout*/

	//
	// check the voltage of the battery, if its higher than
	// the min for wps then go into wps,
	// otherwise go back to comma
	//
	long lastSleepSeconds = timeManager.getCurrentTimeInSeconds()-currentSleepSeconds ;
	poweredDownInLoopSeconds+=lastSleepSeconds;

	lcd.display();
	lcd.setRGB(255,255,0);
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Out of Pause");
	lcd.setCursor(0,1);
	float batteryVoltage = getBatteryVoltage();
	lcd.print(batteryVoltage);
	lcd.print("V ");
	lcd.print(pauseDuringWPS);

	operatingStatus="WPS";
	//lcd.setCursor(0, 1);
	//lcd.print("Awake") ;
	sleep_disable(); /* First thing to do is disable sleep. */
	/* Re-enable the peripherals. */
	power_all_enable();
}

void sendWPSAlert(long time, char *faultData, int batteryVoltage){
	waitingForWPSConfirmation=true;
	wpsCountdown=false;
	inWPS=true;
	operatingStatus="WPS";
	wpsAlertTime=timeManager.getCurrentTimeInSeconds();
	//sdCardManager.storeRememberedValue(time,faultData, batteryVoltage, UNIT_VOLT);
}

void turnPiOffForced(long time){
	float batteryVoltageBefore = getBatteryVoltage();
	digitalWrite(PI_POWER_PIN, LOW);
	delay(1000);
	float batteryVoltageAfter = getBatteryVoltage();
	float voltageDifferential = 1-(batteryVoltageBefore/batteryVoltageAfter);
	sdCardManager.storeRememberedValue(time,FORCED_PI_TURN_OFF,0 , operatingStatus);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_BEFORE_PI_ON, batteryVoltageBefore, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_ATER_PI_ON, batteryVoltageBefore, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_DIFFERENTIAL_AFTER_PI_ON, voltageDifferential, UNIT_PERCENTAGE);
}

void turnPiOff(long time){
	float batteryVoltageBefore = getBatteryVoltage();
	digitalWrite(PI_POWER_PIN, LOW);
	delay(1000);
	float batteryVoltageAfter = getBatteryVoltage();
	float voltageDifferential = 1-(batteryVoltageBefore/batteryVoltageAfter);
	sdCardManager.storeRememberedValue(time,PI_TURN_OFF,0 , operatingStatus);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_BEFORE_PI_ON, batteryVoltageBefore, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_ATER_PI_ON, batteryVoltageBefore, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_DIFFERENTIAL_AFTER_PI_ON, voltageDifferential, UNIT_PERCENTAGE);
}


void turnPiOn(long time){
	float batteryVoltageBefore = getBatteryVoltage();
	digitalWrite(PI_POWER_PIN, HIGH);
	delay(1000);
	float batteryVoltageAfter = getBatteryVoltage();
	float voltageDifferential = 1-(batteryVoltageAfter/batteryVoltageBefore);

	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_BEFORE_PI_ON, batteryVoltageBefore, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_ATER_PI_ON, batteryVoltageBefore, UNIT_VOLT);
	sdCardManager.storeRememberedValue(time,BATTERY_VOLTAGE_DIFFERENTIAL_AFTER_PI_ON, voltageDifferential, UNIT_PERCENTAGE);
}



void defineState(long time, float batteryVoltage,int internalBatteryStateOfCharge, float currentValue, boolean piIsOn){

	if(shuttingDownPiCountdown){
		currentSecondsToPowerOff = secondsToTurnPowerOff -( time - shutDownRequestedseconds );
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Shutting down");
		lcd.setCursor(0,1);

		if(currentSecondsToPowerOff<=0){
			shuttingDownPiCountdown=false;
			manualShutdown=true;
			inPulse=false;
			turnPiOff(time);
			sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_MANUAL_SHUTDOWN, LIFE_CYCLE_EVENT_COMMA_VALUE);
			lcd.print("Pi is OFF");
			currentViewIndex=3;
		}else{
			lcd.print("in ");
			lcd.print(	currentSecondsToPowerOff);
		}
	}else if(batteryVoltage>exitWPSVoltage){
		if(!piIsOn && !manualShutdown)turnPiOn(time);
		operatingStatus="Normal";
		lcd.setRGB(0, 225, 0);
		operatingStatus="Normal";
		wpsCountdown=false;
		wpsSleeping=false;
		inWPS=false;
		waitingForWPSConfirmation=false;

		if(inPulse){
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("Executing Pulse" );
			lcd.setCursor(0, 1);
			lcd.print( "Started at " );
			lcd.print(  pulseStartTime );
		}else{
			//
			// if we are here it means
			// that we are not in pulse and not in wps
			// so display user data according to the value of
			// currentViewIndex
			// currentViewIndex = 0 means show main data
			// currentViewIndex = 1 means Generate Password
			// currentViewIndex = 2 show Network info
			// currentViewIndex = 3 means shutdown request
			// currentViewIndex = 4 shutdown in process
			// i
			switch(currentViewIndex){
			case 0:
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print((int)currentValue);
				lcd.print("mA ") ;
				lcd.print(batteryVoltage) ;
				lcd.print("V ") ;
				lcd.print(internalBatteryStateOfCharge);
				lcd.print("%") ;
				lcd.setCursor(0, 1);
				lcd.print(timeManager.getCurrentDateTimeForDisplay());
				break;

			case 1:
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Create Password");
				lcd.setCursor(0, 1);
				lcd.print(" ");
				break;
			case 2:
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(currentSSID);
				lcd.setCursor(0, 1);
				lcd.print(currentIpAddress);
				break;
			case 3:
				lcd.clear();
				lcd.setCursor(0, 0);
				if(manualShutdown){
					lcd.print("Pi is Off");
					lcd.setCursor(0, 1);
					lcd.print("Turn On Pi?");
				}else if(waitingManualPiStart){
					lcd.print("Waiting for Pi" );
					lcd.setCursor(0, 1);
					lcd.print("To Start" );

				}else{
					lcd.print("Turn Off Pi");
					lcd.setCursor(0, 1);
					lcd.print("Are You Sure?");
				}

				break;

			case 30:
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Shutting Down Pi" );
				lcd.setCursor(0, 1);
				lcd.print(" " );
				break;


			case 35:
				// the pi was just turned on
				// manually
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Waiting for Pi" );
				lcd.setCursor(0, 1);
				lcd.print("To Start" );
				currentViewIndex=3;
				waitingManualPiStart=true;
				break;
			}
		}
	}else if(batteryVoltage>enterWPSVoltage && batteryVoltage<=exitWPSVoltage){
		if(wpsSleeping){
			//delay(1000);
			//lcd.noDisplay();
			long piSleepingRemaining = secondsToNextPiOn-(time - currentSleepStartTime) ;
			lcd.clear();
			lcd.display();
			lcd.setCursor(0,0);
			lcd.setRGB(255,255,0);

			if(piSleepingRemaining<=0){
				wpsSleeping=false;
				if(!digitalRead(PI_POWER_PIN))turnPiOn(time);
				//sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_EVENT_END_WPS, LIFE_CYCLE_EVENT_WPS_VALUE);

				lcd.print("Pi ON WPS ");
				lcd.setCursor(0,1);
				lcd.print(batteryVoltage);
				lcd.print("V ");
				lcd.print(internalBatteryStateOfCharge);
				lcd.print("%") ;
				lastWPSStartUp = time;
			}else{
				//
				// if we are here is because we are in the
				// sleep period of the wps cycle.
				// check to see if we need to store a record in the sd card
				//
				long z =time-lastWPSRecordSeconds;
				lcd.print("wps rec in ");
				long netWPSRecordIn = (long)wpsPulseFrequencySeconds-z;

				lcd.print(netWPSRecordIn);
				lcd.setCursor(0,1);
				lcd.print("pi on in ");
				long piremaining = secondsToNextPiOn-(time - currentSleepStartTime) ;
				lcd.print(piremaining);


				//delay(1000);

				if(netWPSRecordIn<=0){
					lcd.clear();
					lcd.display();

					lastWPSRecordSeconds = timeManager.getCurrentTimeInSeconds();
					WPSSensorRecord anWPSSensorRecord;
					anWPSSensorRecord.batteryVoltage= getBatteryVoltage();
					anWPSSensorRecord.current = calculateCurrent();
					anWPSSensorRecord.stateOfCharge = generalFunctions.getStateOfCharge(batteryVoltage);
					anWPSSensorRecord.lastWPSRecordSeconds=lastWPSRecordSeconds;
					anWPSSensorRecord.hourlyBatteryOutEnergy=hourlyBatteryOutEnergy;
					anWPSSensorRecord.dailyBatteryOutEnergy=dailyBatteryOutEnergy;
					anWPSSensorRecord.hourlyPoweredDownInLoopSeconds=hourlyPoweredDownInLoopSeconds;
					anWPSSensorRecord.dailyPoweredDownInLoopSeconds=dailyPoweredDownInLoopSeconds;
					anWPSSensorRecord.pauseDuringWPS=pauseDuringWPS;
					anWPSSensorRecord.operatingStatus=operatingStatus;
					anWPSSensorRecord.totalDiskUse= 0; //sdCardManager.getDiskUsage();


					//sdCardManager.saveWPSSensorRecord( anWPSSensorRecord);
					lcd.setRGB(255,255,0);
				}else{
					//
					// if we are here is because we are in the sleeping part of the
					// WPS and is not time to take another record
					// now check if there is any reason to keep it from comma
					// ie if its raining and the sensor needs to stay on
					// if not sleep for 8 seconds
					//


					if(pauseDuringWPS){
						pauseWPS();
					}
				}
			}
		}else if(piIsOn){
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("pi ON WPS ");
			lcd.print(batteryVoltage);
			lcd.print(" V");
			lcd.setCursor(0,1);
			lcd.print("Runtime ");
			long secsRunning = time-lastWPSStartUp;
			lcd.print(secsRunning);
		}
	}else if(batteryVoltage>minWPSVoltage && batteryVoltage<=enterWPSVoltage){
		if(!inWPS){
			faultData="Enter WPS";
			sendWPSAlert(time, faultData, batteryVoltage);
			lcd.clear();
			lcd.setRGB(225, 225, 0);
			lcd.setCursor(0, 0);
			lcd.print("WPS Alert Sent");

		}else{
			if(waitingForWPSConfirmation){
				delay(1000);
				long z = time-wpsAlertTime;
				long remaining = secondsToForcedWPS-z;
				lcd.clear();
				lcd.setRGB(225,225,0);
				lcd.setCursor(0,0);

				if( remaining <= 0  ){
					waitingForWPSConfirmation=false;
					operatingStatus="WPS";
					//sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_EVENT_FORCED_START_WPS, LIFE_CYCLE_EVENT_WPS_VALUE);
					lcd.print("pi off");
					wpsSleeping=true;
					currentSleepStartTime = time;
					currentSecondsToPowerOff=0L;
					turnPiOff(time);
					wpsCountdown=false;
				}else{
					lcd.print("Waiting EnterWPS");
					lcd.setCursor(0,1);
					long remaining = secondsToForcedWPS-z;
					lcd.print(remaining);
					lcd.print("  ");
					lcd.print(batteryVoltage);
					lcd.print("V ");
				}
			}else if(wpsCountdown){
				currentSecondsToPowerOff = secondsToTurnPowerOff -( time - wpsCountDownStartSeconds);
				lcd.clear();
				lcd.setCursor(0,0);
				lcd.print("wps countdown ");
				lcd.setCursor(0,1);
				lcd.print(	currentSecondsToPowerOff);
				if(currentSecondsToPowerOff<=0){
					currentSecondsToPowerOff=0;
					turnPiOff(time);
					//sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_EVENT_START_WPS, LIFE_CYCLE_EVENT_WPS_VALUE);
					wpsSleeping=true;
					wpsCountdown=false;
					currentSleepStartTime=time;
				}
			}else if(wpsSleeping){
				//delay(1000);
				//lcd.noDisplay();
				long piSleepingRemaining = secondsToNextPiOn-(time - currentSleepStartTime) ;
				lcd.clear();
				lcd.display();
				lcd.setCursor(0,0);
				lcd.setRGB(255,255,0);

				if(piSleepingRemaining<=0){
					wpsSleeping=false;
					if(!digitalRead(PI_POWER_PIN))turnPiOn(time);
					//sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_EVENT_END_WPS, LIFE_CYCLE_EVENT_WPS_VALUE);

					lcd.print("Pi ON WPS ");
					lcd.setCursor(0,1);
					lcd.print(batteryVoltage);
					lcd.print("V ");
					lcd.print(internalBatteryStateOfCharge);
					lcd.print("%") ;
					lastWPSStartUp = time;
				}else{
					//
					// if we are here is because we are in the
					// sleep period of the wps cycle.
					// check to see if we need to store a record in the sd card
					//
					long z =time-lastWPSRecordSeconds;
					lcd.print("WPS rec in ");
					long netWPSRecordIn = (long)wpsPulseFrequencySeconds-z;

					lcd.print(netWPSRecordIn);
					lcd.setCursor(0,1);
					lcd.print("pi on in ");
					long piremaining = secondsToNextPiOn-(time - currentSleepStartTime) ;
					lcd.print(piremaining);


					//delay(1000);

					if(netWPSRecordIn<=0){
						lcd.clear();
						lcd.display();

						lastWPSRecordSeconds = timeManager.getCurrentTimeInSeconds();
						WPSSensorRecord anWPSSensorRecord;
						anWPSSensorRecord.batteryVoltage= getBatteryVoltage();
						anWPSSensorRecord.current = calculateCurrent();
						anWPSSensorRecord.stateOfCharge = generalFunctions.getStateOfCharge(batteryVoltage);
						anWPSSensorRecord.lastWPSRecordSeconds=lastWPSRecordSeconds;
						anWPSSensorRecord.hourlyBatteryOutEnergy=hourlyBatteryOutEnergy;
						anWPSSensorRecord.dailyBatteryOutEnergy=dailyBatteryOutEnergy;
						anWPSSensorRecord.hourlyPoweredDownInLoopSeconds=hourlyPoweredDownInLoopSeconds;
						anWPSSensorRecord.dailyPoweredDownInLoopSeconds=dailyPoweredDownInLoopSeconds;
						anWPSSensorRecord.pauseDuringWPS=pauseDuringWPS;
						anWPSSensorRecord.operatingStatus=operatingStatus;
						anWPSSensorRecord.totalDiskUse=989; //sdCardManager.getDiskUsage();

						//sdCardManager.saveWPSSensorRecord( anWPSSensorRecord);
						lcd.setRGB(255,255,0);
					}else{
						//
						// if we are here is because we are in the sleeping part of the
						// WPS and is not time to take another record
						// now check if there is any reason to keep it from comma
						// ie if its raining and the sensor needs to stay on
						// if not sleep for 8 seconds
						//

						if(pauseDuringWPS){
							pauseWPS();
						}
					}
				}
			}else{
				if(piIsOn){
					lcd.clear();
					lcd.setCursor(0,0);
					lcd.print("pi ON WPS ");
					lcd.print(batteryVoltage);
					lcd.print(" V");
					lcd.setCursor(0,1);
					lcd.print("Runtime ");
					long secsRunning = time-lastWPSStartUp;
					lcd.print(secsRunning);
				}else{

				}

			}
		}

	}else if(batteryVoltage<minWPSVoltage ){
		if(!inWPS ){
			faultData="Enter WPS";
			sendWPSAlert(time, faultData, batteryVoltage);
			lcd.clear();
			lcd.setRGB(225, 0, 0);
			lcd.setCursor(0, 0);
			lcd.print("Comma Alert Sent");

		}else{
			if(waitingForWPSConfirmation){
				delay(1000);
				long z = time-wpsAlertTime;
				long remaining = secondsToForcedWPS-z;
				lcd.clear();
				lcd.setCursor(0,0);
				lcd.setRGB(255,0,0);
				lcd.setCursor(0,0);
				if( remaining <= 0  ){
					waitingForWPSConfirmation=false;
					operatingStatus="WPS";
					//sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_EVENT_FORCED_START_WPS, LIFE_CYCLE_EVENT_WPS_VALUE);
					wpsSleeping=false;
					currentSecondsToPowerOff=0L;
					if(piIsOn)turnPiOff(time);
					wpsCountdown=false;

					if(f_wdt == 1){
						/* Don't forget to clear the flag. */
						f_wdt = 0;
						/* Re-enter sleep mode. */
						lcd.print("Enter Comma");
						operatingStatus="Comma";
						lcd.setCursor(0,1);
						lcd.print(batteryVoltage);
						lcd.print(" V");
						delay(2000);
						lcd.setRGB(0,0,0);
						lcd.noDisplay();
						//sdCardManager.storeLifeCycleEvent(time,LIFE_CYCLE_EVENT_START_COMMA, LIFE_CYCLE_EVENT_COMMA_VALUE);
						enterArduinoSleep();
					}
				}else{
					lcd.print("Waiting EnterWPS");
					lcd.setCursor(0,1);
					long remaining = secondsToForcedWPS-z;
					lcd.print(remaining);
					lcd.print("  ");
					lcd.print(batteryVoltage);
					lcd.print("V ");
				}
			}else if(wpsCountdown){
				currentSecondsToPowerOff = secondsToTurnPowerOff -( time - wpsCountDownStartSeconds);
				lcd.clear();
				lcd.setCursor(0,0);
				lcd.print("wps countdown ");
				lcd.setCursor(0,1);
				lcd.print(	currentSecondsToPowerOff);
				if(currentSecondsToPowerOff<=0){
					currentSecondsToPowerOff=0;
					if(piIsOn)turnPiOff(time);
					//sdCardManager.storeLifeCycleEvent(time, LIFE_CYCLE_EVENT_START_WPS, LIFE_CYCLE_EVENT_WPS_VALUE);
					wpsSleeping=false;
					wpsCountdown=false;
					if(f_wdt == 1){
						/* Don't forget to clear the flag. */
						f_wdt = 0;
						/* Re-enter sleep mode. */
						lcd.print("Enter Comma");
						operatingStatus="Comma";
						lcd.setCursor(0,1);
						lcd.print(batteryVoltage);
						lcd.print(" V");
						delay(2000);
						lcd.setRGB(0,0,0);
						lcd.noDisplay();
						//sdCardManager.storeLifeCycleEvent(time,LIFE_CYCLE_EVENT_START_COMMA, LIFE_CYCLE_EVENT_COMMA_VALUE);
						enterArduinoSleep();
					}
				}
			}else if(wpsSleeping){
				//
				// if the pi is asleep then go into a comma
				//
				if(f_wdt == 1){
					/* Don't forget to clear the flag. */
					f_wdt = 0;
					/* Re-enter sleep mode. */
					lcd.clear();
					lcd.setRGB(255,0,0);
					lcd.setCursor(0,0);
					lcd.print("Enter Comma");
					operatingStatus="Comma";
					lcd.setCursor(0,1);
					lcd.print(batteryVoltage);
					lcd.print(" V");
					delay(2000);
					lcd.setRGB(0,0,0);
					lcd.noDisplay();
					//sdCardManager.storeLifeCycleEvent(time,LIFE_CYCLE_EVENT_START_COMMA, LIFE_CYCLE_EVENT_COMMA_VALUE);
					enterArduinoSleep();
				}
			}else if(piIsOn){
				//
				// i we are here it means the pi is n
				// and voltage has dropped into
				// comma range so
				faultData="Enter WPS";
				sendWPSAlert(time, faultData, batteryVoltage);
				lcd.clear();
				lcd.setRGB(225, 0, 0);
				lcd.setCursor(0, 0);
				lcd.print("Comma Alert Sent");
			}
		}
	}
}

boolean processDefaultCommands(String command, float batteryVoltage){
	boolean processed=false;
	if(command=="TestWPSSensor"){
		float batteryVoltage = getBatteryVoltage();
		float current = calculateCurrent();
		int stateOfCharge= generalFunctions.getStateOfCharge(batteryVoltage);
		boolean result = true;//sdCardManager.testWPSSensor( batteryVoltage,  current,  stateOfCharge,  operatingStatus);
		if(result){
			Serial.println("Ok-TestWPSSensor");
		}else{
			Serial.println("Failure-TestWPSSensor");
		}
		Serial.flush();
		processed=true;
	}else if(command=="TestLifeCycle"){
		long now = timeManager.getCurrentTimeInSeconds();
		//sdCardManager.storeLifeCycleEvent(now, LIFE_CYCLE_EVENT_END_COMMA, LIFE_CYCLE_EVENT_COMMA_VALUE);
		Serial.println("Ok-TestLifeCycle");
		Serial.flush();

	}else if(command=="ListFiles"){
		Serial.println(" ");
		Serial.println(" ");
		Serial.println(sensorDirName);
		float total = 0.0;//sdCardListFiles();


		Serial.println(" ");

		Serial.print("Used (Kb):  ");
		Serial.println(total);

		Serial.println("");
		Serial.println("Ok-ListFiles");
		Serial.flush();
		processed=true;
	}else if(command=="Ping"){

		Serial.println("Ok-Ping");
		Serial.flush();
		processed=true;
	}else if(command.startsWith("SetTime")){

		if(capacitorVoltage==0){
			//
			// we are in normal operation
			//
			Serial.println("Failure-SetTime");
			Serial.flush();

		}else{
			boolean result = timeManager.setTime(command);
			if(result){
				Serial.println("Ok-SetTime");
			}else{
				Serial.println("Failure-SetTime");
			}

			Serial.flush();
		}
		processed=true;

	}else if(command.startsWith("GetTime")){
		String time = timeManager.getCurrentDateTimeForDisplay();
		Serial.println(time);
		Serial.flush();
		Serial.println("Ok-GetTime");
		Serial.flush();
		processed=true;
	}else if(command.startsWith("VerifyUserCode")){
		String codeInString = generalFunctions.getValue(command, '#', 1);
		long userCode = codeInString.toInt();
		boolean validCode = secretManager.checkCode( userCode);
		String result="Failure-Invalid Code";
		if(validCode)result="Ok-Valid Code";
		Serial.println(result);
		Serial.flush();
		delay(delayTime);
		processed=true;
	}else if(command.startsWith("GetCommandCodeGenerationTime")){

		long secOrig =timeManager.getTimeForCodeGeneration();

		Serial.print("secOrig=");
		Serial.println(secOrig);
		Serial.flush();
		char secretCode[SHARED_SECRET_LENGTH];
		secretManager.readSecret(secretCode);
		Serial.print("secretCode=");
		Serial.println(secretCode);
		Serial.flush();

		TOTP totp = TOTP(secretCode);
		long code=totp. gen_code  (secOrig ) ;


		//long code =secretManager.generateCode();
		Serial.print("code=");
		Serial.println(code);
		Serial.println("Ok-GetCommandCodeGenerationTime");
		Serial.flush();
		delay(delayTime);
		processed=true;
	}else if(command.startsWith("GetCommandCode")){

		long code =secretManager.generateCode();
		//
		// patch a bug in the totp library
		// if the first digit is a zero, it
		// returns a 5 digit number
		if(code<100000){
			Serial.print("0");
			Serial.println(code);
		}else{
			Serial.println(code);
		}
		Serial.flush();
		delay(delayTime);
		processed=true;
	}else if(command.startsWith("GetSecret")){
		if(capacitorVoltage==0){
			//
			// we are in normal operation
			//
			Serial.println("Failure-GetSecret");
			Serial.flush();
		}else{
			char secretCode[SHARED_SECRET_LENGTH];
			secretManager.readSecret(secretCode);
			Serial.println(secretCode);
			Serial.println("Ok-GetSecret");
			Serial.flush();
			delay(delayTime);
		}

		processed=true;
	} else if(command.startsWith("SetSecret")){
		if(capacitorVoltage==0){
			//
			// we are in normal operation
			//
			Serial.println("Failure-SetSecret");
			Serial.flush();
		}else{
			String secret = generalFunctions.getValue(command, '#', 1);
			int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
			int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
			secretManager.saveSecret(secret, numberDigits, periodSeconds);

			Serial.println("Ok-SetSecret");
			Serial.flush();
		}
		delay(delayTime);
		processed=true;
	}else if(command.startsWith("PulseStart")){
		inPulse=true;
		waitingManualPiStart=false;
		pulseStartTime = generalFunctions.getValue(command, '#', 1);
		Serial.println("Ok-PulseStart");
		Serial.flush();
		lcd.clear();
		lcd.setRGB(255,0,0);
		processed=true;
	}else if(command.startsWith("PulseFinished")){
		pulseStopTime = generalFunctions.getValue(command, '#', 1);
		inPulse=false;
		Serial.println("Ok-PulseFinished");
		Serial.flush();
		lcd.clear();
		lcd.setRGB(255,255,255);
		processed=true;


	}else if(command.startsWith("IPAddr")){
		currentIpAddress = generalFunctions.getValue(command, '#', 1);
		Serial.println("Ok-IPAddr");
		Serial.flush();
		delay(delayTime);
		processed=true;
	}else if(command.startsWith("SSID")){
		currentSSID = generalFunctions.getValue(command, '#', 1);
		Serial.println("Ok-currentSSID");
		Serial.flush();
		delay(delayTime);
		processed=true;
	}else if(command.startsWith("HostMode")  ){
		Serial.println("Ok-HostMode");
		Serial.flush();
		delay(delayTime);
		isHost=true;
		processed=true;
	}else if(command.startsWith("NetworkMode")   ){
		Serial.println("Ok-NetworkMode");
		Serial.flush();
		delay(delayTime);
		isHost=false;
		processed=true;
	}else if(command.startsWith("GetSensorData")){
		Serial.println(toReturn);
		Serial.flush();
		processed=true;
	}else if(command.startsWith("EnterWPS")){
		//EnterWPS#10#45#30#1
		secondsToTurnPowerOff = (long)generalFunctions.getValue(command, '#', 1).toInt();
		secondsToNextPiOn = (long)generalFunctions.getValue(command, '#', 2).toInt();
		wpsPulseFrequencySeconds = generalFunctions.getValue(command, '#', 3).toInt();
		int pauseDuringWPSi = generalFunctions.getValue(command, '#', 4).toInt();
		if(pauseDuringWPSi==1)pauseDuringWPS=true;
		else pauseDuringWPS=false;
		waitingForWPSConfirmation=false;
		wpsCountdown=true;
		operatingStatus="WPS";
		wpsCountDownStartSeconds= timeManager.getCurrentTimeInSeconds();
		currentSecondsToPowerOff=0L;

		Serial.println("Ok-EnterWPS");
		Serial.flush();
		processed=true;
	}else if(command.startsWith("ExitWPS")){

		Serial.println("Ok-ExitWPS");
		Serial.flush();
		inWPS=false;
		operatingStatus="Normal";
		currentSecondsToPowerOff=0L;
		wpsCountdown=false;
		processed=true;
	}else if(command.startsWith("UpdateWPSParameters")){
		String minWPSVoltageS = generalFunctions.getValue(command, '#', 1);
		char buffer[10];
		minWPSVoltageS.toCharArray(buffer, 10);
		minWPSVoltage = atof(buffer);

		minWPSVoltage = generalFunctions.stringToFloat(generalFunctions.getValue(command, '#', 1));
		enterWPSVoltage = generalFunctions.stringToFloat(generalFunctions.getValue(command, '#', 2));
		exitWPSVoltage = generalFunctions.stringToFloat(generalFunctions.getValue(command, '#', 3));

		secondsToForcedWPS = generalFunctions.getValue(command, '#', 4).toInt();
		Serial.println("Ok-UpdateWPSParameters");
		Serial.flush();

		processed=true;

	}else if(command.startsWith("GetRememberedValueData")){
		//GetRememberedValueData#0
		int transferData = generalFunctions.getValue(command, '#', 1).toInt();
		boolean result = true;//sdCardManager.readUntransferredFileFromSDCard( transferData,true, RememberedValueDataDirName);
		if(result){
			Serial.println("Ok-GetRememberedValueData");
		}else {
			char text[44];
			snprintf(text, sizeof text, "Failure-error opening %s/%s", remFileName, unstraferedFileName);
			Serial.println(text);
		}
		Serial.flush();
		processed=true;
	}else if(command.startsWith("GetLifeCycleData")){
		//GetLifeCycleData#0
		int transferData = generalFunctions.getValue(command, '#', 1).toInt();
		boolean result = true;//sdCardManager.readUntransferredFileFromSDCard( transferData,true, LifeCycleDataDirName);
		if(result){
			Serial.println("Ok-GetLifeCycleData");
		}else {
			char text[44];
			snprintf(text, sizeof text, "Failure-error opening %s/%s", LifeCycleDataDirName, unstraferedFileName);
			Serial.println(text);
		}
		Serial.flush();
		processed=true;
	}else if(command.startsWith("GetWPSSensorData")){
		//GetWPSSensorData#0
		//GetLifeCycleData#0
		int transferData = generalFunctions.getValue(command, '#', 1).toInt();
		boolean result = true;//sdCardManager.readUntransferredFileFromSDCard( transferData,true, WPSSensorDataDirName);
		if(result){
			Serial.println("Ok-GetWPSSensorData");
		}else {

			char text[44];
			snprintf(text, sizeof text, "Failure-error opening /%s/%s", WPSSensorDataDirName, unstraferedFileName);
			Serial.println(text);

		}
		Serial.flush();
		processed=true;
	}else if(command.startsWith("GetHistoricalWPSSensorData")){

		int date = generalFunctions.getValue(command, '#', 1).toInt();
		int month = generalFunctions.getValue(command, '#', 2).toInt();
		int year = generalFunctions.getValue(command, '#', 3).toInt();
		boolean result  = true;//sdCardManager.getHistoricalData( WPSSensorDataDirName,  date,  month,  year);
		if(result){
			Serial.println("Ok-GetWPSSensorDataHistory");
		}else {
			char text[44];
			snprintf(text, sizeof text, "Failure-error opening %s/%s", WPSSensorDataDirName, unstraferedFileName);

			Serial.println(text);
		}
		Serial.flush();
		processed=true;
	}else if(command.startsWith("GetHistoricalLifeCycleData")){
		//GetHistoricalLifeCycleData#12#1#19
		int date = generalFunctions.getValue(command, '#', 1).toInt();
		int month = generalFunctions.getValue(command, '#', 2).toInt();
		int year = generalFunctions.getValue(command, '#', 3).toInt();
		boolean result  = true;//sdCardManager.getHistoricalData( LifeCycleDataDirName,  date,  month,  year);
		if (result) {
			Serial.println("Ok-GetHistoricalLifeCycleData");
		}else {
			char text[44];
			snprintf(text, sizeof text, "Failure-error opening %s/%s", LifeCycleDataDirName, unstraferedFileName);
			Serial.println(text);
		}
		Serial.flush();
		processed=true;
	}else if(command.startsWith("GetHistoricalRememberedValueData")){
		//GetHistoricalLifeCycleData#12#1#19
		int date = generalFunctions.getValue(command, '#', 1).toInt();
		int month = generalFunctions.getValue(command, '#', 2).toInt();
		int year = generalFunctions.getValue(command, '#', 3).toInt();
		boolean result  = true;//sdCardManager.getHistoricalData( RememberedValueDataDirName,  date,  month,  year);
		if (result) {
			Serial.println("Ok-GetHistoricalRememberedValueData");
		}else {
			char text[44];
			snprintf(text, sizeof text, "Failure-error opening %s/%s", RememberedValueDataDirName, unstraferedFileName);

			Serial.println(text);
		}
		Serial.flush();
		processed=true;
	}else if (command == "AsyncData" ){
		Serial.println("Ok-No Data");
		Serial.flush();
		processed=true;
	}else if (command.startsWith("FaultData") ){
		//Serial.println(faultData);
		if(faultData=="Enter WPS"){

			Serial.print("Fault#WPS Alert#Enter WPS#");
			Serial.print(secretManager.generateCode());

			Serial.print("#@On Load:Notify And Shutdown:Voltage At WPS#");
			Serial.println(batteryVoltage);
			waitingForWPSConfirmation=true;

		}else{
			Serial.println("Ok");
		}

		Serial.flush();
		faultData="";
		delay(delayTime);
		processed=true;
	}else if (command.startsWith("UserCommand") ){
		//
		// this function is not used in Ra2
		// because Ra2 has no buttons
		// but in the case that a teleonome does have
		//human interface buttons connected to the microcontrller
		// or there is a timer, here is where it will
		Serial.println("Ok-UserCommand");
		Serial.flush();
		delay(delayTime);
		processed=true;
	}else if (command.startsWith("TimerStatus") ){
		//
		// this function is not used in Ra2
		// because Ra2 has no btimers
		// but in the case that a teleonome does have
		//human interface buttons connected to the microcontrller
		// or there is a timer, here is where it will be
		Serial.println("Ok-TimerStatus");
		Serial.flush();
		delay(delayTime);
		processed=true;

	}
	return processed;
}


void processButtons(){
	if(digitalRead(selectPin)== LOW ){
		if(currentViewIndex<3){
			currentViewIndex++;
		}else if(currentViewIndex==3){
			currentViewIndex=0;
		}else if(currentViewIndex==10){
			//
			// if we are here it means the user generated a new password
			// so next time the user toggles the select button
			// go back to the order
			currentViewIndex=2;
		}else if(currentViewIndex==30){
			//
			// if we are here it means the user choose
			// to turn off the power supply
			// go back to the order
			currentViewIndex=2;

		}else{
			currentViewIndex=0;
		}
	}
	if(digitalRead(actionPin)== LOW ){
		showingAct=true;
		// currentViewIndex = 0 means show main data
		// currentViewIndex = 1 means Generate Password
		// currentViewIndex = 2 show Network info
		// currentViewIndex = 3 means shutdown/powerup request


		if(currentViewIndex==1 || currentViewIndex==10){
			currentViewIndex=10;
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("New Password" );
			lcd.setCursor(0, 1);
			char*  pass = generalFunctions.generatePassword();
			lcd.print(pass );
		}else if(currentViewIndex==3){
			if(manualShutdown){
				//
				// if we are here it means the
				// pu was manually turned off and now the
				// user is turning it on again
				// set the flag and let
				// define state turn it on
				manualShutdown=false;
				currentViewIndex=35;

			}else{
				currentViewIndex=30;
				Serial.println("Shutdown");
				Serial.flush();
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Shutting Down Pi" );
				lcd.setCursor(0, 1);
				lcd.print(" " );
				shutDownRequestedseconds= timeManager.getCurrentTimeInSeconds();
				shuttingDownPiCountdown=true;
			}




		}

	}
}

void setup() {
	// put your setup code here, to run once:
	lcd.begin(16,2);
	lcd.print("initializing" );
	Serial.begin(9600);
	timeManager.start();
	sdCardManager.start();
	long now = timeManager.getCurrentTimeInSeconds();
	sdCardManager.storeLifeCycleEvent(now, LIFE_CYCLE_EVENT_SETUP_COMPLETED, LIFE_CYCLE_EVENT_COMMA_VALUE);
	lcd.clear();
	lcd.setCursor(0, 0);

	pinMode(CURRENT_SENSOR, INPUT);
	pinMode(PI_POWER_PIN, OUTPUT);

	//
	// the order in cleo2 has  four leds
	// in two rows of two:
	// top row-left is led 2 - battery voltage from ra
	// top row-right is led 1 - MonoNanny Pump Status
	// bottom row-left is led 3 - Sento temperature
	// bottom row-right is led 0 - Sento Pump Status

	leds.setColorRGB(0, 255, 0, 255);
	leds.setColorRGB(1, 255, 0, 255);
	leds.setColorRGB(2, 255, 0, 255);
	leds.setColorRGB(3, 255, 0, 255);

	pinMode(selectPin, INPUT_PULLUP);
	pinMode(actionPin, INPUT_PULLUP);
	lcd.print("initializing" );
}



void loop() {


	processButtons();
	wdt_reset();


	float batteryVoltage = getBatteryVoltage();
	int internalBatteryStateOfCharge = generalFunctions.getStateOfCharge(batteryVoltage);
	float currentValue = calculateCurrent();
	long  lockCapacitorValue=analogRead(LOCK_CAPACITOR_PIN);
	capacitorVoltage= lockCapacitorValue * (5.0 / 1023.0);
	boolean piIsOn = digitalRead(PI_POWER_PIN);
	ambientTemperature = dht.readTemperature();
	ambientHumidity = dht.readHumidity();







	//
	// Generate the SensorData String
	toReturn="";
	//
	// Sensor Request Queue Position 1
	//
	char batteryVoltageStr[15];
	dtostrf(batteryVoltage,4, 1, batteryVoltageStr);
	toReturn.concat(batteryVoltageStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 2
	//
	char currentValueStr[15];
	dtostrf(currentValue,4, 0, currentValueStr);
	toReturn.concat(currentValueStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 3
	//
	char capacitorVoltageStr[15];
	dtostrf(capacitorVoltage,2, 1, capacitorVoltageStr);
	toReturn.concat(capacitorVoltageStr) ;
	toReturn.concat("#") ;


	//
	// Sensor Request Queue Position 4
	//
	toReturn.concat( internalBatteryStateOfCharge);
	toReturn.concat("#") ;
	//
	// Sensor Request Queue Position 5
	//

	toReturn.concat( operatingStatus);
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 6
	//

	char dailyMinBatteryVoltageStr[15];
	dtostrf(dailyMinBatteryVoltage,4, 0, dailyMinBatteryVoltageStr);
	toReturn.concat(dailyMinBatteryVoltageStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 7
	//

	char dailyMaxBatteryVoltageStr[15];
	dtostrf(dailyMaxBatteryVoltage,4, 0, dailyMaxBatteryVoltageStr);
	toReturn.concat(dailyMaxBatteryVoltageStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 8
	//

	char dailyMinBatteryCurrentStr[15];
	dtostrf(dailyMinBatteryCurrent,4, 0, dailyMinBatteryCurrentStr);
	toReturn.concat(dailyMinBatteryCurrentStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 9
	//

	char dailyMaxBatteryCurrentStr[15];
	dtostrf(dailyMaxBatteryCurrent,4, 0, dailyMaxBatteryCurrentStr);
	toReturn.concat(dailyMaxBatteryCurrentStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 10
	//

	char dailyBatteryOutEnergyStr[15];
	dtostrf(dailyBatteryOutEnergy,4, 0, dailyBatteryOutEnergyStr);
	toReturn.concat(dailyBatteryOutEnergyStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 11
	//

	char dailyPoweredDownInLoopSecondsStr[15];
	dtostrf(dailyPoweredDownInLoopSeconds,4, 0, dailyPoweredDownInLoopSecondsStr);
	toReturn.concat(dailyPoweredDownInLoopSecondsStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 12
	//

	char hourlyBatteryOutEnergyStr[15];
	dtostrf(hourlyBatteryOutEnergy,4, 0, hourlyBatteryOutEnergyStr);
	toReturn.concat(hourlyBatteryOutEnergyStr) ;
	toReturn.concat("#") ;
	//
	// Sensor Request Queue Position 13
	//

	char hourlyPoweredDownInLoopSecondsStr[15];
	dtostrf(hourlyPoweredDownInLoopSeconds,4, 0, hourlyPoweredDownInLoopSecondsStr);
	toReturn.concat(hourlyPoweredDownInLoopSecondsStr) ;
	toReturn.concat("#") ;

	//
	// Sensor Request Queue Position 14
	//

	long totalDiskUse=0;//sdCardManager.getDiskUsage();
	toReturn.concat(totalDiskUse/1024);
	toReturn.concat("#");
	//
	// Sensor Request Queue Position 15
	//

	toReturn.concat(pauseDuringWPS);
	toReturn.concat("#");
	//
	// Sensor Request Queue Position 16
	//
	char ambientTemperatureStr[15];
	dtostrf(ambientTemperature,4, 0, ambientTemperatureStr);
	toReturn.concat(ambientTemperatureStr);
	toReturn.concat("#");
	//
	// Sensor Request Queue Position 17
	//
	char ambientHumidityStr[15];
	dtostrf(ambientHumidity,4, 0, ambientHumidityStr);
	toReturn.concat(ambientHumidityStr);
	toReturn.concat("#");

	//lcd.clear();
	//lcd.setCursor(0, 0);

	long now = timeManager.getCurrentTimeInSeconds();
	poweredDownInLoopSeconds=0;
	defineState(now,  batteryVoltage, internalBatteryStateOfCharge, currentValue, piIsOn);



	if( Serial.available() != 0) {
		// lcd.clear();
		command = Serial.readString();
		lcd.setCursor(0, 0);
		lcd.print(command);
		boolean commandProcessed = processDefaultCommands( command, batteryVoltage);
		if(!commandProcessed){
			if (command.startsWith("UpdateTeleonomeStatus")){
				// UpdateTeleonomeStatus#2#success#Test
				//UpdateTeleonomeStatus#3#success#Test

				int id = generalFunctions.getValue(command, '#', 1).toInt();
				String statusValue = generalFunctions.getValue(command, '#', 2);
				String info = generalFunctions.getValue(command, '#', 3);
				ledStatusLine2[id]=info;

				if(statusValue=="success"){
					leds.setColorRGB(id, 0, 255, 0);
				}else  if(statusValue=="warning"){
					leds.setColorRGB(id, 255, 255, 0);
				}else  if(statusValue=="danger"){
					leds.setColorRGB(id, 255, 0, 0);
				}else  if(statusValue=="primary"){
					leds.setColorRGB(id, 0, 0, 255);
				}else  if(statusValue=="crisis"){
					leds.setColorRGB(id, 255, 165, 0);
				}else  if(statusValue=="off"){
					leds.setColorRGB(id, 0, 0, 0);
				}else  if(statusValue=="stale"){
					leds.setColorRGB(id, 148, 0, 211);
				}
				Serial.println("Ok-UpdateTeleonomeStatus");
				Serial.flush();
				delay(delayTime);
			}else{
				//
				// call read to flush the incoming
				//
				Serial.read();
				Serial.println("Failure-Bad Command " + command);
				Serial.flush();
			}
		}
	}
	//
	// this is the end of the loop, to calculate the energy spent on this loop
	// take the time substract the time at the beginning of the loop (the now variable defined above)
	// and also substract the seconds spent in powerdownMode
	// finally add the poweredDownInLoopSeconds to the daily total

	int loopConsumingPowerSeconds = timeManager.getCurrentTimeInSeconds()-now -poweredDownInLoopSeconds;
	dailyBatteryOutEnergy+= loopConsumingPowerSeconds*currentValue/3600;
	hourlyBatteryOutEnergy+= loopConsumingPowerSeconds*currentValue/3600;
	dailyPoweredDownInLoopSeconds+=poweredDownInLoopSeconds;
	hourlyPoweredDownInLoopSeconds+=poweredDownInLoopSeconds;

}


