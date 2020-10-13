// NZ, Christchurch locatoin
#define LAT -43.388018
#define LONG 172.525346

#define MINUTES_AFTER_SS 150       //Number of minutes after sunset before starting trap
#define MINUTES_BEFORE_SR 60      //Number of minutes before sunrise to stop trap

#include <RTClib.h>       //https://github.com/adafruit/RTClib
#include <Servo.h>
#include <Dusk2Dawn.h>    //https://github.com/dmkishi/Dusk2Dawn


#define PIR_1 2
#define PIR_2 13

#define SKIP_BUTTON 20 // !NOT mounted. DO NOT USE. Pin reference only. 

#define SERVO_1_PIN 5
#define SERVO_1_POWER 17
#define SERVO_2_PIN 6
#define SERVO_2_POWER 17
#define STATUS_LED 2

Servo servo1;
Servo servo2;

RTC_PCF8523 rtc;

bool runAtDay = true;

Dusk2Dawn d2d_chch(LAT, LONG, 12);

void setup() {
  pinMode(PIR_1, INPUT);
  pinMode(PIR_2, INPUT);
  pinMode(SKIP_BUTTON, INPUT_PULLUP);

  pinMode(SERVO_1_PIN, OUTPUT);
  pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_1_POWER, OUTPUT);
  digitalWrite(SERVO_1_POWER, LOW);
  digitalWrite(SERVO_2_POWER, LOW);
  pinMode(SERVO_2_POWER, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  
  Serial.begin(57600);
  Serial.println("Starting setup");

  //run_test();
  //set_time();   //TODO

  if (!runAtDay) {
    initRTC();
  }

  // Booting LED sequence. If skip button is held set to run at the day also.
  for (int i = 0; i <= 20; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(500);
    digitalWrite(STATUS_LED, LOW);
    delay(500);
    if (digitalRead(SKIP_BUTTON) == LOW) {
      Serial.println("Enable running at day time");
      runAtDay = true;
    }
  }

  // init servos
  Serial.println("Resetting servos to 0");
  s1(0);
  s2(0);  

  mySleep(10000);
  Serial.println("Waiting for PIR 1");
  waitFor(PIR_1, HIGH);
  Serial.println("Sweeping servo 1");
  s1(60);
  s1(0);

  mySleep(10000);
  Serial.println("Waiting for PIR 2");
  waitFor(PIR_2, HIGH);
  Serial.println("Sweeping servo 2");
  s2(210);
}

void initRTC() {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC!!!");
    while (true) {
      //TODO LED flash sequence to indicate RTC not found.
    }
  } else {
    Serial.println("Found RTC");
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power!!!");
    while (true) {
      //TODO LED flash sequence to indicate RTC lost power.
    }
  }

  if (!rtc.initialized()) {
    Serial.println("RTC not initialized!!!");
    while (true) {
      //TODO LED flash sequence to indicate RTC is not initialized.
    }
  }
  rtc.start();
  printDateTime(rtc.now());
}

void set_time() {
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!!!");
    while (true) {
      //TODO LED flash sequence to indicate RTC not found.
    }
  } else {
    Serial.println("Found RTC");
  }
  // When time needs to be set on a new device, or after a power loss, the
  // following line sets the RTC to the date & time this sketch was compiled  
  Serial.println("Setting date on RTC");
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void run_test() {
  bool rtcConnected = rtc.begin();
  bool rtcInitialized = false;
  if (!rtcConnected) {
    Serial.println("Couldn't find RTC");
  } else {
    Serial.println("Found RTC");
    rtcInitialized = rtc.initialized();
    if (rtcInitialized) {
      Serial.println("Starting RTC");
      rtc.start();
    } {
      Serial.println("RTC is NOT initialized!");
    }
  }
  

  while (digitalRead(SKIP_BUTTON) == HIGH) {
    
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }

  while(true) {
    if (rtcConnected && rtcInitialized) {
      printDateTime(rtc.now());
    }
    Serial.println("moving Servo 1");
    s1(0);
    s1(90);
    Serial.println("moving Servo 2");
    s2(0);
    s2(90);
    Serial.print("PIR1: ");
    Serial.println(digitalRead(PIR_1));
    Serial.print("PIR2: ");
    Serial.println(digitalRead(PIR_2));
    delay(100);
  }
}

void printDateTime(DateTime now) {
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());
}

void waitFor(int pin, int level) {
  while(true) {
    waitForNight();
    if (digitalRead(pin) == level) {
      Serial.println("triggered");
      break;
    }
    mySleep(100);
  }
}

void waitForNight() {
  if (runAtDay) {
    //Serial.println("run at day time");
    return;
  }
  while(true) {
    DateTime now = rtc.now();
    int minutesFromMidnight = now.hour()*60 + now.minute();
    int startMinute = d2d_chch.sunset(now.year(), now.month(), now.day(), false) + MINUTES_AFTER_SS;
    int stopMinute = d2d_chch.sunrise(now.year(), now.month(), now.day(), false) - MINUTES_BEFORE_SR;
    
    if (minutesFromMidnight < stopMinute) {
      Serial.println("early morning");
      break;
    }
    if (minutesFromMidnight > startMinute) {
      Serial.println("late night");
      break;
    }
    Serial.println("waiting for night");
    mySleep(2000);
  }
}

void mySleep(long mill) {
  /*
  WDTCSR = (24);
  WDTCSR = (33);
  WDTCSR |= (1<<6);

  //Disable ADC
  ADCSRA &= ~(1<<7);

  //Enable Sleep
  SMCR |= (1<<2);
  SMCR |= 1;
  __asm__ __volatile("sleep");
  */
  delay(mill);
}

void s1(int angle) {
  digitalWrite(SERVO_1_POWER, HIGH);
  servo1.write(angle);
  delay(2000);
  digitalWrite(SERVO_1_POWER, LOW);
}

void s2(int angle) {
  digitalWrite(SERVO_2_POWER, HIGH);
  servo2.write(angle);
  delay(2000);
  digitalWrite(SERVO_2_POWER, LOW);
}

void loop() {
  mySleep(1000);  
}
