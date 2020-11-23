// NZ, Christchurch locatoin
#define LAT -43.388018
#define LONG 172.525346

#define MINUTES_AFTER_SS       150  // Number of minutes after sunset before starting trap
#define MINUTES_BEFORE_SR       60  // Number of minutes before sunrise to stop trap

#define SONAR_NUMF              2   // Number of sensors.(3)
#define SONAR_NUMB              2
#define MAX_DISTANCE_FRONT    200   // Maximum distance (in cm) to ping (trap wall)
#define MAX_DISTANCE_BACK     100   // Maximum distance (in cm) to ping. (back) 
#define SENSITIVITY             4   // The difference (cm) in readings for the sensor to detect movement

// Front Trap Ultrasonics
#define triggerPin_front1       8    // 2 alternate sensors connected to this line
#define triggerPin_front2       9    // middle ultrasonic sensors connected to this
#define echoPin_front          12    // Front sensors share the same echo

// Back Trap Ultrasonics (x2)
#define triggerPin_back        11    // 1 trigger shared by both back cage sensors
#define echoPin_back1           0     // trigger for closing the trap back door
#define echoPin_back2           1     // furthest cage   

// PIR Sensors
#define PIR_1                   2     
#define PIR_2                   7     // Main front trap PIR
#define PIR_3                  13

// Peripherals
#define SKIP_BUTTON            20     // A6    
#define MODE_BUTTON            21    // A7 HIGH = run during day
#define STATUS_LED              2          

// Servos
#define SERVO_1_PIN             5
#define SERVO_1_POWER          17
#define SERVO_2_PIN             6
#define SERVO_2_POWER          17
#define SERVO_3_PIN            10
#define SERVO_3_POWER          17
#define ENABLE_6V              A0

// Door Angles (Final pos)
#define FRONT_SERVO_ANGLE      60
#define BACK_SERVO_ANGLE      140

// Linear Actuator
#define MOTOR_ENABLE            3
#define MOTOR_DIRECTION         4


// Declare global variables
bool frontTriggered, backTriggered = false;

/* NewPing Library ultrasonics set up. Add/Delete sensors as required
*  Each sensor's trigger pin, echo pin, and max distance to ping.
*/
NewPing sonarFront[SONAR_NUMF] = {                                // Sensor object array for Front cage sensors.
  NewPing(triggerPin_front1, echoPin_front, MAX_DISTANCE_FRONT),  // 2 sensors attached to these pins! 
  NewPing(triggerPin_front2, echoPin_front, MAX_DISTANCE_FRONT)   // (middle centre ultrasonic) 
};

NewPing sonarBack[SONAR_NUMB] = {                                 // Sensor object array.
  NewPing(triggerPin_back, echoPin_back1, MAX_DISTANCE_BACK),      
  NewPing(triggerPin_back, echoPin_back2, MAX_DISTANCE_BACK) 
};

// Servos
Servo servo1;
Servo servo2;

RTC_DS1307 rtc;

bool runAtDay = false;

Dusk2Dawn d2d_chch(LAT, LONG, 12);

void setup() {
  pinMode(PIR_1, INPUT);
  pinMode(PIR_2, INPUT);
  pinMode(ENABLE_6V, OUTPUT);

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

  
  // run_test();
  // set_time();   //TODO
  setup_rtc();

  if (!runAtDay) {
    initRTC();                      //? Is this redundant since because of setup_rtc()? 
  }

  // Booting LED sequence. If skip button is held set to run at the day also.
  for (int i = 0; i <= 20; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(500);
    digitalWrite(STATUS_LED, LOW);
    delay(500);
  }

  /* 
  * init servos
  * S1: 0 --> 60
  * S2: 45 --> 140
  */ 
  Serial.println("Resetting servos to 0");
  s1(0);
  s2(0);  

  // Setup Servo 2 (back door) to 45 degrees
  delay(5000);
  s2(45);

  delay(10000);

  // PIR 1 Wait for signal then trigger
  Serial.println("Waiting for PIR 1");
  waitFor(PIR_1, HIGH);
  Serial.println("Sweeping servo 1");
  s1(60);
  s1(0);

  // PIR 2 Wait for signal then trigger
  mySleep(10000);
  Serial.println("Waiting for PIR 2");
  waitFor(PIR_2, HIGH);
  Serial.println("Sweeping servo 2");
  s2(140);
}


void initRTC() {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC!!!");
    while (true) {
      //TODO LED flash sequence to indicate RTC not found.
    }
    else {

    }
  } 

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  else if (rtc.isrunning()) {
    Serial.println("RTC is already running");
  } 

  printDateTime(rtc.now());
}

void set_time() {
  Serial.println("Setting the RTC time to local compile time");
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
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
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
    Serial.println("Running in 24hr mode (Day + Night)");
    return;
  }
  while(true) {
    DateTime now = rtc.now();
    int minutesFromMidnight = now.hour()*60 + now.minute();
    int startMinute = d2d_chch.sunset(now.year(), now.month(), now.day(), false) + MINUTES_AFTER_SS; // TODO: daylight savings (bool)
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

void setup_rtc() {
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

    DateTime now = rtc.now();
    printDateTime(now);
  }
  printDateTime(rtc.now());

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
  digitalWrite(ENABLE_6V, HIGH);
  servo1.write(angle);
  delay(2000);
  digitalWrite(ENABLE_6V, LOW);
}

void s2(int angle) {
  digitalWrite(ENABLE_6V, HIGH);
  servo2.write(angle);
  delay(2000);
  digitalWrite(ENABLE_6V, LOW);
}

void loop() {
  mySleep(1000); 
}