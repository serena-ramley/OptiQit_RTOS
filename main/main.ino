#include "board.h"
#include "my_PCNT.h"
#include "my_LAC.h"
#include "my_BLE.h"
#include "tones.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define STATE_PWRON      0
#define STATE_WAITFORBLE 1
#define STATE_READY      2
#define STATE_RUN        3
#define STATE_DONE       4
#define STATE_PWROFF     5
#define STATE_ERROR      6

#define MEASUREMENT_IDLE       0
#define MEASUREMENT_STARTED    1
#define MEASUREMENT_DONE       2
#define MEASUREMENT_ERROR      3

#define MEASUREMENT_PERIOD_MS  5
#define MEASUREMENT_REPEATS  100
#define MEASUREMENT_POINTS     10
#define MAX_PCNT_VAL          20

#define BLE_CMD_NOTHING        0
#define BLE_CMD_START          1
#define BLE_CMD_RST            2
#define BLE_CMD_SHTDN          3

#define MELODY_NOT_STARTED     0
#define MELODY_TONE_1          1
#define MELODY_TONE_2          2
#define MELODY_TONE_3          3
#define MELODY_TONE_4          4
#define MELODY_TONE_5          5
#define MELODY_TONE_6          6          
#define MELODY_DONE            7

#define BLE_NOT_CONNECTED      0
#define BLE_NEEDS_TO_BEEP      1
#define BLE_BEEPED             2

// define tasks here
void Task0( void *pvParameters );
void Task1( void *pvParameters );
void Task2( void *pvParameters );
void Task3( void *pvParameters );
void Task4( void *pvParameters );
void Task5( void *pvParameters );
void Task6( void *pvParameters );

// global variables here (to be accessed by multiple tasks)
int kit_state           = STATE_PWROFF;
int measurement_status  = MEASUREMENT_IDLE;
int melody_status       = MELODY_NOT_STARTED;
int ble_status            = BLE_NOT_CONNECTED;
uint8_t pulseCountValue     = 0;
uint8_t pulseCountHistory[MEASUREMENT_POINTS];
int lacPosition         = 0;
LiquidCrystal_I2C lcd(0x27, 16, 2);   // LCD address, 16 Char, 2 lines
int usr_rst_btn_state   = HIGH;
int usr_start_btn_state = HIGH;
int usr_shtdn_btn_state = HIGH;
uint8_t ble_received_cmd = NULL;
bool measurement_transmitted = false;
bool melody_done = false;


// the setup function runs once when you press reset or power the board
void setup() {
  // Task0 Setup
  // Task1 Setup
  pinMode(PIN_LED_RED, OUTPUT);         // initialize digital PIN_LED_RED as an output.
  pinMode(PIN_LED_GREEN, OUTPUT);       // initialize digital PIN_LED_GREEN as an output.
  pinMode(PIN_LED_BLUE, OUTPUT);        // initialize digital PIN_LED_BLUE as an output.
  // Task2 Setup
  pinMode(PIN_BUZZER, OUTPUT);          // initialize digital PIN_BUZZER as an output.
  // Task3 Setup
  Wire.begin(PIN_LCD_SDA,PIN_LCD_SCL);
  // Task4 Setup
  pinMode(PIN_USR_RST, INPUT);          // initialize digital PIN_USR_RST as an input.
  pinMode(PIN_USR_START, INPUT);        // initialize digital PIN_USR_START as an input.
  pinMode(PIN_USR_SHTDN, INPUT);        // initialize digital PIN_USR_SHTDN as an input.
  // Task5 Setup
  pinMode(PIN_CNT_QA, INPUT);           // initialize digital PIN_CNT_QA as an input.
  pinMode(PIN_CNT_QB, INPUT);           // initialize digital PIN_CNT_QB as an input.
  pinMode(PIN_CNT_QC, INPUT);           // initialize digital PIN_CNT_QC as an input.
  pinMode(PIN_CNT_QD, INPUT);           // initialize digital PIN_CNT_QD as an input.
  pinMode(ESP_PCNT_CTRL, INPUT);        // initialize digital PIN_CNT_QC as an input.
  pinMode(PIN_CNT_CLR, OUTPUT);         // initialize digital PIN_CNT_CLR as an output.
  pinMode(PIN_LAC_CLK, OUTPUT);         // initialize digital PIN_LAC_CLK as an output.
  pinMode(PIN_LAC_CW, OUTPUT);          // initialize digital PIN_LAC_CW as an output.

  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  BLEInit();

  // Now set up your tasks to run independently.
  //  Task ID, Task Name, Stack Size, NULL, Priority (0: lowest, configMAX_PRIORITIES - 1: highest), NULL, Core 
  xTaskCreatePinnedToCore(Task0, "STATE MACHINE", 1024, NULL, 8, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task1, "LED",           1024, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task2, "BUZZER",        1024, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task3, "LCD DISPLAY",   8192, NULL, 4, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task4, "USER BUTTONS",  1024, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task5, "PCNT/LAC",      8192, NULL, 7, NULL, ARDUINO_RUNNING_CORE);  
  xTaskCreatePinnedToCore(Task6, "BLE",           2048, NULL, 6, NULL, ARDUINO_RUNNING_CORE);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void Task0(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task0 Description
    STATE MACHINE transitions
  */
  Serial.print("Task0 - STATE_MACHINE - Started\n");
  // Task0 Setup

  // Task0 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    if (kit_state == STATE_PWRON) {
      Serial.print("Task0 - STATE_MACHINE - STATE_PWRON\n");
      measurement_status = MEASUREMENT_IDLE;
      vTaskDelay(1000); 
      kit_state = STATE_WAITFORBLE;
      
    } else if (kit_state == STATE_WAITFORBLE) {
      Serial.print("Task0 - STATE_MACHINE - STATE_WAITFORBLE\n");
      measurement_status = MEASUREMENT_IDLE;
      ble_status = BLE_NOT_CONNECTED;
      if (deviceConnected) {
        ble_status = BLE_NEEDS_TO_BEEP;
        kit_state = STATE_READY;
      }  else if (usr_start_btn_state == LOW) {
        kit_state = STATE_READY;
      } else if (usr_shtdn_btn_state == LOW) {
        kit_state = STATE_PWROFF;
      }
    } else if (kit_state == STATE_READY) {
      Serial.print("Task0 - STATE_MACHINE - STATE_READY\n");   
      measurement_status = MEASUREMENT_IDLE;
      melody_status = MELODY_NOT_STARTED;   
      if ((usr_rst_btn_state == LOW) || (rxBleValue == BLE_CMD_RST)) {
        kit_state = STATE_PWRON;      // USR_RST_BTN takes the state machine to STATE_PWRON
      } else if ((usr_start_btn_state == LOW) || (rxBleValue == BLE_CMD_START)) {
        kit_state = STATE_RUN;
      } else if ((usr_shtdn_btn_state == LOW) || (rxBleValue == BLE_CMD_SHTDN)) {
        kit_state = STATE_PWROFF;
      }
      if (!deviceConnected) {
        kit_state = STATE_WAITFORBLE;
      } 
    } else if (kit_state == STATE_RUN) {
      Serial.print("Task0 - STATE_MACHINE - STATE_RUN\n");
      if (measurement_status == MEASUREMENT_STARTED) {
        vTaskDelay(MEASUREMENT_PERIOD_MS); 
      } else if (measurement_status == MEASUREMENT_ERROR) {
        kit_state = STATE_ERROR;
      } else if (measurement_status == MEASUREMENT_DONE) {
        kit_state = STATE_DONE;
      }
    } else if (kit_state == STATE_DONE) {
      Serial.print("Task0 - STATE_MACHINE - STATE_DONE\n");
      if ((usr_rst_btn_state == LOW) || (rxBleValue == BLE_CMD_RST)) {
        kit_state        = STATE_PWRON;      // USR_RST_BTN takes the state machine to STATE_PWRON
      } else if ((usr_start_btn_state == LOW) || (rxBleValue == BLE_CMD_START)) {
        kit_state = STATE_READY;
        vTaskDelay(1000); 
      } else if ((usr_shtdn_btn_state == LOW) || (rxBleValue == BLE_CMD_SHTDN)) {
        kit_state = STATE_PWROFF;
      }
      if (!deviceConnected) {
        kit_state = STATE_WAITFORBLE;
      }
    } else if (kit_state == STATE_PWROFF) {
      Serial.print("Task0 - STATE_MACHINE - STATE_PWROFF\n");
      if ((usr_rst_btn_state == LOW) || (rxBleValue == BLE_CMD_RST)) {
        kit_state        = STATE_PWRON;      // USR_RST_BTN takes the state machine to STATE_PWRON
      }
    } else if (kit_state == STATE_ERROR) {
      Serial.print("Task0 - STATE_MACHINE - STATE_ERROR\n");
      if ((usr_rst_btn_state == LOW) || (rxBleValue == BLE_CMD_RST)) {
        kit_state        = STATE_PWRON;      // USR_RST_BTN takes the state machine to STATE_PWRON
      }
    }
    rxBleValue = BLE_CMD_NOTHING;
    vTaskDelay(100);
  }
}

void Task1(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task1 Description
    Turns on/off the RGB LED.
  */
  Serial.print("Task1 - LED - Started\n");
  // Task1 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    digitalWrite(PIN_LED_RED, LOW);       // Set PIN_LED_RED voltage LOW
    digitalWrite(PIN_LED_GREEN, LOW);     // Set PIN_LED_GREEN voltage LOW
    digitalWrite(PIN_LED_BLUE, LOW);      // Set PIN_LED_BLUE voltage LOW
    if (kit_state == STATE_PWRON) {
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_BLUE, HIGH);
    } else if (kit_state == STATE_WAITFORBLE) {
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_BLUE, HIGH);
      vTaskDelay(5);
      digitalWrite(PIN_LED_BLUE, LOW);
    } else if (kit_state == STATE_READY) {
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_BLUE, LOW);
      vTaskDelay(1000);
      digitalWrite(PIN_LED_GREEN, HIGH);
    } else if (kit_state == STATE_RUN) {
      vTaskDelay(500);
      if (pulseCountValue == 0) {
        digitalWrite(PIN_LED_RED, HIGH);   // Turn on LED blue
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_BLUE, LOW);
        ledcWriteTone(0, NOTE_C3);
        vTaskDelay(500);
        ledcWriteTone(0, 0);
      } else if (pulseCountValue >= 5) {
        digitalWrite(PIN_LED_RED, LOW);   // Turn on LED green
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_BLUE, LOW);
        ledcWriteTone(0, NOTE_A5);
        vTaskDelay(500);
        ledcWriteTone(0, 0);
      } else {
        digitalWrite(PIN_LED_RED, LOW);  // Turn on LED red
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_BLUE, HIGH);
        ledcWriteTone(0, NOTE_G4);
        vTaskDelay(500);
        ledcWriteTone(0, 0);
      }
    } else if (kit_state == STATE_DONE) {
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_BLUE, LOW);
      
    } else if (kit_state == STATE_PWROFF) {
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_BLUE, LOW);
    } else if (kit_state == STATE_ERROR) {
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_BLUE, LOW);
      vTaskDelay(1000);
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_BLUE, LOW);
      vTaskDelay(995);
    }

    vTaskDelay(5);
  }
}


void Task2(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task2 Description
    Beeps BUZZER.
  */
  Serial.print("Task2 - BUZZER - Started\n");
  // Task2 Setup
  ledcSetup(0, 12000, 8);               // Channel 0, 12 kHz PWM, 8-bit resolution
  ledcAttachPin(PIN_BUZZER, 0);         // Attach PIN_BUZZER to PWM module

  // Task2 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    if (kit_state == STATE_PWRON) {
      ledcWriteTone(0, 523);
      vTaskDelay(250);
      ledcWriteTone(0, 0);
      vTaskDelay(2650);
    } else if (kit_state == STATE_WAITFORBLE) {
      ledcWriteTone(0, 0);
    } else if (kit_state == STATE_READY) {
//      ledcWriteTone(0, 523);
//      vTaskDelay(250);
//      ledcWriteTone(0, 0);
//      vTaskDelay(10000);
    } else if (kit_state == STATE_RUN) {
      // Runs inside LED code to be synchronous
    } else if (kit_state == STATE_DONE) {  
      if (melody_status != MELODY_DONE) {
        ledcWriteTone(0, melody[melody_status]);
        melody_status++;
        vTaskDelay(250);
        ledcWriteTone(0, 0);
        vTaskDelay(250);
      }
      else {
        ledcWriteTone(0, 0);
      }
    } else if (kit_state == STATE_PWROFF) {
      ledcWriteTone(0, 0);  
    } else if (kit_state == STATE_ERROR) {
      ledcWriteTone(0, NOTE_G1);
    }
    vTaskDelay(100);
  }
}


void Task3(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task3 Description
    Display messages on LCD.
  */
  Serial.print("Task3 - LCD Display - Started\n");
  // Task3 Setup
  lcd.init(PIN_LCD_SDA, PIN_LCD_SCL);   // initialize LCD
  lcd.backlight();                      // turn on LCD backlight

  // Task3 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    if (kit_state == STATE_PWRON) {      // if state is READY
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(2, 0);
      lcd.print("OptiQit 2020");
      lcd.setCursor(0, 1);
      lcd.print("Initializing...");
    } else if (kit_state == STATE_WAITFORBLE) {
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(2, 0);
      lcd.print("OptiQit 2020");
      lcd.setCursor(0, 1);
      lcd.print("Waiting for BLE");
    } else if (kit_state == STATE_READY) {      // if state is READY
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(3, 0);
      lcd.print("KIT READY");
      lcd.setCursor(2, 1);
      lcd.print("Press START");
    } else if (kit_state == STATE_RUN) {      // if state is Measurment RUNNING
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(0, 0);
      lcd.print("RUNNING ...");
      for (int point=0; point<=lacPosition; point++) {
        lcd.setCursor(0+point, 1);
        lcd.print("->");
      }
      lcd.setCursor(10, 1);
      lcd.print(pulseCountValue);
    } else if (kit_state == STATE_DONE) {      // if state is Measurment DONE
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(3, 0);
      lcd.print("MEAS DONE !");
      //lcd.setCursor(0, 1);
      //lcd.print("PCNT VAL:");
      //lcd.setCursor(10, 1);
      //lcd.print(pulseCountValue);
    } else if (kit_state == STATE_PWROFF) {    // PWROFF State
      lcd.clear();
      lcd.noBacklight();
    } else if (kit_state == STATE_ERROR) {
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(4, 0);
      lcd.print("ERROR !");
      lcd.setCursor(0, 2);
      lcd.print("Press RESET:");
    }
    vTaskDelay(100);                    // one tick delay (15ms) in between reads for stability
  }
}


void Task4(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task4 Description
    USER BUTTONS.
  */
  Serial.print("Task4 - USER BUTTONS - Started\n");
  // Task4 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    /* get button state + debounce */
    int usr_rst_reading = digitalRead(PIN_USR_RST);
    int usr_start_reading = digitalRead(PIN_USR_START);
    int usr_shtdn_reading = digitalRead(PIN_USR_SHTDN);
    delay(50);                    // delay 50ms
    if (digitalRead(PIN_USR_RST) == usr_rst_reading) {
      usr_rst_btn_state = usr_rst_reading;
    }
    if (digitalRead(PIN_USR_START) == usr_start_reading) {
      usr_start_btn_state = usr_start_reading;
    }
    if (digitalRead(PIN_USR_SHTDN) == usr_shtdn_reading) {
      usr_shtdn_btn_state = usr_shtdn_reading;
    }
    
    vTaskDelay(100);
  }
}


void Task5(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task5 Description
    PCNT/LAC Control.
  */
  Serial.print("Task5 - PCNT/LAC CONTROL - Started\n");

  // Task5 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    if (kit_state == STATE_PWRON) {
      digitalWrite(PIN_CNT_CLR, LOW);         // Don't Clear external PCNTer
      LAC_init(PIN_LAC_CLK, PIN_LAC_CW);      // Initialize the Linear Actuator
    } else if (kit_state == STATE_WAITFORBLE) {
      digitalWrite(PIN_CNT_CLR, LOW);         // Don't Clear external PCNTer
    } else if (kit_state == STATE_READY) {
      digitalWrite(PIN_CNT_CLR, LOW);         // Don't Clear external PCNTer
      pulseCountValue = 0;                    // Reset counter
      for (int lacPosition = 0; lacPosition < MEASUREMENT_POINTS; lacPosition++) {
        pulseCountHistory[lacPosition] = 0;
      }
      LAC_init(PIN_LAC_CLK, PIN_LAC_CW);      // Initialize the Linear Actuator
      lacPosition = 0;
    } else if (kit_state == STATE_RUN) {
      // Start a new measurement
      for (lacPosition = 0; lacPosition < MEASUREMENT_POINTS; lacPosition++)
      {
        digitalWrite(PIN_LAC_CW, LOW);          // Reset the Linear Actuator's direction
        LAC_move(PIN_LAC_CLK);                  // Move the Linear Actuator
        vTaskDelay(1000);                       // Wait till the actuator moves
        Serial.printf("\tTask4 - PCNT - Actuator at position: %d\n", lacPosition);
        
        int pulseCountValueTmp = 0;             // Clear previous measurement
        for (int repeats = 0; repeats < MEASUREMENT_REPEATS; repeats++)
        {
          int16_t count = 0;                    // 16-bit count register
                                                // Clear external PCNTer
          digitalWrite(PIN_CNT_CLR, HIGH);        
          vTaskDelay(5);
          digitalWrite(PIN_CNT_CLR, LOW);  
                                                // Initialize and start ESP32 PCNTer
          if (pcnt_init_and_start(PIN_CNT_QD, ESP_PCNT_CTRL) != ESP_OK) {
            Serial.printf("\tTask4 - PCNT - FAILED: pcnt_init_and_start\n");
            measurement_status = MEASUREMENT_ERROR;
          } else {
            Serial.printf("\tTask4 - PCNT - ESP32_PCNT started\n");
            measurement_status = MEASUREMENT_STARTED;
          }
        
          delay(MEASUREMENT_PERIOD_MS);         // Wait for MEASUREMENT_PERIOD_MS
          pcnt_get(&count);                     // Read ESP32 PCNT Value
                                                // Calculate total number of pulses (ESP32 PCNT + External PCNT)
          pulseCountValueTmp += count*16 + 4*digitalRead(PIN_CNT_QC) + 2*digitalRead(PIN_CNT_QB) + digitalRead(PIN_CNT_QA);
          pcnt_clear();                         // Clear ESP32 Pulse Counter for next acquisition
        }
                                                // Average results
        pulseCountValue = pulseCountValueTmp / MEASUREMENT_REPEATS;
                                                // Save all measurement History
        pulseCountHistory[lacPosition] = pulseCountValue;
      }
      measurement_status = MEASUREMENT_DONE;
    } else if (kit_state == STATE_DONE) {
      digitalWrite(PIN_CNT_CLR, LOW);
    } else if (kit_state == STATE_PWROFF) {
      digitalWrite(PIN_CNT_CLR, LOW);
    }
    vTaskDelay(100);
  }
}


void Task6(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  /*
    Task6 Description
    Turns on an LED on for one second, then off for one second, repeatedly.
  */
  Serial.print("Task6 - BLE CONTROL - Started\n");

  // Task6 Setup
  
  // Task6 Loop
  for (;;)                              // A Task shall never return or exit.
  {
    if (kit_state == STATE_PWRON) {      // if state is READY
      measurement_transmitted = false;
    } else if (kit_state == STATE_WAITFORBLE) {
      measurement_transmitted = false;
    } else if (kit_state == STATE_READY) {      // if state is READY
      measurement_transmitted = false;
    } else if (kit_state == STATE_RUN) {        // if state is Measurment RUNNING 
      measurement_transmitted = false;
    } else if (kit_state == STATE_DONE) {       // if state is Measurment DONE
      if (measurement_transmitted == false) {  
        vTaskDelay(100);
        if (deviceConnected) {
          Serial.print("  Task6 - BLE - Sending Results ...\n");
          for (int lacPosition = 0; lacPosition < MEASUREMENT_POINTS; lacPosition++) {
            txValue = (uint8_t)pulseCountHistory[lacPosition];
            pTxCharacteristic->setValue(&txValue, 1);
            pTxCharacteristic->notify();
            delay(10);                         // bluetooth stack will go into congestion, if too many packets are sent
          }
          measurement_transmitted = true;    // data sent
        } else {
          vTaskDelay(500); // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          Serial.println("  Task6 - BLE - device disconnected -> start advertising");
        }
      }
    } else if (kit_state == STATE_PWROFF) {     // PWROFF State
      measurement_transmitted = false;
    } else if (kit_state == STATE_ERROR) {
    }
    vTaskDelay(500);                    // one tick delay (15ms) in between reads for stability
  }
}
