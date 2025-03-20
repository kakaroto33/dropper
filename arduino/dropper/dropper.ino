/**************************************************************************
 Digital Dropper Monitor
 --------------------------------------------------------------------------
 Author: Haroldo Mitsumi Murata <kakaroto33@gmail.com>
 Date:   2025-01-24

 This firmware to ESP32 controller, to monitor drops counter using IR Led.
 
 Power:   18650 3.7V 30000mAh Li-ion Battery + modules (TP4056, MT3608)
 CPU:     ESP32-WROOM-32-Dev-Board 38PINS (ESP32-D0WDQ6 + 160Mhz + 520KB SRAM + ~1MB FLASH)
 IR:      {IN} Photodiode IR [IO39]
 VBAT:    {IN} Battery Monitor (1/2 V.Divisor) [IO34]
 VUSBC:   {IN} USB-C Charge Monitor (1/3 V.Divisor) [IO35]
 TEMP:    {IN} Temperature Monitor (TMP36GT9) [IO32]
 DISPLAY: {I2C} OLED Display 128x64 (SSD1306) [IO22:SCL][IO21:SDA] {This is default I2C used on Wire lib for ESP32}
 RGB_LED: {OUT} Common RGB LED VCC common [IO19:RED][IO18:GREEN][IO5:BLUE]
 BUZZER:  {OUT} Common buzzer output [IO25]
 ENCODER: {IN} Rotatory Encoder TWO03 [IO26:CLK][IO27:DT][IO14:SW]

 **************************************************************************/

#include <Arduino.h>
#include <math.h>
#include <RotaryEncoder.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Preferences.h>

#include "esp_sleep.h"

//== SCREEN DFINITIONS ========================================================

#define SSD1306_NO_SPLASH
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define MAX_WIDTH 127    // SCREEN_WIDTH  - 1 : Last pixel
#define MAX_HEIGHT 63    // SCREEN_HEIGHT - 1 : Last pixel
#include <Adafruit_SSD1306.h>

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL)
// On an arduino ESP32:     21(SDA), 22(SCL)
#define OLED_RESET     -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Timeout not interacting with setup
#define SCREEN_SETUP_TIMEOUT     10  // In seconds
#define SCREEN_REFERENCE_TIMEOUT 3   // In seconds
#define SCREEN_ALERT_TIMEOUT     3   // In seconds

//== ROTATORY ENCODER =========================================================

#define ROTATORY_IN1 26
#define ROTATORY_IN2 27
#define ROTATORY_SWT 14

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(ROTATORY_IN1, ROTATORY_IN2, RotaryEncoder::LatchMode::FOUR3);

//== BUZZER CONFIG ============================================================

#define BUZZER 25

//== BATTERY CONFIG ===========================================================

#define CHARGE_MON  32
#define VBAR_MON    33

//== PERSISTENT CONFIG ========================================================

Preferences preferences;

//== SENSORS ==================================================================

#define IR_SENSOR 34

//== OTHER SYSTEM DEFINITIONS =================================================

#define MAX_DROPS_SECOND 99    // Fix limit drops per seconds

//== GLOBAL VARIABLES =========================================================

// System State, there reset at setup()
char display_mode = 'D';  // D = Default, S = Setings
int setup_timout  = 0;    // Timout seup screen
int refer_timeout = 0;
int alert_buzzer  = 1;    // Aways startup ON (1)
bool save_changes = false;
float ml_per_drop = 0.05;   // 0.05ml per drop or 1ml has 20 drops
char system_state = 'N';    // N: Normal, L: Lento, P: Parado, R: Muito Rapido
bool refer_adjust = false;  // Ajuste Referencia

// Display Values
int battery_status         = 0;
bool battery_charging      = false;
int drops_second           = 0;
float milliliters_hour     = 0;
float total_volume         = 0;
float set_drops_sec        = 0;
unsigned long header_timer = 0;

// Battery reading
int vbat_leng   = 6;
int vbat_list[10] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
// Iem Position 0 ~ 5
int vbat_pos  = 0;
int vbat_temp = 0;
int vbat_now  = 0;
int vbat_ref  = 0; // Setup referency integer

// Rotatory Encoder
int encoder_pos   = 0;
int encoder_click = 0;
int encoder_dir   = 0;

// Update Timers
unsigned long timer_fast = 0;   // Update every 0.1 second (10fps)
int timer_medium    = 0;   // Update every 1 second
int timer_long      = 0;   // Update every 10 seconds
int timer_ir        = 0;   // IR timer update 1sec
int timer_tg_head   = 0;
int timer_tg_value  = 0;
int timer_sys_swtch = 0;
int timer_buzzer    = 0;
bool refresh_fast   = false;
bool refresh_med    = false;
bool refresh_long   = false;
bool refresh_disp   = false; 
bool refresh_header = false; 
bool refresh_value  = false; 
bool refresh_sys_sw = false; 
bool values_toggle  = false;
bool buzzer_toggle  = false;

// Menu
int menu_pos     = 0;
bool menu_change = false;
int menu_select  = 0;

// Temp variables
unsigned long tmp_millis  = 0;
int tmp_stamp  = 0;
int tmp_stamp2 = 0;
int tmp_pos    = 0;
char tmp_char10[10];
int tmp_int    = 0;
bool debug_plot = false;

// Multicore Tasks 
TaskHandle_t Task1;
TaskHandle_t Task2;

//=============================================================================
//== MAIN LOOP FUNCTIONS ======================================================
//=============================================================================

/**
 * Deafult Setup
 */
void setup() {
  Serial.begin(115200); // Default speed for ESP32
  //Serial.begin(19200); // Due low clock, console maybe is half 9600
  //
  uint32_t Freq = 0;
  setCpuFrequencyMhz(80);   //  240, 160, ==> 80 (OK), -->40 (misses adc read)
  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
  display.display();
  delay(500);

  // Limit Lines
  // display.drawLine(0, 0, display.width()-1, 0, SSD1306_WHITE);
  // display.drawLine(0, 15, display.width()-1, 15, SSD1306_WHITE);
  // display.drawLine(0, 16, display.width()-20, 16, SSD1306_WHITE);
  // display.drawLine(0, display.height() - 1, display.width()-20, display.height() - 1, SSD1306_WHITE);
  resetData();
  prepareDefaultScreen();
  display.display();

  // Inputs
  pinMode(ROTATORY_SWT  , INPUT_PULLUP);
  pinMode(IR_SENSOR     , INPUT_PULLDOWN);
  pinMode(CHARGE_MON    , INPUT_PULLDOWN);
  pinMode(VBAR_MON      , INPUT_PULLDOWN);

  // Output
  pinMode(BUZZER, OUTPUT);
  
  //
  Serial.println(F("# Setup Finished"));
  Serial.print("# Setup running on core: ");
  Serial.println(xPortGetCoreID());

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500); 

}

/**
 * Default Loop
 * @alert Keep loop clean, since we using multi-core tasks
 */
void loop() {}

bool refresh_ir   = false;
bool refresh_drop = false;
unsigned long tmp_millis2  = 0;
int update_drop  = 0;
unsigned long noise_lock = 0;

int loop_counter = 0;
int noise_range  = 150;
int ir_millis_v  = 0;
int ir_line      = 0;
int diff_noise   = 0;
int drop_count   = 0;
int still_high   = 0;
int noise_time   = 25;

// Init Drops List
int drops_leng     = 10;
int drops_list[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Iem Position 0 ~ 5
int drops_pos  = 0;
int drops_temp = 0;
int drops_ref  = 0; // Setup referency integer

/**
 * Core 0 Loop
 *
 * Background monitoring
 */
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  // Main Task Loop HERE
  for (;;) {

    tmp_millis2  = millis();
    tmp_stamp2   = round(tmp_millis2 / 1000);
    refresh_ir   = false;
    refresh_drop = false;
    if (timer_ir < tmp_millis2) {
        timer_ir  = tmp_millis2 + 100;
        refresh_ir = true;
    }
    if (update_drop < tmp_stamp2) {
      update_drop  = tmp_stamp2;
      refresh_drop = true;
    }
    // System swtich delay
    refresh_sys_sw = false;
    if (timer_sys_swtch < tmp_stamp2) {
      refresh_sys_sw = true;
    }
    // We don't need check voltage, only changes spikes
    ir_millis_v  = analogReadMilliVolts(IR_SENSOR);
    loop_counter++;

    // Medium line it take sample each 0.1 ms
    if (refresh_ir) {
      ir_line = (int) (ir_millis_v + (ir_line * 3)) / 4; // More slow medium change
    }

    // Only medium over time
    //ir_line    = (int) (ir_millis_v + (ir_line * 4)) / 5; // More slow medium change
    diff_noise = abs((int)(ir_millis_v - ir_line));

    if (abs(diff_noise) > noise_range) {
      if (still_high == 0) {
        drop_count++;
        // Lock time noise in milli seconds
        noise_lock = tmp_millis2 + noise_time;
      }
      still_high++;
    } else {
      if (tmp_millis2 > noise_lock) {
        still_high = 0;
      }
    }

    if (debug_plot) {
      Serial.print(ir_millis_v);
      Serial.print(' ');
      Serial.print(ir_line);
      Serial.print(' ');
      Serial.print(diff_noise);
      Serial.print(' ');
      if (still_high > 0) {
        Serial.println(200);
      } else {
        Serial.println(100);
      }
    }    
   
    //Update on 1fps
    if (refresh_drop) {
        // Register on rotative list (6 indexes)
        drops_list[drops_pos] = drop_count * 10;
        drops_pos++;
        if (drops_pos >= drops_leng) {
          drops_pos = 0;
        }
        // Get medium
        drops_temp = 0;
        for (tmp_int = 0; tmp_int < drops_leng; tmp_int++) {
          drops_temp += drops_list[tmp_int];
        }
        drops_second = round(drops_temp / drops_leng);
        drops_ref    = (int) (set_drops_sec * 10);

        //drops_second = round((drops_second + (drop_count * 30)) / 4); // 3 x More power over new counter
        // Only switch with delay
        if (refresh_sys_sw) {
          // N: Normal, L: Lento, P: Parado, R: Muito Rapido
          if (drops_second > MAX_DROPS_SECOND) {
            drops_second = MAX_DROPS_SECOND; // limit screen
            system_state = 'R'; 
          } else if (drops_second <= 0) {
            system_state = 'P'; 
          } else if (drops_second < drops_ref) {
            system_state = 'L'; 
          } else if (system_state != 'N') {
            system_state = 'N'; 
          }
          timer_sys_swtch = tmp_stamp2 + SCREEN_ALERT_TIMEOUT;
        } else {
          // Only lmiter
          if (drops_second > MAX_DROPS_SECOND) {
            drops_second = MAX_DROPS_SECOND; // limit screen
          }
        }
        // Add totals
        total_volume += drop_count * ml_per_drop;
        // ml/h
        milliliters_hour = (drops_second * 3600 * ml_per_drop) / 10; // (Drops/s * ml_per_drop * 60 min * 60 sec) / 10 (original count is *10)

        // Serial.print("[LOOP:");
        // Serial.print(loop_counter);
        // Serial.print("][DROP: ");
        // Serial.print(drop_count);
        // Serial.print("][TOTAL:");
        // Serial.print(drops_temp);
        // Serial.print("][ML/H:");
        // Serial.print(milliliters_hour);
        // Serial.print("] DROP/S: ");
        // Serial.println(drops_second);

        loop_counter = 0;
        drop_count   = 0;
    } 
    
    // Add a small delay to let the watchdog process
    //https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    delay(5);
  }

}

/**
 * Core 1 Loop
 */
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  // Main Task Loop HERE
  for (;;) {
    // Timestamp in seconds
    tmp_millis   = millis();
    tmp_stamp    = round(tmp_millis / 1000);
    refresh_fast = false;
    refresh_med  = false;
    refresh_long = false;
    refresh_disp = false;

    // Update on 0.1fps
    if (timer_fast < tmp_millis || timer_fast == 0) {
        timer_fast  = tmp_millis + 100;
        refresh_fast = true;
    } 

    // Update on 1fps
    if (timer_medium != tmp_stamp || timer_medium == 0) {
        timer_medium = tmp_stamp;
        refresh_med   = true;
    } 
    // Update ~10 fps
    if (timer_long < tmp_stamp || timer_long == 0) {
      timer_long  = tmp_stamp + 10;
      refresh_long = true;
    }

    // Detect encode change
    encoder.tick();
    tmp_pos = encoder.getPosition();
    //int menu_pos     = 0;
    if (encoder_pos != tmp_pos) {
      encoder_pos = tmp_pos;
      encoder_dir = (int)(encoder.getDirection());
      menu_change = true;
      if (encoder_dir == 1) { //-1 = RotaryEncoder::Direction::COUNTERCLOCKWISE, 1 = RotaryEncoder::Direction::CLOCKWISE
        menu_pos++;
      } else {
        menu_pos--;
      }
      // Limiter
      if (menu_pos < 0) {
        menu_pos  = 0;
      } else if (menu_pos > 3) {
        menu_pos  = 3;
      }

      // Change Referencia
      if (display_mode == 'D') {
        if (encoder_dir == 1) { //-1 = RotaryEncoder::Direction::COUNTERCLOCKWISE, 1 = RotaryEncoder::Direction::CLOCKWISE
          set_drops_sec = set_drops_sec + 0.1;
        } else {
          set_drops_sec = set_drops_sec - 0.1;
        }
        if (set_drops_sec >= 10 ) {
          set_drops_sec = 9.9;
        } else if (set_drops_sec < 0) {
          set_drops_sec = 0;
        }
        save_changes  = true;
        refer_timeout = tmp_stamp + SCREEN_REFERENCE_TIMEOUT;
        refer_adjust  = true;
      }

      if (display_mode == 'S') {
        setup_timout = tmp_stamp + SCREEN_SETUP_TIMEOUT;
      }

      // Serial.print("pos:");
      // Serial.print(tmp_pos);
      // Serial.print(", dir:");
      // Serial.print(encoder_dir);
      // Serial.print(", menu:");
      // Serial.println(menu_pos);
    }

    // Read the state of the pushbutton value:
    if (digitalRead(ROTATORY_SWT) == LOW) {
      // If is buzzer beeping 
      if (alert_buzzer == 1 && display_mode == 'D' && encoder_click == 0) {
        if (system_state == 'L' || system_state == 'P' || system_state == 'R') {
          // just buzzer off, instead enter menu
          alert_buzzer = 0;
          // skip slick
          encoder_click = 1;
        }
      }
      // Normal click
      if (encoder_click == 0) {
        Serial.println("[ROTATORY_SWT:LOW]");
        if (display_mode == 'D') {
          display_mode = 'S';
          prepareSetupScreen();
          refresh_disp  = true;
          refresh_med   = true;
          refresh_long  = true;

        } else if (display_mode == 'S') {
          if (menu_pos == 0) { // Bipi
            if (alert_buzzer == 0) {
              alert_buzzer = 1;
            } else {
              alert_buzzer = 0;
            }
            save_changes = true;
            updateSpeaker();
          } else if (menu_pos == 1) { // Reset
            resetData();
            display_mode = 'D';
            prepareDefaultScreen();
            refresh_disp = true;
            refresh_med  = true;
            refresh_long = true;
          } else {  // Exit
            display_mode = 'D';
            prepareDefaultScreen();
            refresh_disp = true;
            refresh_med  = true;
            refresh_long = true;
          }
        }
        setup_timout = tmp_stamp + SCREEN_SETUP_TIMEOUT;
      }
      encoder_click++;
    } else {
      encoder_click = 0;
    }

    // Default Display Mode 
    if (display_mode == 'D') {
      // Update on 10fps
      if (refresh_fast) {
        updateValues();
        refresh_disp = true;
      } 
      // Check display  setup timeout
      if (tmp_stamp > refer_timeout) {
        refer_adjust = false;
      }
    } 
    // Setup Display Mode
    else {
      // SCREEN_SETUP_TIMEOUT
      // Update on 0.1fps
      if (refresh_fast) {
        refresh_disp = true;
        updateSetup();
      } 

      // Check display  setup timeout
      if (tmp_stamp > setup_timout) {
        display_mode = 'D';
        prepareDefaultScreen();
        refresh_med  = true;
        refresh_long = true;
        refresh_disp = true;
      }
    }

    // Header bar always display
    // Update on 1fps
    if (refresh_med) {
      // updateTime();
      updateHeader();
      refresh_disp = true;
    } 
    // Update ~10 fps
    if (refresh_long) {
      // updateBattery();
      // updateSpeaker();
      refresh_disp = true;
      if (save_changes) {
        save_changes = false;
        savePreferences();
      }
    }

    if (refresh_disp) {
      display.display();
    }

    // Buzzer
    if (alert_buzzer == 1) {
      // N: Normal, L: Lento, P: Parado, R: Muito Rapido
      if (system_state == 'L' || system_state == 'P' || system_state == 'R') {
        if (timer_buzzer < tmp_millis || timer_fast == 0) {
          if (system_state == 'L') {
            if (!buzzer_toggle) {
              timer_buzzer  = tmp_millis + 200;
            } else {
              timer_buzzer  = tmp_millis + 2000;
            }

          } else { // P: Parado, R: Muito Rapido
            timer_buzzer  = tmp_millis + 3000;
          }
          buzzer_toggle = !buzzer_toggle;
        }
        if (buzzer_toggle) {
          digitalWrite(BUZZER, HIGH);
        } else {
          digitalWrite(BUZZER, LOW);
        }
      } else {
        if (buzzer_toggle) {
          buzzer_toggle = false;
          digitalWrite(BUZZER, LOW);
        }
      } 
    } else {
      if (buzzer_toggle) {
        buzzer_toggle = false;
        digitalWrite(BUZZER, LOW);
      }
    }

    // Battery
    // Charging
    if (refresh_med) {
      tmp_int = analogRead(CHARGE_MON);
      if (tmp_int > 500) { // Normally read 1600
        battery_charging = true;
      } else {
        battery_charging = false;
      }
      
    }
    // Voltage Status
    if (refresh_med) {
      vbat_now = analogRead(VBAR_MON);
      // Register on rotative list (6 indexes)
      vbat_list[vbat_pos] = vbat_now;
      vbat_pos++;
      if (vbat_pos >= vbat_leng) {
        vbat_pos = 0;
      }
      // Get medium
      vbat_temp = 0;
      for (tmp_int = 0; tmp_int < vbat_leng; tmp_int++) {
        vbat_temp += vbat_list[tmp_int];
      }
      vbat_ref    = (int) round(vbat_temp / vbat_leng);
      // Serial.print(F("# Battery Vref: "));
      // Serial.println(vbat_ref);
      // 4.1V ~= 100% ~= 2290 vref
      // 3.1V  = 1%   ~= 1690 vref
      // Error Low Bat 
      // 3.0   =      ~= 1600 vref // Keep battery safe, enter in deep slepp
      //
      // 4.1/3.1 = 2290/1690 = 100/1
      // (2290 - 1690) = 600
      // (2290 - 1690)/600 = 1 (100%)
      // ((X - 1690)/600) * 100  = y%
      // (X - 1690)/6  = y%
      tmp_int = (int) round((vbat_ref - 1690) / 6);
      if (tmp_int < 0) {
        tmp_int = 0; // keep positive
      } else if (tmp_int > 100) {
        tmp_int = 100; // keep positive
      }
      battery_status = tmp_int;

      // Spceial State
      // - If has battery: vbat_now > 500
      // - If charge is : 0%
      // - Show message battery low for 10 seconds
      // - Enter in deep sleep
      // Obs: For deve and test, must poweroff switch if battery is low
      //if (tmp_int == 0 && vbat_now > 500) {
      if (tmp_int == 0 && vbat_now > 500) { 
          digitalWrite(BUZZER, LOW);
          showLowBatteryAlert();
          delay(10000); 
          display.clearDisplay();
          display.display();
          delay(100); 
          // Enter deep sleep
          powerOffSystem();
      }

      // Serial.print(F("# Battery %: "));
      // Serial.println(battery_status);
    }    
    // Add a small delay to let the watchdog process
    //https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    delay(5); // Pause for 0.05 second
  }
}

//=============================================================================
//== FUNCTIONS ================================================================
//=============================================================================

/**
 * Do deep sleep to power off
 */
void powerOffSystem()
{
  // Disable IOs
  pinMode(BUZZER, INPUT_PULLDOWN);
  pinMode(ROTATORY_SWT, INPUT_PULLDOWN);
  // https://forum.arduino.cc/t/hibernate-on-arduino-nano-esp32/1214370
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_ULP, ESP_PD_OPTION_OFF);
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_ULP, ESP_PD_OPTION_OFF);
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

/**
 * Load persistent preferences
 */
void loadPreferences()
{
  // Persistent Storage
  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("dropper", true);

  // Get the counter value, if the key does not exist, return a default value of 0
  // Note: Key name is limited to 15 chars.
  //alert_buzzer   = preferences.getInt("alert_buzzer", 0); // Don't need
  set_drops_sec  = preferences.getFloat("set_drops_sec", 0);
  
  // Close the Preferences
  preferences.end();
}

/**
 * Load persistent preferences
 */
void savePreferences()
{
  // Persistent Storage
  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("dropper", false);

  // Store the counter to the Preferences
  //preferences.putInt("alert_buzzer", alert_buzzer); // Don't need
  preferences.putFloat("set_drops_sec", set_drops_sec);

  // Close the Preferences
  preferences.end();
}

/**
 * Reset Data 
 */
void resetData() {
  battery_status   = 0;
  battery_charging = false;
  drops_second     = 0;
  milliliters_hour = 0;
  total_volume     = 0;
  header_timer     = millis();
  alert_buzzer     = 1; // Default ON (1)
  loadPreferences();
}

/**
 * Low battery Alert
 */
void showLowBatteryAlert() 
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(26, 0);
  display.print(F("BATERIA")); 
  display.setCursor(38, 17);
  display.print(F("MUITO")); 
  display.setCursor(38, 33);
  display.print(F("BAIXA")); 
  display.setCursor(2, 49);
  display.print(F("DESLIGANDO")); 
  display.display();
}

/**
 * Prepare display yellow header
 */
void prepareHeader()
{
  // Battery Body Outer Line
  display.drawLine(108,  4, 108,  9, SSD1306_WHITE);
  display.drawLine(109,  4, 109,  9, SSD1306_WHITE);
  display.drawLine(110,  2, 110, 11, SSD1306_WHITE);
  display.drawLine(123,  2, 123, 11, SSD1306_WHITE);
  display.drawLine(110,  2, 123,  2, SSD1306_WHITE);
  display.drawLine(110, 11, 123, 11, SSD1306_WHITE);

  // Separator
  display.drawLine(0 , 15, MAX_WIDTH,  15, SSD1306_WHITE);        // Header Line
}


/**
 * Render full header
 */
void updateHeader()
{
  // Ivert to Alert
  // Toggle on 0.5fps
  if (timer_tg_head <  millis()) {
      timer_tg_head =  millis() + 500;
      refresh_header = !refresh_header;
  } 
  
  if (!refresh_header) {
    display.fillRect(0, 0, SCREEN_WIDTH, 16, SSD1306_BLACK);
  }

  prepareHeader();
  updateTime();
  updateBattery();
  updateSpeaker();

  if (refresh_header) {
    if (system_state == 'L' || system_state == 'P' || system_state == 'R') {
      display.fillRect(0, 0, SCREEN_WIDTH, 16, SSD1306_INVERSE);
    }
  }
}

/**
 * Prepare display with fixed drawn parts and labels
 */
void prepareDefaultScreen()
{
  // Clear display buffer
  display.clearDisplay(); 
  //prepareHeader();
}

/**
 * Prepare setup screen
 */
void prepareSetupScreen()
{
  // Clear display buffer
  display.clearDisplay();
  //prepareHeader();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(7, 18);
  //               012345678901234567890
  display.print(F("Aviso Bipi:")); 

  display.setCursor(7, 30);
  display.print(F("Resetar")); 

  display.setCursor(7, 42);
  display.print(F("Sair")); 

  // display.setCursor(7, 54);
  // display.print(F("Unused")); 
}

/**
 * Update Setup Screen
 */
void updateSetup()
{
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  // Aviso Bipi:
  // Clear
  display.fillRect(74, 18, 24, 8, SSD1306_BLACK);
  // Text
  display.setCursor(78, 18);
  if (alert_buzzer == 0) {
    display.print(F("Nao")); 
  } else {
    display.print(F("Sim")); 
  }
  
  // Reset position
  if (menu_change) {
    display.fillRect(0, 18, 4, 62, SSD1306_BLACK);
    menu_change = false;
  }

  // Draw menu rectangle pos
  if (menu_pos == 0) {
    // Alerta Bipi
    display.fillRect(0, 18, 4, 8, SSD1306_WHITE);
  } else if (menu_pos == 1) {
    // Reset
    display.fillRect(0, 30, 4, 8, SSD1306_WHITE);
  } else {
    // Exit
    display.fillRect(0, 42, 4, 8, SSD1306_WHITE);
  } 
  // Unused
  // display.fillRect(0, 54, 4, 8, SSD1306_WHITE);
  

}

/**
 * Update Text Values
 */
void updateValues()
{
     
  // Big Clear Rectangle
  display.fillRect(0, 17, MAX_WIDTH, MAX_HEIGHT - 17, SSD1306_BLACK);

  // Separator
  display.drawLine(75, 18, 75, MAX_HEIGHT - 2, SSD1306_WHITE);    // Column Line

  // Body Drops/sec
  // Text  
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);   
  display.setCursor(80, 22); 
  sprintf(tmp_char10,"%02d", drops_second);
  display.print(F(tmp_char10)); 
  // Custom Dot
  display.fillRect(100, 48, 4, 4, SSD1306_WHITE);
  // Label
  display.setTextSize(1);
  display.setCursor(82, 53); 
  display.print(F("gotas/s"));


  if (timer_tg_value <  millis()) {
      timer_tg_value =  millis() + 500;
      refresh_value = !refresh_value;
  } 
  
  values_toggle = true;
   
  if (refer_adjust) {
    // In adjust reference fixed 
    values_toggle = false;
  } else if (system_state == 'L' || system_state == 'P' || system_state == 'R') {  // N: Normal, L: Lento, P: Parado, R: Muito Rapido
    values_toggle = refresh_value;
  } 
  
  if (!values_toggle) {
    display.setTextSize(2);
    if (refer_adjust) {
      display.setCursor(4, 37);
      sprintf(tmp_char10,"%.1f", set_drops_sec);
      display.print(F(tmp_char10)); 

      display.setTextSize(1);
      display.setCursor(44, 43);
      display.print(F("gt/s")); 

      display.setCursor(4, 25);
      display.print(F("Referencia:")); 
    } else if (system_state == 'L') {
      display.setCursor(5, 30);
      display.print(F("LENTO")); 
    } else if (system_state == 'P') {
      display.setCursor(2, 30);
      display.print(F("PARADO")); 
    } else if (system_state == 'R') {
      display.setCursor(2, 22);
      display.print(F("MUITO")); 
      display.setCursor(2, 42);
      display.print(F("RAPIDO")); 
    } 
    if (!refer_adjust) {
      display.fillRect(0, 17, 74, 45, SSD1306_INVERSE);
    }
  } else {
    // Velocidade Milliliters per hour
    // Label 
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print(F("Vel: ")); 
    // Text
    display.setTextSize(1);
    display.setCursor(24, 20);
    sprintf(tmp_char10,"%.0f", milliliters_hour);
    display.print(F(tmp_char10)); 
    display.print(F(" ml/h")); 

    // Total Milliliters
    // Label
    display.setCursor(0, 30);
    display.print(F("Tot: "));  
    // Text
    display.setCursor(24, 30);
    sprintf(tmp_char10,"%.0f", total_volume);
    display.print(F(tmp_char10)); 
    display.print(F(" ml")); 

    // Referência
    // Label
    display.setCursor(0, 40);
    display.print(F("Ref: ")); 
    // Text
    display.setCursor(24, 40);
    sprintf(tmp_char10,"%.1f", set_drops_sec);
    display.print(F(tmp_char10)); 
    display.print(F(" gt/s")); 
  }
}

/**
 * Battery Bar and % Text
 */
void updateBattery()
{
  // Set limiter
  if (battery_status > 100) {
    battery_status = 100;
  }
  // Clear Text Area
  display.fillRect(80, 3, 24, 9, SSD1306_BLACK);
  // Draw Text
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(80, 4);              // Start at top-left corner
  if (battery_status < 10) {
    display.print(F("  "));
  } else if (battery_status < 100) {
    display.print(F(" "));  
  }
  display.print(battery_status);
  display.print(F("%"));
  
  // Clear Battery 
  display.fillRect(112, 5, 10, 6, SSD1306_BLACK);
  // Fill
  int bat_fill = round(battery_status / 10);
  if (bat_fill > 0) {
    // Start pos is 114, but we need fill right to left
    display.fillRect(122 - bat_fill, 4, bat_fill, 6, SSD1306_WHITE);
  }

  // Drawn chargin icon
  if (battery_charging) {
    display.drawLine(116, 5, 118, 5, SSD1306_INVERSE);
    display.drawLine(116, 6, 120, 6, SSD1306_INVERSE);
    display.drawLine(113, 7, 117, 7, SSD1306_INVERSE);
    display.drawLine(115, 8, 117, 8, SSD1306_INVERSE);
  }

  // Draw a single pixel in white
  // display.drawPixel(10, 10, SSD1306_WHITE);

  
}

void updateSpeaker()
{
  int sx = 64;
  // Clean
  display.fillRect(sx, 2, 13, 12, SSD1306_BLACK);

  // Speaker
  if (alert_buzzer == 1) {
    display.drawLine(sx     , 5, sx     ,  9, SSD1306_WHITE);
    display.drawLine(sx     , 5, sx + 2 ,  3, SSD1306_WHITE);
    display.drawLine(sx     , 9, sx + 2 , 11, SSD1306_WHITE);
    display.drawLine(sx + 3 , 6, sx + 3 ,  8, SSD1306_WHITE);
    display.drawPixel(sx + 4, 5, SSD1306_WHITE);
    display.drawPixel(sx + 4, 9, SSD1306_WHITE);
    display.drawLine(sx + 7 , 2, sx + 7 , 12, SSD1306_WHITE);
    display.drawLine(sx + 8 , 3, sx + 8 , 11, SSD1306_WHITE);
    display.drawLine(sx + 9 , 4, sx + 9 , 10, SSD1306_WHITE);
    display.drawLine(sx + 10, 5, sx + 12,  5, SSD1306_WHITE);
    display.drawLine(sx + 10, 9, sx + 12,  9, SSD1306_WHITE);
    display.drawLine(sx + 12, 5, sx + 12,  9, SSD1306_WHITE);
  }
  
}

/**
 * Elapsed Time
 */
void updateTime()
{
  int timestamp = round((millis() - header_timer) / 1000);
  int hours     = floor(timestamp / 3600);
  int minutes   = floor(timestamp % 3600 / 60);
  int seconds   = timestamp % 60;
  char time_string[16];
  
  sprintf(time_string,"%02d:%02d:%02d", hours, minutes, seconds);

  // Serial.print(F("# Time: "));
  // Serial.println(time_string);

  // Clear Text Area
  display.fillRect(3, 3, 50, 9, SSD1306_BLACK);

  // Text Param
  display.setTextSize(1);
  display.setCursor(3, 4);
  display.print(F(time_string)); 

}
