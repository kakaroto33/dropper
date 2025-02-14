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
#include <RotaryEncoder.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

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
#define SCREEN_SETUP_TIMEOUT 30 // In seconds

//== ROTATORY ENCODER =========================================================

#define ROTATORY_IN1 26
#define ROTATORY_IN2 27
#define ROTATORY_SWT 14

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(ROTATORY_IN1, ROTATORY_IN2, RotaryEncoder::LatchMode::TWO03);

//== GLOBAL VARIABLES =========================================================

// System State
char display_mode = 'D';  // D = Default, S = Setings
int setup_timout  = 0;    // Timout seup screen

// Display Values
int battery_status  = 0;
int drops_second    = 88;
int milliliters_sec = 888;
int total_volume    = 888;
float set_drops_sec = 8.8;

// Rotatory Encoder
int encoder_pos   = 0;
int encoder_click = 0;

// Update Timers
int update_medium  = 0;   // Update every 1 second
int update_long    = 0;   // Update every 10 seconds
int update_test    = 0;
bool refresh_med   = false;
bool refresh_long  = false;
bool refresh_disp  = false; 

// Temp variables
int tmp_stamp  = 0;
int tmp_stamp2 = 0;
int tmp_pos    = 0;
char tmp_char10[10];

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
  //
  uint32_t Freq = 0;
  setCpuFrequencyMhz(80);
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
  prepareDefaultScreen();
  display.display();

  // Inputs
  pinMode(ROTATORY_SWT, INPUT_PULLUP);

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
    

    tmp_stamp2 = round(millis() / 1000);
    if (update_test != tmp_stamp2) {
      // Serial.print("[TASK_1] Core: ");
      // Serial.println(xPortGetCoreID());

      if (battery_status >= 100) {
        battery_status = 0;
      } else {
        battery_status++;
      }

      if (drops_second >= 99) {
        drops_second = 0;
      } else {
        drops_second++;
      }
      update_test = tmp_stamp2;
    }

    //
    // encoder.tick();
    // int newPos = encoder.getPosition();
    // if (encoder_pos != newPos) {
    //   encoder_pos = newPos;
    //   Serial.print("pos:");
    //   Serial.print(newPos);
    //   Serial.print(" dir:");
    //   newPos = (int)encoder.getDirection();
    //   Serial.println(newPos);
    // } 
    

    // Add a small delay to let the watchdog process
    //https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    delay(1);
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
    tmp_stamp    = round(millis() / 1000);
    refresh_med  = false;
    refresh_long = false;
    refresh_disp = false;

    // Update on 1fps
    if (update_medium != tmp_stamp) {
        update_medium = tmp_stamp;
        refresh_med   = true;
    } 
    // Update ~10 fps
    if (update_long < tmp_stamp) {
      update_long  = tmp_stamp + 10;
      refresh_long = true;
    }

    // Detect encode change
    encoder.tick();
    tmp_pos = encoder.getPosition();
    if (encoder_pos != tmp_pos) {
      Serial.print("pos:");
      Serial.print(tmp_pos);
      Serial.print(" dir:");
      Serial.println((int)(encoder.getDirection()));
      encoder_pos = tmp_pos;
      if (display_mode == 'S') {
        setup_timout = tmp_stamp + SCREEN_SETUP_TIMEOUT;
      }
    } // if

    // Read the state of the pushbutton value:
    if (digitalRead(ROTATORY_SWT) == LOW) {
      Serial.println("[ROTATORY_SWT:LOW]");
      if (encoder_click == 0) {
        if (display_mode == 'D') {
          display_mode = 'S';
          prepareSetupScreen();
          refresh_disp = true;
          refresh_med  = true;
          refresh_long = true;
        }
        setup_timout = tmp_stamp + SCREEN_SETUP_TIMEOUT;
      }
      encoder_click++;
    } else {
      encoder_click = 0;
    }

    // Default Display Mode 
    if (display_mode == 'D') {
      // Update on 1fps
      if (refresh_med) {
        updateValues();
        refresh_disp = true;
      } 
      // // Update ~10 fps
      // if (refresh_long) {
      //   updateBattery();
      //   refresh_disp = true;
      // }
    } 
    // Setup Display Mode
    else {
      // SCREEN_SETUP_TIMEOUT
      // Update on 1fps
      if (refresh_med) {
        
        //display.display();
      } 

      // Check display  setup timeout
      if (tmp_stamp > setup_timout) {
        display_mode = 'D';
        prepareDefaultScreen();
        refresh_disp = true;
      }
    }

    // Header bar always display
    // Update on 1fps
    if (refresh_med) {
      updateTime();
      refresh_disp = true;
    } 
    // Update ~10 fps
    if (refresh_long) {
      updateBattery();
      refresh_disp = true;
    }

    if (refresh_disp) {
      display.display();
    }
    
    // Add a small delay to let the watchdog process
    //https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    delay(1); // Pause for 0.05 second
  }
}

//=============================================================================
//== FUNCTIONS ================================================================
//=============================================================================


/**
 * Prepare display yellow header
 */
void prepareHeader()
{
  // [Yellow Bar] Battery Body Outer Line
  display.drawLine(110,  6, 110, 11, SSD1306_WHITE);
  display.drawLine(111,  6, 111, 11, SSD1306_WHITE);
  display.drawLine(112,  4, 112, 13, SSD1306_WHITE);
  display.drawLine(125,  4, 125, 13, SSD1306_WHITE);
  display.drawLine(112,  4, 125,  4, SSD1306_WHITE);
  display.drawLine(112, 13, 125, 13, SSD1306_WHITE);
}

/**
 * Prepare display with fixed drawn parts and labels
 */
void prepareDefaultScreen()
{
  // Clear display buffer
  display.clearDisplay(); 
  prepareHeader();

  // Separator
  display.drawLine(0 , 16, MAX_WIDTH,  16, SSD1306_WHITE);        // Header Line
  display.drawLine(75, 20, 75, MAX_HEIGHT - 2, SSD1306_WHITE);    // Column Line

  // Labels
  // Velocidade Milliliters per hour
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(F("Vel: ")); 

  // Total Milliliters
  display.setCursor(0, 30);
  display.print(F("Tot: "));  

  // Referência
  display.setCursor(0, 40);
  display.print(F("Ref: ")); 

  // Gotas por segundo
  display.setCursor(82, 53); 
  display.print(F("gotas/s"));

}

void prepareSetupScreen()
{
  // Clear display buffer
  display.clearDisplay();
  prepareHeader();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(F("SETUP @@@@@@@@@@@@@@")); 

}

/**
 * Update Text Values
 */
void updateValues()
{
     
  // Body Drops/sec
  // Custom Dot
  display.fillRect(80, 22, 44, 29, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);   
  display.setCursor(80, 22); 
  sprintf(tmp_char10,"%02d", drops_second);
  display.print(F(tmp_char10)); 
  // Custom Dot
  display.fillRect(100, 48, 4, 4, SSD1306_WHITE);
    
  // Velocidade Milliliters per hour
  display.setTextSize(1);
  display.setCursor(24, 20);
  display.print(milliliters_sec); 
  display.print(F(" ml/h")); 

  // Total Milliliters
  display.setCursor(24, 30);
  display.print(total_volume); 
  display.print(F(" ml")); 

  // Referência
  display.setCursor(24, 40);
  sprintf(tmp_char10,"%.1f", set_drops_sec);
  display.print(F(tmp_char10)); 
  display.print(F(" gt/s")); 
    
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
  display.fillRect(82, 3, 24, 9, SSD1306_BLACK);
  // Draw Text
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(82,5);              // Start at top-left corner
  if (battery_status < 10) {
    display.print(F("  "));
  } else if (battery_status < 100) {
    display.print(F(" "));  
  }
  display.print(battery_status);
  display.print(F("%"));
  
  // Clear Battery 
  display.fillRect(114, 6, 10, 6, SSD1306_BLACK);
  // Fill
  int bat_fill = round(battery_status / 10);
  if (bat_fill > 0) {
    // Start pos is 114, but we need fill right to left
    display.fillRect(124 - bat_fill, 6, bat_fill, 6, SSD1306_WHITE);
  }
}

/**
 * Elapsed Time
 */
void updateTime()
{
  int timestamp = round(millis() / 1000);
  int hours     = floor(timestamp / 3600);
  int minutes   = floor(timestamp % 3600 / 60);
  int seconds   = timestamp % 60;
  char time_string[16];
  
  sprintf(time_string,"%02d:%02d:%02d", hours, minutes, seconds);

  // Serial.print(F("# Time: "));
  // Serial.println(time_string);

  // Clear Text Area
  display.fillRect(0, 3, 50, 9, SSD1306_BLACK);

  // Text Param
  display.setTextSize(1);
  display.setCursor(0, 5);
  display.print(F(time_string)); 

}

/*
void testareas() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer
  // Yellow Bar
  display.drawLine(0, 0, display.width()-1, 0, SSD1306_WHITE);
  display.drawLine(0, 15, display.width()-1, 15, SSD1306_WHITE);

  // Body
  display.drawLine(0, 16, display.width()-20, 16, SSD1306_WHITE);
  display.drawLine(0, display.height() - 1, display.width()-20, display.height() - 1, SSD1306_WHITE);

  display.display();

  // 
  // for(i=0; i < 17; i++) {
  //   display.drawLine(0, i, display.width()-1, i, SSD1306_WHITE);
  //   display.display();
  //   delay(1);
  // }
  // delay(250);
  // for(i=18; i < display.height(); i++) {
  //   display.drawLine(0, i, display.width()-20, i, SSD1306_WHITE);
  //   display.display();
  //   delay(1);
  // }

  delay(250);
}

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}


void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}
*/
