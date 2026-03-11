
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "EEncoder.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/adc.h"

// ================= TFT =================
#define TFT_CS   16
#define TFT_DC   17
#define TFT_RST  20
Adafruit_ST7735 display(TFT_CS, TFT_DC, TFT_RST);

// ================= ENCODER =================
#define ROTARY_ENCODER_A_PIN 10
#define ROTARY_ENCODER_B_PIN 11
#define ROTARY_ENCODER_BUTTON_PIN 12

EEncoder rotaryEncoder(
  ROTARY_ENCODER_A_PIN,
  ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN,
  4
);

// ================= CONSTANTS =================
#define OUTPUT_PIN 22
#define CURRENT_SENSOR_PIN 27
#define VOLTAGE_SENSOR_PIN 26

#define DEFAULT_FREQ_STEP   1000
#define DEFAULT_ONTIME_STEP 1

float vLimit = 30.0f;
float cLimit = 20.0f;

volatile uint16_t vRAWLIMIT;
volatile uint16_t cRAWLIMIT;

// ================= MODES =================
enum Mode{
  MODE_FREQ,
  MODE_ONTIME,
  MODE_VLIMIT,
  MODE_CLIMIT,
  MODE_STEP
};

Mode mode = MODE_FREQ;
Mode previousMode = MODE_FREQ;

// ================= STEP MEMORY =================
const int stepUnits[]={1,10,50,100,500,1000};

int freqStep   = DEFAULT_FREQ_STEP;
int ontimeStep = DEFAULT_ONTIME_STEP;
int vStep      = 1;
int cStep      = 1;

int stepUnitIndex=1;
int stepUnit=stepUnits[1];

// ================= BUTTON =================
bool buttonHeld=false;
bool longPressTriggered=false;
unsigned long buttonPressTime=0;
#define LONG_PRESS_TIME 600

// ================= SHARED =================
volatile uint16_t currentRaw=0;
volatile uint16_t voltageRaw=0;
volatile uint8_t shortCircuitActive = 0;

// ================= MEASUREMENT =================
#define VOLTAGE_SENSE_COEF 0.014652
#define VOLTAGE_SENSE_OFFSET -0.15
#define CURRENT_SENSE_COEF 0.007326f
#define CURRENT_SENSE_OFFSET -0.04f

volatile float currentValue=0;
volatile float voltageValue=0;
float pulseEnergy=0;

// ================= TIMING =================
volatile unsigned long frequency=10000;
volatile uint32_t periodMicros=100;
volatile unsigned long onTimeMicros=1;
volatile unsigned long lastToggleMicros=0;
volatile bool pinState=LOW;

uint32_t lastUIupdate=0;

// ================= FAST GPIO =================
inline void FAST_HIGH(){ sio_hw->gpio_set = 1u<<OUTPUT_PIN; }
inline void FAST_LOW(){ sio_hw->gpio_clr = 1u<<OUTPUT_PIN; }

// ================= FILTER =================
float voltageFilter(float v,float k){ static float o=0;o=(1-k)*o+k*v;return o;}
float currentFilter(float v,float k){ static float o=0;o=(1-k)*o+k*v;return o;}

// ================= ADC =================
inline uint16_t readADC0(){adc_select_input(0);adc_read();return adc_read();}
inline uint16_t readADC1(){adc_select_input(1);adc_read();return adc_read();}

// ================= RAW LIMIT UPDATE =================
void updateRawLimits(){
  vRAWLIMIT = vLimit / VOLTAGE_SENSE_COEF;
  cRAWLIMIT = cLimit / CURRENT_SENSE_COEF;
}

// ================= STEP LOADER =================
void loadStepForMode(){
  switch(mode){
    case MODE_FREQ:   stepUnit=freqStep; break;
    case MODE_ONTIME: stepUnit=ontimeStep; break;
    case MODE_VLIMIT: stepUnit=vStep; break;
    case MODE_CLIMIT: stepUnit=cStep; break;
    default: break;
  }
}

// ================= DISPLAY LIVE =================
void updateLiveValues()
{
  display.fillRect(0,48,130,48,ST77XX_BLACK);

  // ===== VOLTAGE LINE =====
  if(mode==MODE_VLIMIT){
      display.fillRect(0,46,130,14,ST77XX_YELLOW);
      display.setTextColor(ST77XX_BLACK);
  }
  else{
      display.setTextColor(ST77XX_CYAN);
  }

  display.setCursor(4,48);
  display.printf("V:%5.1f",voltageValue);

  display.setCursor(95,48);
  display.printf("L:%2.0f",vLimit);


  // ===== CURRENT LINE =====
  if(mode==MODE_CLIMIT){
      display.fillRect(0,62,130,14,ST77XX_YELLOW);
      display.setTextColor(ST77XX_BLACK);
  }
  else{
      display.setTextColor(ST77XX_GREEN);
  }

  display.setCursor(4,64);
  display.printf("CUR:%5.1f",currentValue);

  display.setCursor(95,64);
  display.printf("L:%2.0f",cLimit);


// ===== ENERGY =====
  display.setTextColor(ST77XX_YELLOW);
  display.setCursor(4,80);
  display.printf("E:%6.2f",pulseEnergy);
  drawShortIndicator();
}

// ================= WAVEFORM =================
void drawWaveform(unsigned long onUs,unsigned long freq){

 unsigned long period=1000000UL/freq;

 int graphY=100,graphH=24;
 int yHigh=graphY,yLow=graphY+graphH;

 display.fillRect(0,graphY-2,130,graphH+4,ST77XX_BLACK);

 int cycleW=160/4;
 int x=0;

 for(int i=0;i<4;i++){
  int onW=(onUs*cycleW)/period;
  display.drawLine(x,yLow,x,yHigh,ST77XX_WHITE);
  display.drawLine(x,yHigh,x+onW,yHigh,ST77XX_WHITE);
  display.drawLine(x+onW,yHigh,x+onW,yLow,ST77XX_WHITE);
  display.drawLine(x+onW,yLow,x+cycleW,yLow,ST77XX_WHITE);
  x+=cycleW;
 }
}

// ================= DISPLAY MAIN =================
void drawModeFill(int y)
{
  display.fillRect(0, y-2, 130, 14, ST77XX_YELLOW);
}
void updateDisplay()
{
 display.fillScreen(ST77XX_BLACK);

 int y=0;

 // ===== FREQUENCY =====
 if(mode==MODE_FREQ){
   drawModeFill(y);
   display.setTextColor(ST77XX_BLACK);
 }else display.setTextColor(ST77XX_WHITE);

 display.setCursor(4,y);
 display.printf("F:%lu Hz",frequency);
 y+=16;

 // ===== ON TIME =====
 if(mode==MODE_ONTIME){
   drawModeFill(y);
   display.setTextColor(ST77XX_BLACK);
 }else display.setTextColor(ST77XX_WHITE);

 display.setCursor(4,y);
 display.printf("ON:%lu us",onTimeMicros);
 y+=16;

 // ===== STEP =====
 if(mode==MODE_STEP){
   drawModeFill(y);
   display.setTextColor(ST77XX_BLACK);
 }else display.setTextColor(ST77XX_WHITE);

 display.setCursor(4,y);
 display.printf("STEP:%d",stepUnit);
 y+=16;

 // ===== VOLTAGE =====
 if(mode==MODE_VLIMIT){
   drawModeFill(y);
   display.setTextColor(ST77XX_BLACK);
 }else display.setTextColor(ST77XX_WHITE);

 display.setCursor(4,y);
 display.printf("V:%5.1f",voltageValue);

 display.setCursor(95,y);
 display.printf("L:%2.0f",vLimit);
 y+=16;

 // ===== CURRENT =====
 if(mode==MODE_CLIMIT){
   drawModeFill(y);
   display.setTextColor(ST77XX_BLACK);
 }else display.setTextColor(ST77XX_WHITE);

 display.setCursor(4,y);
 display.printf("CUR:%5.1f",currentValue);

 display.setCursor(95,y);
 display.printf("L:%2.0f",cLimit);

 // ===== WAVEFORM =====
 drawWaveform(onTimeMicros,frequency);
 drawShortIndicator();
}

// ================= ENCODER =================
void onRotate(EEncoder& enc){

 int inc=enc.getIncrement();

 if(mode==MODE_STEP){
   stepUnitIndex=(stepUnitIndex+inc+6)%6;
   stepUnit=stepUnits[stepUnitIndex];

   switch(previousMode){
     case MODE_FREQ: freqStep=stepUnit; break;
     case MODE_ONTIME: ontimeStep=stepUnit; break;
     case MODE_VLIMIT: vStep=stepUnit; break;
     case MODE_CLIMIT: cStep=stepUnit; break;
     default: break;
   }
   updateDisplay();
   return;
 }

 if(mode==MODE_FREQ){
   long nf=(long)frequency+inc*stepUnit;
   if(nf>1){
     frequency=nf;
     periodMicros=1000000UL/frequency;
   }
 }
 else if(mode==MODE_ONTIME){
   long no=(long)onTimeMicros+inc*stepUnit;
   onTimeMicros=constrain(no,1,(long)periodMicros-1);
 }
 else if(mode==MODE_VLIMIT){
   vLimit+=inc;
   if(vLimit<0)vLimit=0;
   updateRawLimits();
 }
 else if(mode==MODE_CLIMIT){
   cLimit+=inc;
   if(cLimit<0)cLimit=0;
   updateRawLimits();
 }

 updateDisplay();
}

// ================= CORE1 EDM =================
void core1Task()
{
    while(true)
    {
        uint32_t now = time_us_32();

        // =====================================================
        // 1️⃣ PULSE GENERATOR (ONLY TIMING)
        // =====================================================
        uint32_t delayTime =
            pinState ? onTimeMicros :
                       (periodMicros - onTimeMicros);

        if(now - lastToggleMicros >= delayTime)
        {
            lastToggleMicros = now;
            pinState = !pinState;
        }

        // =====================================================
        // 2️⃣ SHORT DETECTION (SEPARATE SYSTEM)
        // =====================================================
        uint16_t localCurrent = currentRaw;
        uint16_t localVoltage = voltageRaw;


bool shortDetected = false;


       if(localCurrent > cRAWLIMIT ||
               localVoltage < vRAWLIMIT)
         {
                shortDetected = true;
          }
        

        shortCircuitActive = shortDetected ? 1 : 0;
        __compiler_memory_barrier();

        // =====================================================
        // 3️⃣ OUTPUT GATE (FINAL DRIVER CONTROL)
        // =====================================================
        if(pinState && !shortDetected)
            FAST_HIGH();
        else
            FAST_LOW();
    }
}
//////////////////////////////////////////////////////////////
void drawShortIndicator()
{
    static uint8_t lastState=255;

    uint8_t state = shortCircuitActive;   // atomic read

    if(lastState==state) return;
    lastState = state;

    if(state)
        display.fillRect(131,0,29,128,ST77XX_RED);
    else
        display.fillRect(131,0,29,128,ST77XX_GREEN);
}
// ================= SETUP =================
void setup(){

 pinMode(ROTARY_ENCODER_BUTTON_PIN,INPUT_PULLUP);

 SPI.begin();
 display.initR(INITR_BLACKTAB);
 display.setRotation(1);

 adc_init();
 adc_gpio_init(26);
 adc_gpio_init(27);
 adc_set_clkdiv(0);

 pinMode(OUTPUT_PIN,OUTPUT);

 rotaryEncoder.setEncoderHandler(onRotate);
 rotaryEncoder.setAcceleration(350);

 periodMicros=1000000UL/frequency;
 updateRawLimits();
 loadStepForMode();

 updateDisplay();

 multicore_launch_core1(core1Task);
}

// ================= LOOP =================
void loop(){

 rotaryEncoder.update();
 rotaryEncoder.update();
 rotaryEncoder.update();

 // ---------- BUTTON ----------
 bool pressed=digitalRead(ROTARY_ENCODER_BUTTON_PIN)==LOW;

 if(pressed && !buttonHeld){
   buttonHeld=true;
   buttonPressTime=millis();
   longPressTriggered=false;
 }

 if(buttonHeld && !longPressTriggered &&
    millis()-buttonPressTime>=LONG_PRESS_TIME){
   longPressTriggered=true;
   previousMode=mode;
   mode=MODE_STEP;
   updateDisplay();
 }

 if(!pressed && buttonHeld){
   buttonHeld=false;

   if(!longPressTriggered){
      mode=(Mode)((mode+1)%4);
      loadStepForMode();
   }
   else{
      mode=previousMode;
      loadStepForMode();
   }

   updateDisplay();
 }

 // ---------- ADC ----------
 uint16_t vRaw=readADC0();
 uint16_t cRaw=readADC1();

 voltageRaw=vRaw;
 currentRaw=cRaw;

 float instantVoltage=vRaw*VOLTAGE_SENSE_COEF+VOLTAGE_SENSE_OFFSET;
 float instantCurrent=cRaw*CURRENT_SENSE_COEF+CURRENT_SENSE_OFFSET;

 voltageValue=voltageFilter(instantVoltage,0.03f);
 currentValue=currentFilter(instantCurrent,0.03f);

 pulseEnergy=voltageValue*currentValue*onTimeMicros/1000.0f;

 uint32_t now=time_us_32();

 if(now-lastUIupdate>=100000){
   lastUIupdate=now;
   updateLiveValues();
 }
}