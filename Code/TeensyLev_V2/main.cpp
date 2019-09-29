/*
* main source file
* Created:  05/10/2018 23:07:43
* Author:   Vindar
*
* Source code for TeensyLev V2 (with adafruit mini TFT + joystick breakout).
*
*/


#include <Arduino.h>
#include <SPI.h>                            // spi library 
#include <DMAChannel.h>                     // DMA library by PJRC, use to generate stable 40Khz signal combined with FTM timers
#include <Adafruit_GFX.h>                   // Core graphics library -> USE THE ORIGINAL ADAFRUIT LIBRARY AND NOT THE OUTDATED TEENSY VERSION
#include <Adafruit_ST7735.h>                // Hardware-specific library for ST7735 -> USE THE ORIGINAL ADAFRUIT LIBRARY AND NOT THE OUTDATED TEENSY VERSION
#include <Adafruit_miniTFTWing.h>           // control button via seesaw
#include <Fonts/FreeSans9pt7b.h>            // font used
#include <Fonts/FreeMono12pt7b.h>           // font used

#include "fastcos.h"
#include "startlogo.h"





/**
* Very fast RNG. (for test purpose: no guarantee of good statistical properties). 
* The generator always use the same deterministic seed.
**/
class FastRNG
{

public:

    /* type of integer returned by the generator */
    typedef uint32_t result_type;

    /* Default constructor. Always initialize with the same seed. */
    FastRNG() : _gen_x(123456789), _gen_y(362436069), _gen_z(521288629) { }

    /* min value */
    static constexpr result_type min() { return 0; }

    /* max value */
    static constexpr result_type max() { return 4294967295UL; }

    /* return a random number */
    inline uint32_t operator()()
        {
        uint32_t t;
        _gen_x ^= _gen_x << 16; _gen_x ^= _gen_x >> 5; _gen_x ^= _gen_x << 1;
        t = _gen_x; _gen_x = _gen_y; _gen_y = _gen_z; _gen_z = t ^ _gen_x ^ _gen_y;
        return _gen_z;
        }

    /* return a uniform number on [0,1).  */
    inline float unif() { return ((float)(operator()())) / (4294967296.0); }

    /* discard results */
    void discard(unsigned long long z) { for (unsigned long long i = 0; i < z; i++) operator()(); }


private:

    uint32_t _gen_x, _gen_y, _gen_z;        // state of the generator

};



/* generate N(0,1) random variable using "ratio of uniform" method 
   taken from "Numerical recipies in C++" 
 */
template<class random_t> inline float normalLaw(random_t & gen)
    {
    float u, v, x, y, q;
    do {
        u = gen.unif(); v = 1.7156*(gen.unif() - 0.5);
        x = u - 0.449871; y = abs(v) + 0.386595;
        q = (x*x) + y * (0.19600*y - 0.25472*x);
        } 
    while ((q > 0.27597) && (q > 0.27846 || (v*v) > -4.*logf(u)*(u*u)));
    return (v / u);
    }


/**********************************************************************
* MISC
**********************************************************************/


/** Format a floating point value inside a buffer */
char * tostring(char * dest, float val, const char * unit) { sprintf(dest, "%.2f%s", val, unit); return dest; }

/** Format an integer value inside a buffer */
char * tostring(char * dest, int32_t val, const char * unit) { sprintf(dest, "%li%s", val, unit); return dest; }



/**********************************************************************
* VOLTAGE CONTROL
**********************************************************************/

const int32_t ANALOG_RES_BIT = 10;                                  // DO NOT CHANGE -> does not work with other values
const int32_t ANALOG_RES = (((int32_t)1) << ANALOG_RES_BIT) - 1;    // 

const int VOLTAGE_SENSOR_PIN = A8;                                  // analog pin to detect current voltage. 
const int VOLTAGE_CONTROL_PIN = A14;                                // DAC pin to adjust voltage

const float DEFAULT_DAC = 0.5;                                      // default start value
const float DAC_STEP = 0.005;                                       // change step

volatile float _dac = DEFAULT_DAC;                                  // current dac value


volatile float prev_voltage = 0;
                                                                    /* return the current voltage */
inline float voltage()
{
    int32_t r = analogRead(VOLTAGE_SENSOR_PIN);
    float v = ((r*3.3f) / ANALOG_RES) * ((3.3 + 12.0) / 3.3); // voltage divider with resistors of 3.3K and 12K
    const float diff = v - prev_voltage;
    if ((diff <= 0.02) && (diff >= -0.02))
        {
        v = prev_voltage;
        }
    prev_voltage = v;
    return v;
}

/* return the current dac value between 0.0 and 1.0 */
inline float getDAC() { return _dac; }

/* set the dac value between 0.0 and 1.0 */
inline void setDAC(float val = DEFAULT_DAC)
{
    if (val < 0.0f) val = 0.0f; else if (val > 1.0f) val = 1.0f;
    _dac = val;
    int32_t V = (int32_t)((1.0f - _dac)*ANALOG_RES);
    analogWrite(VOLTAGE_CONTROL_PIN, V);
}

/* increase dac */
inline void incDAC() { setDAC(_dac + DAC_STEP); }

/* decrease dac */
inline void decDAC() { setDAC(_dac - DAC_STEP); }

/* initialization */
void initVoltage()
{
    analogReference(EXTERNAL);
    analogReadAveraging(32);
    analogWriteResolution(ANALOG_RES_BIT);
    analogReadResolution(ANALOG_RES_BIT);
    pinMode(VOLTAGE_SENSOR_PIN, INPUT);     // pin for reading voltage
    pinMode(VOLTAGE_CONTROL_PIN, OUTPUT);   // pin  for setting voltage
    setDAC();                               // set the default voltage
}


/**********************************************************************
* FTM TIMERS
**********************************************************************/


/* convert mod to frequency */
constexpr int32_t freq_to_mod(double freq) { return (int32_t)(F_BUS + freq) / (2 * freq); }

/* convert frequency to mod */
constexpr double mod_to_freq(int32_t mod) { return ((double)F_BUS) / (2 * mod - 1); }


const double DEFAULT_FREQ = 40000;
const double MIN_FREQ = 5000;
const double MAX_FREQ = 80000;

const int32_t DEFAULT_MOD   = freq_to_mod(DEFAULT_FREQ);        // mod for default frequency
const int32_t MIN_MOD       = freq_to_mod(MIN_FREQ);            // mod for min frequency
const int32_t MAX_MOD       = freq_to_mod(MAX_FREQ);            // mod for max frequency

volatile int32_t _mod;      // number of ticks per period w.r.t the internal clock to get frequency freq.
volatile int32_t _phase0;   // position of the first trigger; must be between 1 and mod -1
volatile int32_t _phase1;   // position of the second trigger; must be between 1 and mod -1
volatile int32_t _diff;     // keep track of the two pi phase difference


/* return the current frequency */
inline double getFreq() { return mod_to_freq(_mod); }

/* create and start the timer */
inline void createTimer()
{
    _mod = freq_to_mod(DEFAULT_FREQ);                           // set number of ticks w.r.t. frequency
    _phase0 = _mod / 2;                                         // first trigger
    _phase1 = _mod / 2 + 1;                                     // second trigger
    _diff = 0;                                                  // no phase diff
                                                                // set the FTM timer    
    FTM2_SC = 0;                                                // status and control initially disabled
    FTM2_CNT = 0;                                               // set counter at zero
    FTM2_MOD = _mod - 1;                                        // store it
    FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);                    // start the timer using the system clock. (divide by 1)
    FTM2_C0SC = 0x69;                                           // channel interrupt enable and edge aligned pwm
    FTM2_C1SC = 0x69;                                           // channel interrupt enable and edge aligned pwm
    FTM2_C0V = _phase0;                                         // trigger CH0 at this value -> must not be 0 or mod-1
    FTM2_C1V = _phase1;                                         // trigger CH1 at this value -> must not be 0 or mod-1
    CORE_PIN32_CONFIG = PORT_PCR_IRQC(1) | PORT_PCR_MUX(3);     // pin 32 is associated with FTM2_CH0 (mux(3)  and triggers DMA(port B) on rising edge (IRQC(1))    
}


/** Increment phase */
inline void incPhase()
{
    _diff++;
    if (_diff >= _mod - 1) _diff = -_mod + 1;
    if (_phase0 < _mod - 1) { FTM2_C0V = (++_phase0); return; }
    FTM2_C0V = _phase0 = 1;
    return;
}


/** Decrement phase */
inline void decPhase()
{
    _diff--;
    if (_diff <= -_mod) _diff = _mod - 2;
    if (_phase1 < _mod - 1) { FTM2_C1V = (++_phase1); return; }
    FTM2_C1V = _phase1 = 1;
    return;
}


/** return the phase difference in [-1,1] */
inline float getdiffphase()
{
    return (-1.0f + 2 * ((float)(_diff + _mod - 1)) / ((float)(2 * _mod - 2)));
}
    



/**********************************************************************
* PHASE AND FREQUENCY CONTROL VIA INTERRUPT
**********************************************************************/

const int8_t            UP   = 1;   // phase directions
const int8_t            DOWN = -1;  // 
const int8_t            NONE = 0;   // 

const int32_t DEFAULT_COMMAND_RATE = 1000;  // 1ms between counts


/** Structure containing a phase command, chained list */
struct PhaseCommand
    {
    PhaseCommand * next;        // pointer to the next command (or nullptr if no more command)
    int32_t rate;               // number of microsecond between each count
    int32_t count;              // number of counts to perform (negative = infinite)
    int8_t  dir;                // direction of the phase shifting (UP or DOWN or NONE)
    };


IntervalTimer       phaseint;       // object that manage the interrupt routine

/* command being executed */
volatile int32_t        current_count;      // number of count remaining
volatile int32_t        current_rate;       // current interrupt rate
volatile float          current_mult_scale; // current mult scale factor
volatile int8_t         current_dir;        // direction of the current command
volatile PhaseCommand * current_com;        // pointer to the command structure

const int32_t DELTA_MINMAXPOS = 500;

volatile int32_t        current_position;       // current position
volatile int32_t        min_position;           // min position
volatile int32_t        max_position;           // min position

/* command modifier */
volatile float          adjust_mult_scale = 1.0f;   // scale multiplier
volatile float          adjust_rate_scale = 1.0f;   // rate multiplier
volatile int32_t        adjust_offset = 0;          // offset to apply

const int32_t MAX_ADJUST_OFFSET_RATE = 1000;    // rates bound 
const int32_t MIN_ADJUST_OFFSET_RATE = 200;     // for offset change

const int32_t OFFSET_UNIT = 1000;   // INCEMENT/DECREMENT VALUE FOR OFFSET
const float MULT_RATE_UNIT = 1.1;   // INCREMENT/DECREMENT VALUE FOR COMMAND COUNT RATE MULTIPLIER
const float MULT_SCALE_UNIT = 1.1;  // INCREMENT/DECREMENT VALUE FOR COMMAND COUNT SCALE MULTIPLIER


/** Phase command interrupt */
FASTRUN void phaseInterrupt()
    {
    if (adjust_offset != 0) // phase offset adjustement comes first
        { 
        if (adjust_offset > 0) { incPhase(); current_position++; adjust_offset--; } else { decPhase(); current_position--; adjust_offset++; }
        // change rate while offsetting if it is too fast or too slow
        if (current_rate > MAX_ADJUST_OFFSET_RATE)
            { // too slow, speed up
            current_rate = MAX_ADJUST_OFFSET_RATE;
            phaseint.update(current_rate);
            }
        else if (current_rate < MIN_ADJUST_OFFSET_RATE)
            { // too fast, speed down
            current_rate = MIN_ADJUST_OFFSET_RATE;
            phaseint.update(current_rate);
            }
        if (current_position < min_position) min_position = current_position;
        if (current_position > max_position) max_position = current_position;
        return;
        }
    if (current_com == nullptr) return; // no command, return. 
    if (current_count != 0)
        { // execute the command
        if (current_dir == UP) { incPhase(); current_position++; } else { decPhase(); current_position--; } 
        if (current_count > 0) current_count--;
        if (current_position < min_position) min_position = current_position;
        if (current_position > max_position) max_position = current_position;
        }   
    if (current_count != 0)
        {
        // do we need to adjust the scale. 
        if (adjust_mult_scale != current_mult_scale)
            { // yes
            current_count = (int32_t)(current_count * (adjust_mult_scale / current_mult_scale)); // rescale the remaining count
            current_mult_scale = adjust_mult_scale;
            }
        // do we need to adjust the rate  ?
        const int32_t nrate = (int32_t)(adjust_rate_scale * current_com->rate);
        if (nrate != current_rate)
            { // yes
            current_rate = nrate;
            phaseint.update(current_rate);
            }
        }
    if(current_count == 0)
        { // command finished so we load the next one
        current_com = current_com->next;                                        // the next command
        if (current_com == nullptr) { return; }                                 // no more command, we are done
        current_count = (int32_t)(adjust_mult_scale * (current_com->count));    // set the total count (rescaled)
        current_mult_scale = adjust_mult_scale;                                 // save the current mult-scale
        current_dir = current_com->dir;                                         // set the direction
        const int32_t nrate = (int32_t)(adjust_rate_scale * current_com->rate);
        if (current_rate != nrate)          
            {
            current_rate = nrate;
            phaseint.update(current_rate);                                  
            }
        }
    }


/* launch a new phase command, override the current one (if any) */
void setPhaseCommand(PhaseCommand * com)
    {
    noInterrupts();
    current_com = com;
    if (com == nullptr)
        {
        current_count = -1;         // default values
        current_mult_scale = 1.0f;  //
        current_dir = UP;           //
        current_rate = 1000;        //
        }
    else
        {
        current_count = (int32_t)(adjust_mult_scale * (com->count));
        current_mult_scale = adjust_mult_scale;
        current_dir = com->dir;
        current_rate = (int32_t)(adjust_rate_scale * com->rate);
        phaseint.priority(64);
        phaseint.begin(phaseInterrupt, current_rate);
        }
    interrupts();
    }


/**
 * Launch a single command. Override the current one (if any)
 *
 * @param   micros  microsecond between each count
 * @param   count   number of counts (negative = infinite)
 * @param   dir     direction (UP, DOWN, NONE)
 **/
void singlePhaseCommand(int32_t micros, int32_t count, int8_t dir)
    {
    static PhaseCommand singlecom;
    singlecom.count = count;
    singlecom.dir = dir;
    singlecom.next = nullptr;
    singlecom.rate = micros;
    setPhaseCommand(&singlecom);
    }


/* stop any ongoing phase command */
inline void cancelPhaseCommand() { setPhaseCommand(nullptr); }


/**
 * Wait until there is no more phase shifting in progress
 */
inline void waitEndPhaseCommand()
    {
    while (current_com != nullptr) { yield(); }
    }


/* set the rate multplier */
inline void commandRate(float mult_rate = 1.0f) { noInterrupts(); adjust_rate_scale = mult_rate; interrupts(); }

/* increase the rate multiplier by one unit */
inline void commandRateInc() { noInterrupts(); adjust_rate_scale /= MULT_RATE_UNIT; interrupts(); }

/* decrease the rate multiplier by one unit */
inline void commandRateDec() { noInterrupts(); adjust_rate_scale *= MULT_RATE_UNIT; interrupts(); }

/* return the rate multiplier*/
inline float getCommandRate() { return adjust_rate_scale; }



/* set the scale multplier */
inline void commandScale(float mult_rate = 1.0f) { noInterrupts(); adjust_mult_scale = mult_rate; interrupts(); }

/* increase the scale multiplier by one unit */
inline void commandScaleInc() { noInterrupts(); adjust_mult_scale *= MULT_SCALE_UNIT; interrupts(); }

/* decrease the scale multiplier by one unit */
inline void commandScaleDec() { noInterrupts(); adjust_mult_scale /= MULT_SCALE_UNIT; interrupts(); }

/* return the scale multiplier*/
inline float getCommandScale() { return adjust_mult_scale; }


/* increase the offset */
inline void commandOffsetInc() { noInterrupts(); adjust_offset += OFFSET_UNIT; interrupts(); }

/* decrease the offset */
inline void commandOffsetDec() { noInterrupts(); adjust_offset -= OFFSET_UNIT; interrupts(); }

/* return the current_postion variable to 0 */
inline void resetOffset() { noInterrupts();  adjust_offset = 0; interrupts(); }


/* return the current_postion variable to 0 */
inline void resetMinMaxPos() { noInterrupts();  min_position = current_position - DELTA_MINMAXPOS; max_position = current_position + DELTA_MINMAXPOS; interrupts(); }




/**
 * Set phase to its original value (180 deg)
 * Return immediately. 
 * 
 * @param   micro   time between each phase shift. 
 **/
void phaseReset(int32_t micro = DEFAULT_COMMAND_RATE)
    {
    if (_diff == 0) return;
    if (_diff > 0) { singlePhaseCommand(micro, _diff, DOWN); } else { singlePhaseCommand(micro, -_diff, UP); }
    }


/**
 * Align the phase to the next quarter (-90, 0, 90, 180 deg) 
 * Return immediately.
 *
 * @param   micro   time between each phase shift.
 **/
void phaseAlignNext(int32_t micro = DEFAULT_COMMAND_RATE)
    {
    int32_t g = _diff + _mod - 1;
    const int32_t q1 = (_mod - 1) / 2;
    if (g < q1) { singlePhaseCommand(micro, q1 - g, UP); return; }
    const int32_t q2 = (_mod - 1);
    if (g < q2) { singlePhaseCommand(micro, q2 - g, UP); return; }
    const int32_t q3 = q1 + q2;
    if (g < q3) { singlePhaseCommand(micro, q3 - g, UP); return; }
    singlePhaseCommand(micro, q2 + q2 - g, UP);
    }



/**
 * Decrement the frequency by one unit. 
 * This resets the phase to its default value.
 * Return when the whole operation is completed.
 * 
 * @param   micro   time between each phase shift (if resetting phase needed). 
 **/
inline void decFreq(int32_t micro = DEFAULT_COMMAND_RATE)
    {
    if (_mod >= MIN_MOD) return;
    if (_diff != 0) { phaseReset(micro); waitEndPhaseCommand(); }// set phase to 0 before changing frequency
    _mod++;
    FTM2_MOD = _mod - 1;
    }


/**
* Increment the frequency by one unit.
* This resets the phase to its default value.
* Return when the whole operation is completed.
*
* @param    micro   time between each phase shift (if resetting phase needed).
**/
inline void incFreq(int32_t micro = DEFAULT_COMMAND_RATE)
    {
    if (_mod <= MAX_MOD) return;
    if (_diff != 0) { phaseReset(micro); waitEndPhaseCommand();  } // set phase to 0 before changing frequency
    if (_phase0 == _mod - 2)
        { // phase0 = phase1 = -mod - 2 : we increase both so they are 0 before we decrease _mod. 
        incPhase();
        decPhase();
        }
    _mod--;
    FTM2_MOD = _mod - 1;
    }


/**
 * Resets the frequency to its default value
 * This also resets the phase to its default value.
 * Return when the whole operation is completed.
 *
 * @param   micro   (Optional) The micro.
 **/
void resetFreq(int32_t micro = DEFAULT_COMMAND_RATE)
    {
    while (_mod != DEFAULT_MOD)
        {
        if (DEFAULT_MOD > _mod) decFreq(micro); else incFreq(micro);
        delayMicroseconds(micro);
        }
    }



/**********************************************************************
* DMA CHANNELS
**********************************************************************/

// pins to output signal (on GPIOD_PDOR = {2,14,7,8,6,20,21,5} )
const int BRIDGE_IN1_PIN = 5;               // Motor bridge A
const int BRIDGE_IN2_PIN = 6;               // Motor bridge A
const int BRIDGE_IN3_PIN = 7;               // Motor bridge B
const int BRIDGE_IN4_PIN = 8;               // Motor bridge B

DMAChannel dmaA, dmaB;                      // the two dma channels
DMASetting dmaSettingA, dmaSettingB;        // settings for the DMA chanels 
volatile uint8_t toggleA = 16 + 128;        // toogle pin 5 and 6 on portD = {2,14,7,8,6,20,21,5} (volatile to insure it is put in RAM)
volatile uint8_t toggleB = 4 + 8;           // toogle pin 7 and 8 on portD = {2,14,7,8,6,20,21,5} (volatile to insure it is put in RAM)
const int dma_len = 10000;                  // number of bytes per dma transaction


/* interrupt called at completion (UNUSED)
void dmaInt() { static int stl = 1; stl = 1 - stl; dmaA.clearInterrupt(); } */


/* start the dma transfer synchronized with the FTM timer. */
void startDMA()
{
    pinMode(BRIDGE_IN1_PIN, OUTPUT);    // signal output pin
    pinMode(BRIDGE_IN2_PIN, OUTPUT);    //
    pinMode(BRIDGE_IN3_PIN, OUTPUT);    //
    pinMode(BRIDGE_IN4_PIN, OUTPUT);    //

    digitalWrite(BRIDGE_IN1_PIN, HIGH);   // pins on the same motor driver always have opposite sign. 
    digitalWrite(BRIDGE_IN2_PIN, LOW);    // 
    digitalWrite(BRIDGE_IN3_PIN, HIGH);   // 
    digitalWrite(BRIDGE_IN4_PIN, LOW);    //

                                          // settings for dmaA : triggers on FTM CH0 and swap signal on IN1 and IN2
    dmaSettingA.source(toggleA);                            // byte used for output
    dmaSettingA.destination(GPIOD_PTOR);                    // toggle mode on portD
    dmaSettingA.transferSize(1);                            // one byte at a time
    dmaSettingA.transferCount(dma_len);                     // total number of bytes per transfer
    dmaSettingA.replaceSettingsOnCompletion(dmaSettingA);   // chain with itself    
                                                            //dmaSettingA.interruptAtCompletion();                  

                                                            // settings for dmaB : triggers on FTM CH0 and swap signal on IN3 and IN4
    dmaSettingB.source(toggleB);                            // byte used for output
    dmaSettingB.destination(GPIOD_PTOR);                    // toggle mode on portD
    dmaSettingB.transferSize(1);                            // one byte at a time
    dmaSettingB.transferCount(dma_len);                     // total number of bytes per transfer
    dmaSettingB.replaceSettingsOnCompletion(dmaSettingB);   // chain with itself

                                                            // load settings into the actual dma objects
    dmaA = dmaSettingA;
    dmaB = dmaSettingB;

    //dmaA.attachInterrupt(dmaInt);

    dmaA.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0);    // trigger with phase0
    dmaB.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH1);    // trigger with phase1

    DMAPriorityOrder(dmaB, dmaA);
    dmaA.enable();
    dmaB.enable();
}



/**********************************************************************
* SCREEN MANAGEMENT
**********************************************************************/

const int8_t TFT_RST_PIN = -1;      // no pin : reset is managed by seesaw
const int8_t TFT_CS_PIN = 10;       // CS pin for spi transaction with the TFT screen
const int8_t TFT_DC_PIN = 9;        // DS pin used by TFT screen for choose operation mode

Adafruit_miniTFTWing ss;                                                        // seesaw driver for joystick/button, resetting the screen and setting its brightness. 
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);     // TFT screen driver

/*  offscreen buffer class  */
class GFXBuffer : public Adafruit_GFX {
public:

    static const int16_t BUF_WIDTH = 160;   // screen width
    static const int16_t BUF_HEIGHT = 80;   // screen height

    GFXBuffer() : Adafruit_GFX(BUF_WIDTH, BUF_HEIGHT)
    {
    }

    virtual ~GFXBuffer()
    {
    }

    /* draw a pixel on the buffer */
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color)  override
    {
        if ((x < 0) || (y < 0) || (x >= BUF_WIDTH) || (y >= BUF_HEIGHT)) return;
        buffer[x + BUF_WIDTH * y] = color;
    }

    /* clear the screen with a given color */
    virtual void fillScreen(uint16_t color) override
    {
        const uint8_t hi = color >> 8, lo = color & 0xFF;
        if (hi == lo) { memset(buffer, lo, WIDTH * HEIGHT * 2); return; }
        for (uint32_t i = 0; i<(BUF_WIDTH * BUF_HEIGHT); i++) buffer[i] = color;
    }

    /* return a pointer to the offscreen buffer */
    uint16_t* getBuffer(void) { return buffer; }

private:

    uint16_t buffer[BUF_WIDTH*BUF_HEIGHT];  // the offscreen buffer
};


GFXBuffer obuf;                 // the offscreen buffer


/* blit the offscreen buffer onto the screen*/
inline void blitBuffer()
    {
    tft.drawRGBBitmap(0, 0, obuf.getBuffer(), GFXBuffer::BUF_WIDTH, GFXBuffer::BUF_HEIGHT);
    }


/* init the tft screen and the seesaw chip */
void initUIControl()
{
    ss.begin();                     // init seesaw
    ss.tftReset();                  // reset the screen
    ss.setBacklight(0);             //set the backlight 
    tft.initR(INITR_MINI160x80);    // initialize a ST7735S chip, mini display
    tft.setRotation(3);             // screen orientation
}



/**********************************************************************
* INPUT CONTROL AND POWERSAVE MODE MANAGEMENT
**********************************************************************/

/* simple structure to store a button state */
struct ButtonStatus
{
    /* ctor */
    ButtonStatus() : _updatetime(0), _changetime(0), _prevstate(0) , _status(false), _changed(false) {}
                
    /* return true if the button is currently pressed */
    inline bool on() const { return _status; }

    /* return true if the button is currently not pressed */
    inline bool off() const { return (!_status); }

    /* return true if the button was just pressed */
    inline bool pressed() const { return (_changed && _status); }

    /* return true if the button was just released */
    inline bool released() const { return (_changed && (!_status)); }

    /* return true is the button just changed state */
    inline bool changed() const { return _changed; }

    /* number of milliseconds since the last change of state was detected */
    inline uint32_t time_changed() const { return (uint32_t)_changetime;  }

    /* return the total time in microsecond that the previous state lasted */
    inline uint32_t time_previous_state() const { return _prevstate; }

    /* number of milliseconds since the last update */
    inline uint32_t time_update() const { return (uint32_t)_updatetime; }

    /* update the button status, return 0 if no cstatus cahnge and mask otherwise */
    inline uint32_t update(const uint32_t mask, const uint32_t code)
        {
        bool cstatus = ((code & mask) == 0); // true if currently pressed
        if (cstatus != _status) { _changed = true; _prevstate = _changetime;  _changetime = 0; }
        else { _changed = false; }
        _status = cstatus;
        _updatetime = 0;
        return (_changed ? mask : 0);
        }

private:

    elapsedMillis _updatetime;
    elapsedMillis _changetime;
    uint32_t      _prevstate;
    bool          _status;
    bool          _changed;
};


ButtonStatus B_up, B_down, B_left, B_right, B_select, B_a, B_b; // the buttons

const uint32_t POWERSAVE_MILLISECONDS = 60000;          // wait time before entering powersave mode: 30 seconds
const uint32_t EXIT_POWERSAVE_MILLISECONDS = 3600000;   // wait time before exiting powersave mode: 1 hour (to remind we are still on). 
volatile bool ispowersave = false;                      // true if we are on powersave (reduced luminosity)
volatile bool idleflag = true;                          // initially in idle state.
elapsedMillis idletime;                                 // time elapsed since the last action


/* return the number of milliseconds since no button have been pressed
return 0 if some button is curently pressed */
inline uint32_t idleTime() { return ((idleflag) ? ((uint32_t)idletime) : 0); }


/* update the status of all the buttons 
   return the button whose status changed since the last update
   (sets the bit corresponding to the the masksTFTWING_BUTTON_***
   in the return code) */
uint32_t updatebuttons()
    {
    uint32_t code = ss.readButtons(); 
    uint32_t ret = 0; 
    ret |= B_up.update(TFTWING_BUTTON_UP, code);
    ret |= B_down.update(TFTWING_BUTTON_DOWN, code);
    ret |= B_left.update(TFTWING_BUTTON_LEFT, code);
    ret |= B_right.update(TFTWING_BUTTON_RIGHT, code);
    ret |= B_select.update(TFTWING_BUTTON_SELECT, code);
    ret |= B_a.update(TFTWING_BUTTON_A, code);
    ret |= B_b.update(TFTWING_BUTTON_B, code);
    if (ret != 0)
        { // some button state changed
        if (B_up.off() && B_down.off() && B_left.off() && B_right.off() && B_select.off() && B_a.off() && B_b.off())
            { // starting idle_time
            idleflag = true; // set the flag
            idletime = 0;    // reset time counter
            }
        else
            {
            idleflag = false;
            }
        }
    if ((!ispowersave) && (idleTime() > POWERSAVE_MILLISECONDS))
        { // enter powersave mode
        ispowersave = true;
        ss.setBacklight(61000); // dim the backlight
        }
    else if (ispowersave)
        {
        if ((ret != 0)||(idleTime() > EXIT_POWERSAVE_MILLISECONDS))
            { // exit powersave mode
            ispowersave = false;
            ss.setBacklight(0); // max screen luminosity
            if (ret == 0)
                {
                idleflag = true; // set the flag
                idletime = 0;    // reset time counter
                }
            }
        }
    return ret; 
    }





/**********************************************************************
* START LOGO
**********************************************************************/


/* draw the standing wave on the logo */
void drawstandingwave(float x1, float x2, uint16_t color1, uint16_t color2, uint16_t color3, uint16_t bkcol)
    {
    const float step = 2 * PI / 200;
    const float ry = 20.0f;
    const int cy = 40.0f; 
    for (int u = 0; u < 4 * 160; u++)
        {
        const int i = (u >> 2);
        const double y1 = ry * fastcos(x1);
        const double y2 = ry * fastcos(x2);
        const double y3 = (y1 + y2)/4;
        const int j1 = (int)round(y1) + cy;
        const int j2 = (int)round(y2) + cy;
        const int j3 = (int)round(y3) + cy;
        if (logo(i,j1) == bkcol) obuf.drawPixel(i, j1, color1);
        if (logo(i,j2) == bkcol) obuf.drawPixel(i, j2, color2);
        if (logo(i, j3) == bkcol)
        {
            obuf.drawPixel(i, j3, color3);
            //obuf.drawLine(i, j3, i, 2*cy - j3, color3);
        }
        x1 += step;
        x2 += step;
        }
    }


/* display the start logo */ 
void startlogo()
    {
    memcpy(obuf.getBuffer(), teensyLevLogo, 160 * 80 * sizeof(uint16_t));           // blit the logo on the offscreen buffer
    const uint16_t bkcol = logo(0, 40);
    const float  stx = PI/25;
    float x1 = 0.0f;
    float x2 = 0.0f;
    for (int i = 0; i < 50; i++)
        {
        drawstandingwave(x1, x2, ST77XX_RED, ST77XX_GREEN, ST77XX_WHITE, bkcol);    // draw the waves
        blitBuffer();                                                               // display on the screen
        delay(50);                                                                  // wait 
        drawstandingwave(x1, x2, bkcol, bkcol, bkcol, bkcol);                       // erase the waves
        x1 += stx; 
        x2 -= stx;
        }
    }


/**********************************************************************
* SCREEN UPDATE VIA A PERIODIC INTERRUPT
**********************************************************************/

IntervalTimer dispint;  // display interupt object

const int8_t    MS_MAX_TICK = 20;           // max tick per second
volatile int8_t ms_max_tick = MS_MAX_TICK;  // current rate
volatile int8_t ms_tick;
volatile int8_t ms_redraw;

typedef void (*Dispfun)();          // typedef for a display function
volatile Dispfun dispfun = 0;       // pointer to the current display function


/* interrupt that redraw the main screen. */
FASTRUN void display_interrupt()
    {
    ms_tick++;
    if ((ms_redraw) || (ms_tick > ms_max_tick))
        {
        if (dispfun)
            {
            dispfun();              // call the function that draws on the offscreen buffer
            blitBuffer();           // blit the buffer on the main screen
            }
        ms_tick = 0;
        ms_redraw = 0;
        }
    }


/* start the display interrupt */
void begin_display()
    {
    dispint.priority(192);                                              // low priority
    dispint.begin(display_interrupt, 1000000.0 / MS_MAX_TICK);          // interrupt routine is called MS_MAX_TICK times per second
    }


/* Set the function to call for drawing the screen */
inline void set_display_fun(Dispfun dfun = 0)
    {
    dispfun = dfun;
    }


/* refresh the screen immediately */
void screen_refresh_once(bool wait_redrawn_finished = true)
    {
    ms_redraw = 1;
    if (wait_redrawn_finished)
        {
        while (ms_redraw) { yield(); }
        }
    }

/* set maximum screen refresh rate */
inline void screen_refresh_fast() { ms_max_tick = 1; }


/* set slow screen refresh rate */
inline void screen_refresh_slow() { ms_max_tick = MS_MAX_TICK; }





/**********************************************************************
* MAIN SCREEN ROUTINE
**********************************************************************/


volatile int8_t main_screen_selection   = 0;                                                                            // current position of the selection triangle on the main screen
volatile bool main_screen_highA         = false;                                                                        // button A hightlighted
volatile bool main_screen_highB         = false;                                                                        // button B hightlighted

void main_screen_up() { main_screen_selection--; if (main_screen_selection < 0) main_screen_selection = 2; }            // move the main screen selection triangle up

void main_screen_down() { main_screen_selection++; if (main_screen_selection >= 3) main_screen_selection = 0; }         // move the main screen selection triangle down

int8_t main_screen_select() { return main_screen_selection; }                                                           // current selection

void main_screen_highlightA(bool status) { main_screen_highA = status; }
bool main_screen_highlightA() { return main_screen_highA; }

void main_screen_highlightB(bool status) { main_screen_highB = status; }
bool main_screen_highlightB() { return main_screen_highB; }


/* draw the main screen on the offline buffer */
void draw_main_screen()
    {
    static char buffer[16];
    obuf.fillScreen(ST77XX_BLACK);
    obuf.setTextSize(1);
    obuf.setTextColor(ST77XX_WHITE);

    // draw move button
    if  (main_screen_highlightA())
        {
        obuf.drawRect(145, 0, 15, 35, ST77XX_YELLOW);
        obuf.fillRect(145, 0, 15, 35, ST77XX_YELLOW);
        obuf.setTextColor(ST77XX_BLUE);
        }
    else
        {
        obuf.drawRect(145, 0, 15, 35, ST77XX_BLUE);
        obuf.fillRect(145, 0, 15, 35, ST77XX_BLUE);
        obuf.setTextColor(ST77XX_WHITE);
        }
    obuf.setCursor(150, 0); obuf.print("m");
    obuf.setCursor(150, 8); obuf.print("o");
    obuf.setCursor(150, 17); obuf.print("v");
    obuf.setCursor(150, 24); obuf.print("e");

    // draw reset button
    if  (main_screen_highlightB())
        {
        obuf.drawRect(145, 80 - 42, 15, 42, ST77XX_YELLOW);
        obuf.fillRect(145, 80 - 42, 15, 42, ST77XX_YELLOW);
        obuf.setTextColor(ST77XX_BLUE);
        }
    else
        {
        obuf.drawRect(145, 80 - 42, 15, 42, ST77XX_BLUE);
        obuf.fillRect(145, 80 - 42, 15, 42, ST77XX_BLUE);
        obuf.setTextColor(ST77XX_WHITE);
        }
    obuf.setCursor(150, 39); obuf.print("r");
    obuf.setCursor(150, 47); obuf.print("e");
    obuf.setCursor(150, 46 + 8); obuf.print("s");
    obuf.setCursor(150, 46 + 16); obuf.print("e");
    obuf.setCursor(150, 47 + 24); obuf.print("t");

    obuf.setTextColor(ST77XX_WHITE);

    // display voltage info
    float dac = getDAC();
    float volt = voltage();
    int oy = 0;
    obuf.setCursor(9, 1 + oy);
    obuf.println("volt.");
    int32_t wv = (int32_t)(dac*(140 - 41));
    obuf.drawRect(41, 0 + oy, 140 - 41, 9, ST77XX_BLUE);
    obuf.fillRect(41, 0 + oy, wv, 9, ST77XX_BLUE);
    obuf.setCursor(72, 1 + oy);
    obuf.setTextColor(ST77XX_WHITE);
    obuf.print(tostring(buffer, volt, "V"));

    // display frequency info
    double freq = getFreq();
    oy = 15;
    obuf.setCursor(9, 1 + oy);
    obuf.println("freq.");
    wv = ((freq - MIN_FREQ)*(140 - 41))/(MAX_FREQ - MIN_FREQ);
    obuf.drawRect(41, 0 + oy, 140 - 41, 9, ST77XX_BLUE);
    obuf.fillRect(41, 0 + oy, wv, 9, ST77XX_BLUE);
    obuf.setCursor(65, 1 + oy);
    obuf.print(tostring(buffer, (int32_t)freq, "Hz"));

    // display phase info 
    const float d = getdiffphase();
    const int lx = 138;
    const int cy = 47;
    const float ry = 15;
    float x1 = 0;
    float x2 = d * 2 * PI + PI;
    const float step = 3 * 2 * PI / (4 * lx);
    for (int u = 8; u < 4 * lx; u++)
        {
        const int i = (u >> 2);
        const double y1 = ry * fastcos(x1);
        const double y2 = ry * fastcos(x2);
        const int j1 = (int)round(y1) + cy;
        const int j2 = (int)round(y2) + cy;
        if (j1 == j2) { obuf.drawPixel(i, j1, ST77XX_RED); }
        else
            {
            obuf.drawPixel(i, j1, ST77XX_YELLOW);
            obuf.drawPixel(i, j2, ST77XX_GREEN);
            }
        x1 += step;
        x2 += step;
        }
    obuf.setCursor(32, 69);
    obuf.print("phase:");
    obuf.setCursor(67, 69);
    float d2 = (d + 1.0)*180.0;
    if (d2 > 180) d2 -= 360;
    obuf.print(tostring(buffer, d2, ""));

    // draw the selection triangle
    int ty = (main_screen_selection == 0) ? 1 : ((main_screen_selection == 1) ? 16 : 70);
    int tx = (main_screen_selection == 2) ? 24 : 0;
    int tl = 7;
    while (tl > 0) { obuf.drawFastVLine(tx, ty, tl, ST77XX_RED); tl -= 2; tx++; ty++; }
    }


/* the main screen control loop */
void mainScreenLoop()
    {
    main_screen_highlightA(false);
    main_screen_highlightB(false);

    set_display_fun(draw_main_screen);

    screen_refresh_slow();
    screen_refresh_once();

    while (1)
        {
        yield();         // keep usb serial responsive
        updatebuttons(); // update the buttons states

        if (B_down.released()) { main_screen_down(); screen_refresh_once(); continue; }
        if (B_up.released()) { main_screen_up(); screen_refresh_once(); continue; }

        if (B_a.pressed()) { main_screen_highlightA(true); screen_refresh_once(); continue; }
        if (B_a.released()) { return; } // exit to auto move screen

        if (B_b.pressed()) { main_screen_highlightB(true); screen_refresh_once(); continue; }
        if (B_b.released()) 
            { 
            screen_refresh_fast();
            main_screen_highlightB(false);
            setDAC();
            phaseReset();
            waitEndPhaseCommand();
            resetFreq();
            screen_refresh_slow();
            screen_refresh_once();
            continue; 
            }

        switch (main_screen_select())
            {
            case 0:
                {
                if (B_left.on())  
                    { 
                    if (B_left.time_changed() > 1500) { decDAC(); decDAC(); decDAC(); }
                    if (B_left.time_changed() > 500) { decDAC(); decDAC(); decDAC(); }
                    decDAC();
                    screen_refresh_once(); 
                    delay(20);
                    continue;
                    }
                if (B_right.on()) 
                    { 
                    if (B_left.time_changed() > 1500) { incDAC(); incDAC(); incDAC(); }
                    if (B_left.time_changed() > 500) { incDAC(); incDAC(); incDAC(); }
                    incDAC();
                    screen_refresh_once(); 
                    delay(20);
                    continue;
                    }
                if (B_select.released()) { setDAC(); screen_refresh_once(); continue; }
                break;
                }
            case 1:
                {
                if (B_left.on())  
                    { 
                    if (B_left.time_changed() > 1500) { decFreq(); decFreq(); decFreq(); decFreq(); decFreq(); }
                    if (B_left.time_changed() > 500) { decFreq(); decFreq(); decFreq(); }
                    decFreq();
                    screen_refresh_once(); 
                    delay(50);
                    continue;
                    }
                if (B_right.on()) 
                    { 
                    if (B_left.time_changed() > 1500) { incFreq();  incFreq(); incFreq(); incFreq(); incFreq(); }
                    if (B_left.time_changed() > 500) { incFreq(); incFreq(); incFreq(); }
                    incFreq();
                    screen_refresh_once(); 
                    delay(50);
                    continue;
                    }
                if (B_select.released()) { resetFreq(); screen_refresh_once(); continue; }
                break;
                }
            case 2:
                {
                if (B_left.pressed()) { screen_refresh_fast(); singlePhaseCommand(700, -1, DOWN); }
                if (B_right.pressed()) { screen_refresh_fast(); singlePhaseCommand(700, -1, UP); }
                if (B_left.released() || B_right.released()) { cancelPhaseCommand(); screen_refresh_slow(); screen_refresh_once(); }
                if (B_select.released())
                    {
                    if (B_select.time_previous_state() > 500)
                        { // long press
                        screen_refresh_fast();
                        phaseReset();
                        waitEndPhaseCommand();  // wait until operation complete
                        screen_refresh_slow();
                        screen_refresh_once();
                        }
                    else
                        { // short press
                        screen_refresh_fast();
                        phaseAlignNext();
                        waitEndPhaseCommand();  // wait until operation complete
                        screen_refresh_slow();
                        screen_refresh_once();
                        }
                    }
                break;
                }
            }
        }
    }







/**********************************************************************
* MOVE OBJECTS
**********************************************************************/


/* Base class for a move*/
class BaseMove
    {

    public:
        
        BaseMove(const char * name, int lx)     
            {
            strcpy(_name, name);
            _lx = lx;
            }
        
        ~BaseMove()     {}

        /* name of move */
        const char * name() { return _name; };

        /* number of pixels for width of name()*/
        int name_size() { return _lx; }

        /* Return the first command */
        virtual PhaseCommand * first_com(); 

    protected:

        char _name[256];
        int _lx;
    };



/* no move */
class NoneMove : public BaseMove
{
public:

    NoneMove() : BaseMove("none", 58)
    {
    }

    virtual PhaseCommand * first_com() override
    {
        return nullptr;
    }

};


/* sinus move */
class SinusMove : public BaseMove
    {
    public:

        SinusMove() : BaseMove("sinus", 53)
            {
            _com[0].count = 1000;
            _com[0].dir = UP;
            _com[0].next = &_com[1];
            _com[0].rate = 100;

            _com[1].count = 500;
            _com[1].dir = UP;
            _com[1].next = &_com[2];
            _com[1].rate = 200;

            _com[2].count = 250;
            _com[2].dir = UP;
            _com[2].next = &_com[3];
            _com[2].rate = 400;

            _com[3].count = 100;
            _com[3].dir = UP;
            _com[3].next = &_com[4];
            _com[3].rate = 600;

            _com[4].count = 100;
            _com[4].dir = DOWN;
            _com[4].next = &_com[5];
            _com[4].rate = 600;

            _com[5].count = 250;
            _com[5].dir = DOWN;
            _com[5].next = &_com[6];
            _com[5].rate = 400;

            _com[6].count = 500;
            _com[6].dir = DOWN;
            _com[6].next = &_com[7];
            _com[6].rate = 200;

            _com[7].count = 1000;
            _com[7].dir = DOWN;
            _com[7].next = &_com[8];
            _com[7].rate = 100;


            _com[8].count = 1000;
            _com[8].dir = DOWN;
            _com[8].next = &_com[9];
            _com[8].rate = 100;

            _com[9].count = 500;
            _com[9].dir = DOWN;
            _com[9].next = &_com[10];
            _com[9].rate = 200;

            _com[10].count = 250;
            _com[10].dir = DOWN;
            _com[10].next = &_com[11];
            _com[10].rate = 400;

            _com[11].count = 100;
            _com[11].dir = DOWN;
            _com[11].next = &_com[12];
            _com[11].rate = 600;

            _com[12].count = 100;
            _com[12].dir = UP;
            _com[12].next = &_com[13];
            _com[12].rate = 600;

            _com[13].count = 250;
            _com[13].dir = UP;
            _com[13].next = &_com[14];
            _com[13].rate = 400;

            _com[14].count = 500;
            _com[14].dir = UP;
            _com[14].next = &_com[15];
            _com[14].rate = 200;

            _com[15].count = 1000;
            _com[15].dir = UP;
            _com[15].next = &_com[0];
            _com[15].rate = 100;
            }

        virtual PhaseCommand * first_com() override
            {
            return _com;
            }

    private:

        PhaseCommand _com[16];
    };



/* square move */
class SquareMove : public BaseMove
{
public:

    SquareMove() : BaseMove("square", 49)
        {
        _com[0].count = 2000;
        _com[0].dir = UP;
        _com[0].next = &_com[1];
        _com[0].rate = 200;

        _com[1].count = 4000;
        _com[1].dir = DOWN;
        _com[1].next = &_com[2];
        _com[1].rate = 200;

        _com[2].count = 2000;
        _com[2].dir = UP;
        _com[2].next = &_com[0];
        _com[2].rate = 200;
        }

    virtual PhaseCommand * first_com() override
    {
        return _com;
    }

private:

    PhaseCommand _com[4];
};



/* square move */
class FallMove : public BaseMove
{
public:

    FallMove() : BaseMove("fall", 58)
        {
        _com[0].count = 2000;
        _com[0].dir = UP;
        _com[0].next = &_com[1];
        _com[0].rate = 1000;

        _com[1].count = 4000;
        _com[1].dir = DOWN;
        _com[1].next = &_com[2];
        _com[1].rate = 10;

        _com[2].count = 2000;
        _com[2].dir = UP;
        _com[2].next = &_com[0];
        _com[2].rate = 1000;
        }

    virtual PhaseCommand * first_com() override
    {
        return _com;
    }

private:

    PhaseCommand _com[4];
};



// number of types of move
const int NB_TYPE_MOVE = 4; 

// instance for each move type
NoneMove    move_obj_none;
SinusMove   move_obj_sinus; 
SquareMove  move_obj_square;
FallMove    move_obj_fall;

// arry of pointer toward move instances
BaseMove * listMoveObj[NB_TYPE_MOVE] = { &move_obj_none,  &move_obj_square, &move_obj_sinus, &move_obj_fall };

// current cursor position for move type
int cursor_move_type = 0;

void move_next_type() { cursor_move_type++; if (cursor_move_type >= NB_TYPE_MOVE) cursor_move_type = 0;  }      // next move type
void move_prev_type() { cursor_move_type--; if (cursor_move_type < 0) cursor_move_type = NB_TYPE_MOVE - 1; }    // previous move type
void move_reset_type() { cursor_move_type = 0; }                                                                // reset to no move
BaseMove * get_move_type() { return listMoveObj[cursor_move_type]; }                                            // return a pointer to the current move object. 




/**********************************************************************
* MOVE SCREEN ROUTINE
**********************************************************************/


volatile int8_t move_screen_selection = 0;                                                                              // current position of the selection triangle on the main screen
void move_screen_up() { move_screen_selection--;   if (move_screen_selection < 0) move_screen_selection = 3; }          // move the main screen selection triangle up
void move_screen_down() { move_screen_selection++; if (move_screen_selection > 3) move_screen_selection = 0; }          // move the main screen selection triangle down
int8_t move_screen_select() { return move_screen_selection; }                                                           // current selection


volatile bool move_screen_highB = false;                                                                                // button B hightlighted
void move_screen_highlightB(bool status) { move_screen_highB = status; }
bool move_screen_highlightB() { return move_screen_highB; }

volatile int cursor_move_scale = 0;          

volatile int cursor_move_speed = 0;         

volatile int cursor_move_off = 0;            



volatile    float saved_scale = 1.0;
volatile    int ind_scale = 0;
volatile    float saved_rate = 1.0;
volatile    int ind_rate = 0;


const int MIN_IND_SCALE = -30; 
const int MAX_IND_SCALE = 30;

const int MIN_IND_RATE = -30;
const int MAX_IND_RATE = 30;


    /* draw the main screen on the offline buffer */
    void draw_move_screen2()
    {
        // init
        static char buffer[16];
        obuf.fillScreen(ST77XX_BLACK);
        obuf.setTextSize(1);
        obuf.setTextColor(ST77XX_WHITE);

        // draw the moving ball 
        int pp = 12 + (119*(current_position - min_position)) / (max_position - min_position);
        obuf.fillCircle(pp, 71, 5, ST77XX_RED);

        // draw the selection triangle
        int ty = (move_screen_selection == 0) ? 4 :
            (move_screen_selection == 1) ? 21 :
            (move_screen_selection == 2) ? 36 : 51;
        int tx = 0;
        int tl = 7;
        while (tl > 0) { obuf.drawFastVLine(tx, ty, tl, ST77XX_RED); tl -= 2; tx++; ty++; }


        // draw move button
        if (main_screen_highlightA())
            {
            obuf.drawRect(145, 0, 15, 35, ST77XX_YELLOW);
            obuf.fillRect(145, 0, 15, 35, ST77XX_YELLOW);
            obuf.setTextColor(ST77XX_BLUE);
            }
        else
            {
            obuf.drawRect(145, 0, 15, 35, ST77XX_BLUE);
            obuf.fillRect(145, 0, 15, 35, ST77XX_BLUE);
            obuf.setTextColor(ST77XX_WHITE);
            }
        obuf.setCursor(150, 0); obuf.print("m");
        obuf.setCursor(150, 8); obuf.print("o");
        obuf.setCursor(150, 17); obuf.print("v");
        obuf.setCursor(150, 24); obuf.print("e");


        // draw reset button
        if (move_screen_highlightB())
            {
            obuf.drawRect(145, 80 - 42, 15, 42, ST77XX_YELLOW);
            obuf.fillRect(145, 80 - 42, 15, 42, ST77XX_YELLOW);
            obuf.setTextColor(ST77XX_BLUE);
            }
        else
            {
            obuf.drawRect(145, 80 - 42, 15, 42, ST77XX_BLUE);
            obuf.fillRect(145, 80 - 42, 15, 42, ST77XX_BLUE);
            obuf.setTextColor(ST77XX_WHITE);
            }
        obuf.setCursor(150, 39); obuf.print("a");
        obuf.setCursor(150, 47); obuf.print("l");
        obuf.setCursor(150, 46 + 8); obuf.print("i");
        obuf.setCursor(150, 46 + 16); obuf.print("g");
        obuf.setCursor(150, 47 + 24); obuf.print("n");
        obuf.setTextColor(ST77XX_WHITE);

        // draw the type
        int oy = 3;
        obuf.setCursor(9, 1 + oy);
        obuf.println("type");
        BaseMove * mt = get_move_type();
        obuf.setCursor(43 + mt->name_size()/2, oy);
        obuf.setTextColor(ST77XX_YELLOW);
        obuf.print(mt->name());

        // draw the scale       
        int32_t wv = (int32_t)(((ind_scale - MIN_IND_SCALE)*(137 - 43))/(MAX_IND_SCALE - MIN_IND_SCALE));
        oy = 20;
        obuf.setTextColor(ST77XX_WHITE);
        obuf.setCursor(9, 1 + oy);
        obuf.println("scale");
        obuf.drawRect(43, 0 + oy, 137 - 43, 9, ST77XX_BLUE);
        obuf.fillRect(43, 0 + oy, wv, 9, ST77XX_BLUE);

        obuf.setCursor(75, oy);
        obuf.print(tostring(buffer, adjust_mult_scale , ""));


        // draw the speed
        wv = (int32_t)(((ind_rate - MIN_IND_RATE)*(137 - 43)) / (MAX_IND_RATE - MIN_IND_RATE));
        oy = 35;
        obuf.setCursor(9, 1 + oy);
        obuf.println("speed");
        obuf.drawRect(43, 0 + oy, 137 - 43, 9, ST77XX_BLUE);
        obuf.fillRect(43, 0 + oy, wv, 9, ST77XX_BLUE);

        obuf.setCursor(75, oy);
        obuf.print(tostring(buffer, 1.0f/adjust_rate_scale, ""));

        // draw the offset
        wv = adjust_offset; 
        bool inv = false; 
        if (wv < 0) { inv = true; wv = -wv; }
        wv = wv >> 5; 
        if (wv > 47) wv = 47;
        oy = 50;
        obuf.setCursor(9, 1 + oy);
        obuf.println(" off.");
        int cc = 43 + 47; 
        obuf.fillRect(cc-2, 0 + oy, 3, 9, ST77XX_BLUE);
        if (inv) obuf.fillRect(cc - wv, 0 + oy, wv, 9, ST77XX_BLUE); else obuf.fillRect(cc, 0 + oy, wv, 9, ST77XX_BLUE);
    }






void moveScreenLoop()
{
    main_screen_highlightA(true);
    move_screen_highlightB(false);

    move_screen_selection = 0;
    resetMinMaxPos(); 

    set_display_fun(draw_move_screen2);
    screen_refresh_once();
    screen_refresh_fast();


    // restore last move command with scale and rate. 
    commandRate(saved_rate);
    commandScale(saved_scale);
    setPhaseCommand(get_move_type()->first_com());


    while (1)
    {
        yield();         // keep usb serial responsive
        updatebuttons(); // update the buttons states

        if (B_down.released()) { move_screen_down(); screen_refresh_once(); continue; }
        if (B_up.released()) { move_screen_up(); screen_refresh_once(); continue; }


        if (B_a.pressed()) { main_screen_highlightA(false);  screen_refresh_once(); continue; }
        if (B_a.released()) 
            { // reset default speed and scale, save and exit move menu
            saved_rate = getCommandRate();
            saved_scale = getCommandScale();
            cancelPhaseCommand();
            commandScale(1.0f);
            commandRate(1.0f);
            return;
            }


        if (B_b.pressed()) { move_screen_highlightB(true);  screen_refresh_once(); continue; }
        if (B_b.released())
            { // reset
            move_screen_highlightB(false);  
            resetMinMaxPos();
            screen_refresh_once(); 
            continue;
            }

        if (move_screen_selection == 0)
            {
            if (B_right.pressed())  { move_next_type();  setPhaseCommand(get_move_type()->first_com()); screen_refresh_once(); continue; }
            if (B_left.pressed())   { move_prev_type();  setPhaseCommand(get_move_type()->first_com()); screen_refresh_once(); continue; }
            if (B_select.pressed()) { move_reset_type(); setPhaseCommand(get_move_type()->first_com()); screen_refresh_once(); continue; }
            }

        if (move_screen_selection == 1)
            {
            if (B_left.on())        { if (ind_scale > MIN_IND_SCALE) { ind_scale--; commandScaleDec(); screen_refresh_once(); } delay(50); continue; }
            if (B_right.on())       { if (ind_scale < MAX_IND_SCALE) { ind_scale++;   commandScaleInc(); screen_refresh_once(); } delay(50); continue; }
            if (B_select.pressed()) { ind_scale = 0; commandScale(); screen_refresh_once(); continue; }
            }

        if (move_screen_selection == 2)
            {
            if (B_left.on())        { if (ind_rate > MIN_IND_RATE) { ind_rate--;    commandRateDec(); screen_refresh_once(); } delay(50); continue; }
            if (B_right.on())       { if (ind_rate < MAX_IND_RATE) { ind_rate++;    commandRateInc(); screen_refresh_once(); } delay(50); continue; }
            if (B_select.pressed()) { ind_rate = 0;  commandRate(); screen_refresh_once(); continue; }
            }

        if (move_screen_selection == 3)
            {
            if (B_left.on()) { commandOffsetDec(); screen_refresh_once(); delay(50); continue; }
            if (B_right.on()) { commandOffsetInc(); screen_refresh_once(); delay(50); continue; } 
            if (B_select.pressed()) { resetOffset(); screen_refresh_once(); continue; }
            }

    }
}




/**********************************************************************
* Main program
**********************************************************************/

int main()
    {
    initVoltage();          // start voltage control
    createTimer();          // start the FTM timer
    startDMA();             // start DMA transfert. 
    singlePhaseCommand(1000, 1, UP); // single command to activate the interrupt.
    initUIControl();        // initialize screen and inputs
    startlogo();            // display the start logo 
    begin_display();        // begin the screen display interrupt

    while (1)
        {
        mainScreenLoop();   // display the main screen
        moveScreenLoop();   // display the move screen
        }
    }

/* end of file */


