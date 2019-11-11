// Requires Adafruit_ASFcore library!

// Be careful to use a platform-specific conditional include to only make the
// code visible for the appropriate platform.  Arduino will try to compile and
// link all .cpp files regardless of platform.
#if defined(ARDUINO_ARCH_SAMD)

#include <sam.h>
#include "WatchdogSAMD.h"

int WatchdogSAMD::enable(int maxPeriodMS, bool isForSleep) {
    // Enable the watchdog with a period up to the specified max period in
    // milliseconds.

    // Review the watchdog section from the SAMD21 datasheet section 18:
    // http://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf

    int     cycles;
    uint8_t bits;

    if(!_initialized) _initialize_wdt();

#if defined(__SAMD51__)
    WDT->CTRLA.reg = 0; // Disable watchdog for config
    while(WDT->SYNCBUSY.reg);
#else
    WDT->CTRL.reg = 0; // Disable watchdog for config
    while(WDT->STATUS.bit.SYNCBUSY);
#endif

    // You'll see some occasional conversion here compensating between
    // milliseconds (1000 Hz) and WDT clock cycles (~1024 Hz).  The low-
    // power oscillator used by the WDT ostensibly runs at 32,768 Hz with
    // a 1:32 prescale, thus 1024 Hz, though probably not super precise.

    if((maxPeriodMS >= 16000) || !maxPeriodMS) {
        cycles = 16384;
        bits   = 0xB;
    } else {
        cycles = (maxPeriodMS * 1024L + 500) / 1000; // ms -> WDT cycles
        cycles = cycles >> 2;   // min cycles is 8
        bits = 0;
        // Cycle choices are in powers of 2
        while( cycles = cycles>>1){
            ++bits;
        }
        cycles = 8<<bits;
    }

    // Watchdog timer on SAMD is a slightly different animal than on AVR.
    // On AVR, the WTD timeout is configured in one register and then an
    // interrupt can optionally be enabled to handle the timeout in code
    // (as in waking from sleep) vs resetting the chip.  Easy.
    // On SAMD, when the WDT fires, that's it, the chip's getting reset.
    // Instead, it has an "early warning interrupt" with a different set
    // interval prior to the reset.  For equivalent behavior to the AVR
    // library, this requires a slightly different configuration depending
    // whether we're coming from the sleep() function (which needs the
    // interrupt), or just enable() (no interrupt, we want the chip reset
    // unless the WDT is cleared first).  In the sleep case, 'windowed'
    // mode is used in order to allow access to the longest available
    // sleep interval (about 16 sec); the WDT 'period' (when a reset
    // occurs) follows this and is always just set to the max, since the
    // interrupt will trigger first.  In the enable case, windowed mode
    // is not used, the WDT period is set and that's that.
    // The 'isForSleep' argument determines which behavior is used;
    // this isn't present in the AVR code, just here.  It defaults to
    // 'false' so existing Arduino code works as normal, while the sleep()
    // function (later in this file) explicitly passes 'true' to get the
    // alternate behavior.

#if defined(__SAMD51__)
    if(isForSleep) {
        WDT->INTFLAG.bit.EW      = 1;    // Clear interrupt flag
        WDT->INTENSET.bit.EW     = 1;    // Enable early warning interrupt
        WDT->CONFIG.bit.PER      = 0xB;  // Period = max
        WDT->CONFIG.bit.WINDOW   = bits; // Set time of interrupt
        WDT->EWCTRL.bit.EWOFFSET = 0x0;  // Early warning offset
        WDT->CTRLA.bit.WEN       = 1;    // Enable window mode
        while(WDT->SYNCBUSY.reg);        // Sync CTRL write
    } else {
        WDT->INTENCLR.bit.EW     = 1;    // Disable early warning interrupt
        WDT->CONFIG.bit.PER      = bits; // Set period for chip reset
        WDT->CTRLA.bit.WEN       = 0;    // Disable window mode
        while(WDT->SYNCBUSY.reg);        // Sync CTRL write
    }

    reset();                             // Clear watchdog interval
    WDT->CTRLA.bit.ENABLE = 1;           // Start watchdog now!
    while(WDT->SYNCBUSY.reg);
#else
    if(isForSleep) {
        WDT->INTENSET.bit.EW   = 1;      // Enable early warning interrupt
        WDT->CONFIG.bit.PER    = 0xB;    // Period = max
        WDT->CONFIG.bit.WINDOW = bits;   // Set time of interrupt
        WDT->CTRL.bit.WEN      = 1;      // Enable window mode
        while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
    } else {
        WDT->INTENCLR.bit.EW   = 1;      // Disable early warning interrupt
        WDT->CONFIG.bit.PER    = bits;   // Set period for chip reset
        WDT->CTRL.bit.WEN      = 0;      // Disable window mode
        while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
    }

    reset();                             // Clear watchdog interval
    WDT->CTRL.bit.ENABLE = 1;            // Start watchdog now!
    while(WDT->STATUS.bit.SYNCBUSY);
#endif

    return (cycles * 1000L + 512) / 1024; // WDT cycles -> ms
}

void WatchdogSAMD::reset() {
    // Write the watchdog clear key value (0xA5) to the watchdog
    // clear register to clear the watchdog timer and reset it.
#if defined(__SAMD51__)
    while(WDT->SYNCBUSY.reg);
#else
    while(WDT->STATUS.bit.SYNCBUSY);
#endif
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}


uint8_t WatchdogSAMD::resetCause() {
#if defined(__SAMD51__)
  return RSTC->RCAUSE.reg;
#else
  return PM->RCAUSE.reg;
#endif
}

void WatchdogSAMD::disable() {
#if defined(__SAMD51__)
    WDT->CTRLA.bit.ENABLE = 0;
    while(WDT->SYNCBUSY.reg);
#else
    WDT->CTRL.bit.ENABLE = 0;
    while(WDT->STATUS.bit.SYNCBUSY);
#endif
}

void WDT_Handler(void) {
    // ISR for watchdog early warning, DO NOT RENAME!
#if defined(__SAMD51__)
    WDT->CTRLA.bit.ENABLE = 0;       // Disable watchdog
    while(WDT->SYNCBUSY.reg);
#else
    WDT->CTRL.bit.ENABLE = 0;        // Disable watchdog
    while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
#endif
    WDT->INTFLAG.bit.EW  = 1;        // Clear interrupt flag
    Serial.println(watchdog_warn_msg);        // Print the debug message
}

int WatchdogSAMD::sleep(int maxPeriodMS) {

    int actualPeriodMS = enable(maxPeriodMS, true); // true = for sleep

    // Enable standby sleep mode (deepest sleep) and activate.
    // Insights from Atmel ASF library.
#if (SAMD20 || SAMD21)
    // Don't fully power down flash when in sleep
    NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
#endif
#if defined(__SAMD51__)
    PM->SLEEPCFG.bit.SLEEPMODE = 0x4;         // Standby sleep mode
    while(PM->SLEEPCFG.bit.SLEEPMODE != 0x4); // Wait for it to take
#else
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#endif

    __DSB(); // Data sync to ensure outgoing memory accesses complete
    __WFI(); // Wait for interrupt (places device in sleep mode)

    // Code resumes here on wake (WDT early warning interrupt).
    // Bug: the return value assumes the WDT has run its course;
    // incorrect if the device woke due to an external interrupt.
    // Without an external RTC there's no way to provide a correct
    // sleep period in the latter case...but at the very least,
    // might indicate said condition occurred by returning 0 instead
    // (assuming we can pin down which interrupt caused the wake).

    return actualPeriodMS;
}

void WatchdogSAMD::warn( const String &msg, int timeout ){
    watchdog_warn_msg = msg;
    enable(timeout, true); // true = use interrupt, not reset
}

void WatchdogSAMD::_initialize_wdt() {
    // One-time initialization of watchdog timer.
    // Insights from rickrlh and rbrucemtl in Arduino forum!

#if defined(__SAMD51__)
    // SAMD51 WDT uses OSCULP32k as input clock now
    // section: 20.5.3
    OSC32KCTRL->OSCULP32K.bit.EN1K  = 1; // Enable out 1K (for WDT)
    OSC32KCTRL->OSCULP32K.bit.EN32K = 0; // Disable out 32K

    // Enable WDT early-warning interrupt
    NVIC_DisableIRQ(WDT_IRQn);
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0); // Top priority
    NVIC_EnableIRQ(WDT_IRQn);

    while(WDT->SYNCBUSY.reg);
    
    USB->DEVICE.CTRLA.bit.ENABLE = 0;         // Disable the USB peripheral
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE);   // Wait for synchronization
    USB->DEVICE.CTRLA.bit.RUNSTDBY = 0;       // Deactivate run on standby
    USB->DEVICE.CTRLA.bit.ENABLE = 1;         // Enable the USB peripheral
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE);   // Wait for synchronization
#else
    // Generic clock generator 2, divisor = 32 (2^(DIV+1))
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
    // Enable clock generator 2 using low-power 32KHz oscillator.
    // With /32 divisor above, this yields 1024Hz(ish) clock.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                        GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_SRC_OSCULP32K |
                        GCLK_GENCTRL_DIVSEL;
    while(GCLK->STATUS.bit.SYNCBUSY);
    // WDT clock = clock gen 2
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                        GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN_GCLK2;

    // Enable WDT early-warning interrupt
    NVIC_DisableIRQ(WDT_IRQn);
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0); // Top priority
    NVIC_EnableIRQ(WDT_IRQn);
#endif

    _initialized = true;
}

#endif // defined(ARDUINO_ARCH_SAMD)
