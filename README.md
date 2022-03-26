# STM32F1_GBF

This project is a frequency generator using the STM32 PMW feature.

You have the control over the frequency and the duty-cycle of the square signal.
Then the **timers are auto configured** to the best setup in order to fit the expected frequency and duty-cycle.

There are two ways we can use the generator:
- Connect two potentiometers to use a wide range of frequencies and variable duty-cycle on the fly (Default config)
- Define fixed frequency and fixed duty-cycle (Uncomment `#define FIXED_FREQ` and `#define FIXED_DUTY` in main.c)

---
**Spec:**
- Duty-cycle is between 0 and 100% by 1% steps.
- Min frequency is 1Hz.
- Max frequency is 720_000Hz, but it can go higher if you don't need the duty-cycle to be fully functional. However, this would require some code changes (allow `(arr < 100)` and max frequency should be 72_000_000Hz).

Min/max frequency can be setup to cleverly use your potentiometer range. (See `#define FREQ_MIN` and `#define FREQ_MAX` in main.c)  
Potentiometer end-zone can be defined to avoid unreachable values.  (See `#define POT_MIN` and `#define POT_MAX` in main.c)

---
**Hardware connection:**
- A0 - Frequency analog entry (0 to 3.3V)
- A1 - Duty-cycle analog entry (0 to 3.3V)
- A8, A9, A10, A11 - Timer 1 output
- B13, B14 - Timer 1 reverse output
- B10, B11 - Timer 2 output
- A6, A7 - Timer 3 output

All timers are set up the same way, but some differences or phase shifts can appear between timers.
Both Timer 2 and Timer 3 have two more channels available if 3 sets of 4 channels are needed.

Thanks for reading! 
