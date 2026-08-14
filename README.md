# Atari 2600 for MiSTer FPGA with full ARM CPU support

This is a cleaned, **Atari 2600 only (with ARM CPU support added later) FPGA core for 
MiSTer, derived from the combined `Atari7800_MiSTer` repository.
Every Atari 7800 specific subsystem (MARIA graphics, POKEY audio, YM2151, 7800 cartridge,
BIOS, XM expansion) has been removed. 

## What works (2600 features retained)

* TIA video with HDMI and CRT (15 kHz) output paths (scanline / scandoubler /
  composite blending / palette loading).
* Mono TIA audio (no stereo / multichannel, per spec).
* USB and SNAC controllers: joystick, paddle, lightgun, trackball, keypad,
  driving, ST mouse, Amiga mouse, BoosterGrip, Robotron, SaveKey, Quadtari.
* Supercharger tape loading from the ADC (`Load Tape From ADC`).
* All standard 2600 mappers handled by `cart2600` / `banks2600` (F8, F6, FE,
  E0, 3F, F4, P2, FA, CV, 2K, UA, E7, F0, 32, AR, 3E, SB, WD, EF, ...).

  The core has been tested on a few dozen titles that require my ARM CPU (including the 64KB Elevator Agent and the 128KB Turbo Arcade), and all except the two listed below work.
Of course, all the original classics also work perfectly.

*  **Startup splash screen**: at power-on the core shows an "ATARI 2600 ARM" logo (will be changed on next time)
 It is dismissed first time that a game ROM is loaded and it's valid for both the 15 kHz CRT and the HDMI
  scaler.

## Build

Open `Atari2600.qpf` with **Quartus Prime 17.0.2** and compile. The project
uses the standard MiSTer framework in `sys/` (PLLs, HPS IO, video mixer, SDRAM,
etc.) which is provided by the MiSTer build environment. `build_id.v` is
generated at build time (a static copy is included so the project opens
standalone). The compiling time is very long (about half hour on my PC)

## Notes / known limitations
Original Atari 2600 Paddle DB9 via Snac ADC is still not working, will be fixed on next release too  

Warning: Make sure you have the STABILIZE VIDEO option set to OFF otherwise some titles will have the screen flickering and it is not a problem with the game or the core but rather it is this option that is messing up.
