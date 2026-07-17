# Atari 2600 (MiSTer) - 2600-only build

This is a cleaned, **Atari 2600 only** FPGA core for MiSTer, derived from the
combined `Atari7800_MiSTer` repository. Every Atari 7800 specific subsystem
(MARIA graphics, POKEY audio, YM2151, 7800 cartridge, BIOS, XM expansion and
the high-score cartridge) has been removed. What remains is a self-contained
2600 core that reuses the original 2600 TIA / RIOT / 6502 / cartridge / video /
audio paths.

## What works (2600 features retained)

* TIA video with HDMI and CRT (15 kHz) output paths (scanline / scandoubler /
  composite blending / palette loading).
* Mono TIA audio (no stereo / multichannel, per spec).
* USB and SNAC controllers: joystick, paddle, lightgun, trackball, keypad,
  driving, ST mouse, Amiga mouse, BoosterGrip, Robotron, SaveKey, Quadtari.
* Supercharger tape loading from the ADC (`Load Tape From ADC`).
* All standard 2600 mappers handled by `cart2600` / `banks2600` (F8, F6, FE,
  E0, 3F, F4, P2, FA, CV, 2K, UA, E7, F0, 32, AR, 3E, SB, WD, EF, ...).
* **Startup splash screen**: at power-on the core shows an "ATARI 2600" logo
  (bitmap extracted from the supplied `rom.mif`, stored in
  `rtl/startup_logo.hex`) as an overlay. It is dismissed on the first user
  input (keyboard / joystick) or when a game ROM is loaded. The overlay reuses
  the TIA sync stream, so it is valid for both the 15 kHz CRT and the HDMI
  scaler. See `REPORT_STARTUP_SCREEN.txt` for implementation and verification
  details.

## Build

Open `Atari2600.qpf` with **Quartus Prime 17.0.2** and compile. The project
uses the standard MiSTer framework in `sys/` (PLLs, HPS IO, video mixer, SDRAM,
etc.) which is provided by the MiSTer build environment. `build_id.v` is
generated at build time (a static copy is included so the project opens
standalone).

## Notes / known limitations

See `CHANGELOG_2600_CLEANUP.txt` for the full list of modifications, what was
removed, what was kept, the difficulties encountered, and recommended next
steps. In particular:

* The TIA clock generator and the 2600 address decoder that originally lived
  inside MARIA were reproduced 1:1 from the core's own 2600-mode logic, so
  2600 timing/behaviour is preserved.
* Synthesis and on-hardware verification were **not** performed in the cleaning
  environment (no Quartus / board available). The project must be built and
  tested on real hardware before being declared fully functional.
* The OSD still contains a few legacy option slots (kept to preserve the
  `status[]` bit layout and avoid an unverifiable renumber); their 7800
  behaviours are forced to their 2600-correct values. See the changelog.
