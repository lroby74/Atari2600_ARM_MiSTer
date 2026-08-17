# Atari 2600 for MiSTer FPGA with full ARM CPU support

This is a cleaned, **Atari 2600 only** FPGA core for MiSTer (with ARM CPU support
added later), derived from the combined `Atari7800_MiSTer` repository.
Every Atari 7800 specific subsystem (MARIA graphics, POKEY audio, YM2151, 7800
cartridge, BIOS, XM expansion) has been removed.

## What works (2600 features retained)

* TIA video with HDMI and CRT (15 kHz) output paths (scanline / scandoubler /
  composite blending / palette loading).
* TIA audio with the standard MiSTer stereo mix option.
* USB and SNAC controllers: joystick, paddle, lightgun, trackball, keypad,
  driving, ST mouse, Amiga mouse, BoosterGrip, Robotron, SaveKey, QuadTari.
* Supercharger tape loading from the ADC (`Load Tape From ADC`).
* All standard 2600 mappers handled by `cart2600` / `banks2600` (F8, F6, FE,
  E0, 3F, F4, P2, FA, CV, 2K, UA, E7, F0, 32, AR, 3E, SB, WD, EF, ...).
* **ARM-based cartridges**: CDF, CDFJ, CDFJ+ and DPC+, run by a Thumb core in
  the FPGA. Tested on a few dozen titles, including the 64 KB Elevator Agent
  and the 128 KB Turbo Arcade. All the original classics work as well.

* **Startup splash screen**: at power-on the core shows an "ATARI 2600 ARM"
  screen. It is dismissed the first time a game ROM is loaded, and it is
  generated for both the 15 kHz CRT and the HDMI scaler.

## Controller notes

* **QuadTari** (OSD entry, default `On`). Games detect the adapter from the two
  *analog* pins of the port, not from the controller multiplexing: with the
  entry set to `Off` the port now looks exactly like a plain 2600 port, so the
  adapter is really invisible. Note that the second fire button of a two-button
  pad travels on that same line, so it is disabled together with the adapter.
* **QuadTari + SaveKey**. In the QuadTari standard player 2 sits on **port 2**,
  which a SaveKey occupies. When `Port2 Input` is set to `SaveKey` (or `None`)
  and QuadTari is on, the second pad is routed automatically to the second
  controller of port 1, so two players can share port 1 while the SaveKey keeps
  port 2. Nothing to configure. With a real joystick on port 2 the second pad
  stays there, where it belongs.
* The SaveKey is enabled **only** by setting `Port1/Port2 Input` to `SaveKey`.
  With the ports on `Auto` or `Joystick` the EEPROM is held in reset.

## Build

Open `Atari2600.qpf` with **Quartus Prime 17.0.2 Lite Edition** and compile.
The project uses the standard MiSTer framework in `sys/` (PLLs, HPS IO, video
mixer, SDRAM, etc.). `build_id.v` is generated at build time (a static copy is
included so the project opens standalone).

### Getting the same timing results

The `.qsf` is set up so that the compilation is **reproducible**:

```
set_global_assignment -name NUM_PARALLEL_PROCESSORS 1
set_global_assignment -name SEED 12
```

**Please leave `NUM_PARALLEL_PROCESSORS` at 1.** With `ALL` the fitter uses every
core of the machine and the result *depends on how many there are*: the same
tree with the same seed gave +0.079 and +0.224 ns on two different runs here.
With a fixed value the fit is deterministic, and the seed chosen in this repo is
worth something on your machine too. The price is compilation time (the fitter
takes about 15 minutes here instead of a few).

Expected results with the settings above, so you can tell at once whether
something went wrong on your side:

```
Worst-case setup slack, Slow 1100 mV 100C ....  +0.375 ns
Worst-case setup slack, Slow 1100 mV -40C ....  +0.147 ns
Critical warnings (332148) ...................  0
Errors .......................................  0
```

The tightest domain is `pll_hdmi`, which belongs to the MiSTer framework rather
than to this core. Do **not** try to gain margin by removing the HDMI output:
it was measured to make things *worse* (mean slack -0.383 ns with HDMI in
against -0.639 ns without, over 8 seeds) because the bottleneck is routing, not
area — with more free space the fitter spreads the logic out.

A different Quartus version will give different numbers, and no seed can
compensate for that. Also make sure `db/`, `incremental_db/` and `output_files/`
are not present in the tree, otherwise the first compilation starts from
somebody else's placement.

## Notes / known limitations

The two bugs that were listed here have been **fixed** and verified on real
hardware:

1. **Lode Runner Demo** (sprite placement) and **Spiders Arcade** (the first
   yellow segment of the shot appearing at the old ship position) had the same
   root cause: the 6507 read the Display Data words of the COMMSTREAM response
   while the ARM was still rewriting them. The fix holds the 6507 on the
   `DSPTR` write, outside the visible kernel, and costs about 3 per mille of a
   frame. Corruptions went 1160 -> 0 on Spiders and 299 -> 1 on Lode Runner,
   with 37 of 38 titles bit-identical in the video regression.

Still open, if anybody wants to help:

* **Elevator Agent** does not fit its ARM run inside the VBlank window. Measured:
  it needs about 1.4x the window even when the run starts at the very beginning
  (95,403 clk_vid needed against 69,311 available), so it is not a matter of
  clock frequency — the ARM would have to do less work, not run faster.
