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
  E0, 3F, F4, P2, FA, **FA2**, CV, 2K, UA, E7, F0, 32, AR, 3E, SB, WD, EF, ...).
  **FA2** (CBS RAM Plus extended, Harmony/Melody: 7 banks of 4K, 256 bytes of
  RAM, and a 1K ARM header in the 32K variant) was added on 20 Aug 2026 after
  a report on the MiSTer forum: it is what *Star Castle Arcade* needs.
  **SB** (SUPERbank, 128K-256K) was already implemented but was only detected
  when a byte pattern happened to be present; 128K and 256K images now fall
  back to SB the way Stella does. Real titles covered by this: *Circus Convoy*
  and *Quest for Tikal* (128K) and *Rescue From Poseidon's Gate* (128K,
  detected automatically).
* **ARM-based cartridges**: CDF, CDFJ, CDFJ+ and DPC+, run by a Thumb core in
  the FPGA. Tested on a few dozen titles, including the 64 KB Elevator Agent
  and the 128 KB Turbo Arcade. All the original classics work as well.

* **Startup splash screen**: at power-on the core shows an "ATARI 2600 ARM"
  screen. It is dismissed the first time a game ROM is loaded, and it is
  generated for both the 15 kHz CRT and the HDMI scaler.

## Video options

* **Stabilize Video** (OSD entry, default `On`). **Leave it on `On`.** The core
  then emits a fixed NTSC-sized visible window (240 lines) instead of the one
  the game builds for itself with the VBLANK register, while the frame length
  stays exactly the one the game produces.

  It matters because some games use the VBLANK register as a *drawing* tool and
  raise it dozens of times inside one frame. *Star Castle Arcade* is one of
  them: with the entry on `Off` the MiSTer scaler cannot tell where the active
  area is, re-detects the video mode, and the picture doubles while the
  frequency overlay keeps coming back.

  Until 21 Aug 2026 the entry was a trade-off, because a few DPC+ titles (both
  *Umi* games, *Epic Adventure V2.2*, *Evil Magician Returns*) wobbled with it
  on and had to be run with it `Off`. That was **our** bug, not theirs: the
  stabilizer emitted **two VSYNCs per frame** whenever the frame length changed
  by 1 to 3 lines, producing 4-line frames with zero active lines. Games with a
  constant frame length — *Star Castle* among them — never hit that path, which
  is why the two groups wanted opposite settings. Fixed in `video_stabilize`
  (`rtl/TIA.sv`): one VSYNC per frame, the game's own. Verified on real
  hardware on about 15 titles.

## Controller notes

* **Two-button controllers.** The second fire button of a modern two-button pad
  is delivered on the port's analog pin, the way the real hardware does it, so
  games that read a second button get one. This is a difference from the
  original `Atari7800_MiSTer` core: *1942 VCS* responds to the second button
  here and not there, verified by running the same ROM on both cores on real
  hardware (21 Aug 2026). It is unlikely to be the only such title.
  The same line carries the QuadTari detection, so setting the **QuadTari**
  entry to `Off` disables the second button together with the adapter.
* **Two-button sticks on SNAC.** Verified on real hardware with a two-button
  joystick wired to the **SMS / SC-3000 / Amiga** standard, where fire B sits on
  **pin 9**. That pin reaches `INPT1` on port 1 and `INPT3` on port 2 — the same
  line two-button games read, and the same one the USB path uses — so such a
  stick works without any configuration. An **Atari 7800 gamepad is not
  compatible** for the second button: its first fire works as usual, the second
  does not reach that line.
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

## Credits

* The **MiSTer** project and the `Atari7800_MiSTer` core this one is derived
  from, together with everyone who worked on the original TIA, RIOT and
  cartridge code.
* **Stella** and **Gopher2600**, whose implementations were used throughout as
  independent references — mapper detection, CDF/CDFJ+/DPC+ register layouts
  and, more than once, as an oracle to settle a disagreement about what the
  hardware really does.
* **[Claude Code](https://claude.com/claude-code) (Max plan), used inside VS
  Code**, which wrote the code in this fork.

### On the use of AI

The work in this fork was done with an AI assistant, and it could not have been
done without one. That is worth stating plainly rather than leaving it to be
discovered.

What matters more is how it was kept honest. **Every number in this README was
measured**, either on the Verilator testbench or on real MiSTer hardware, and
several convincing theories died on the bench. The vertical wobble on the DPC+
titles is the clearest example: it was blamed on the games for weeks, then on
their frame length, then a "vertical cadence lock" was written to compensate
for it — and that lock was **rejected on hardware**, because it made the
picture roll. Only then did a probe on the raw TIA signals show the real cause,
four lines away: the stabilizer was emitting two VSYNCs per frame. The rejected
attempt is still in the history, on purpose.

Two objections usually come up.

*"We tried AI and the results were a disaster."* The difference is not the
model, it is the environment. In a chat window you paste a file in and get a
file out that nobody runs. Here the assistant had a shell: it compiled with
Quartus, ran regressions over 38 titles under Verilator, drove Gopher2600
headless as an oracle, and got told it was wrong by the bench — and by the
hardware — on a regular basis.

*"AI code is unmaintainable and unreadable."* Read the diff. Every non-obvious
change carries the reason next to it, including the measurements that justified
it and the attempts that failed; the negative results are written down as
carefully as the positive ones, because they are what stops the same wrong idea
from coming back in three weeks.

**A note from the repository owner:** about a dozen assistants were tried before
this one. The difference was not the model on its own — it was having it inside
the machine that compiles, with the ability to be proven wrong.
