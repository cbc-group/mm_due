# mm_due
Arduino Due firmware compatible with existing Arduino32Hub adapter.

This repository is intended to be the replacement firmware for [Arduino32bitBoards device adapter](https://github.com/bonnom/Arduino32BitBoards), with finer internal DAC control for sheet-scan microscopes.

Differences compare to the original design:
- Use built-in DAC as sheet-scan galvo driver.
- Analog controls are repurposed as drive position and time step register.
- Direct port access for laser control.
- Maximum of 16 sequences.
- Trigger mode and blanking mode are redesigned to follow the mnemonics on Micro-Manager official site.
- _No timed triggered mode_ for now.

## Supported board
This firmware is designed for [Arduino Due](https://store.arduino.cc/usa/due) _only_. Though it is highly possible to expand it for any
ARM Cortex-M3 controllers.

## Installation
1. Clone or manually download the repository, and move contents in `libraries/` to your local library folder.
2. Compile and upload the `firmware.ino`. Done :)

## Operations
#### Step 1. Camera adjustment
In our case, "blanking" is edge-triggered instead of level-triggered. Therefore, for an Andor sCMOS, please configure the Aux Output to
`FIRE ALL` or `FIRE ANY`.

#### Step 2. Positive/Negative logic (optional)
From the official site
> The "blanking" feature of the device adapter allows to ensure that the sample is illuminated only when the camera is exposing.

If your camera trigger signal is active low, aka, negative TTL, please toggle the `Blank On` parameter to `High`. Toggle `Blanking Mode` will either enable or disable this mode.

#### Step 3-1. Live preview
Control the `Arduino32-Switch` state will directly manipulate bank PIOD, ranging from [D.0 to D.7 on the board](http://www.robgray.com/temp/Due-pinout-A4.png) (mapped to Due digital pins 25-28,14,15,29,11).

#### Step 3-2. Sequence mode
Sequence mode uses internal sequence buffer to decide which laser to enable.

Please enable the `Sequence Mode` option from the adapter, and execute the upload script in `scripts/` folder to setup desired sequences. If `Sequence Mode` is not enabled first, the script will fail with an exception message.

Use Multi-D. acquisition to configure total acquisition frames (in Time, Channel and Layers).
