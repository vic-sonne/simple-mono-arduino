# Simple Mono Synth
 A possible answer to "what's the best begginer synth?"

 Designed to be hands-on, knob per function, expandable, hackable.
  
 ## Features
 - Monophonic
 - East-coast / subtractive architecture
 - Single oscillator
 - Low pass filter
 - ADSR envelope
 - Modulator with character

## What it sounds like

🚧 WIP - coming soon

## Getting started

### Requirements

- [Electrosmith Daisy](https://daisy.audio/)
- [Arduino IDE](https://www.arduino.cc/en/software/) or [VS Code + PlatformIO](https://platformio.org/install/ide?install=vscode)
- Required Libraries
  - DaisyDuino
  - FortysevenEffect's MIDI Library

### Build and upload

> [!NOTE]
> If using the arduino IDE, make sure to follow [Daisy's setup guide](https://daisy.audio/tutorials/arduino-dev-env/)

```bash
git clone https://github.com/vic-sonne/simple-mono-arduino.git
```
1. Open `simple_mono_refacto_1.ino`
2. Select the correct board
3. Upload

## Hardware

🚧 WIP - BOM & Schematic coming soon

> [!NOTE]
> The core of the synth is the Daisy. The BOM and schematic will be illustrations of how you can
> create a board around it to hold the required pots and switches

## Oscillator Modes

The synth offers 3 different oscillator banks with their own set of waveforms and
modulations. Banks must be decided at the firmware level and require firmware upload if you wish to
change them. Each bank comes with a set of shapes that can be selected in real time.
The following table summarises the available combinations:

| Compiled oscillator bank | Bottom | Middle | Top |
|---|---|---|---|
| Bank 1 | PWM Square | WIP concept | WIP concept |
| Bank 2 | PWM Square | Analog-leaning oscillator | Digital-leaning oscillator |
| Bank 3 | PWM Square | Sawtooth with wavefolding and hard sync | Triangle-to-Saw-to-Notch wave with analog-style FM |

> [!TIP]
> If you'd like alternate options, feel free to open an issue or tweak the code yourself and open
> a pull request to make your ideas a part of the project!

## Contributing

Please read `CONTRIBUTING.md` before submitting changes.

## Author

Made by Victorien from Purple Noize, with love

