#include "DaisyDuino.h"
#include "SynthHardware.h"
#include "VoiceManager.h"
#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

SynthHardware g_hw;
VoiceManager g_vm;

static void AudioCallback(float **in, float **out, size_t size)
{
  // optional control-rate scheduling inside here, but at minimum:
  g_vm.ProcessBlock(out, size);
}

void handleNoteOn(byte ch, byte note, byte vel)
{
  if (vel == 0.f)
  {
    digitalWrite(LED_BUILTIN, 0);
    g_vm.NoteOff(ch, note, vel);
  }
  else
  {
    digitalWrite(LED_BUILTIN, 1);
    g_vm.NoteOn(ch, note, vel);
  }
}

void handleNoteOff(byte ch, byte note, byte vel)
{
  digitalWrite(LED_BUILTIN, 0);
  g_vm.NoteOff(ch, note, vel);
}

void setup()
{
  g_hw.Init();
  float sr = DAISY.get_samplerate();

  g_vm.Init(sr);

  pinMode(LED_BUILTIN, OUTPUT);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.begin(MIDI_CHANNEL_OMNI);

  DAISY.begin(AudioCallback);
}

void loop()
{
  MIDI.read();

  // control-rate hardware update
  static uint32_t last_ctrl = 0;
  uint32_t now = millis();
  if (now - last_ctrl > 2)
  {
    last_ctrl = now;
    g_hw.UpdateControls();
    g_vm.UpdateParamsFromHardware(g_hw);
  }
}
