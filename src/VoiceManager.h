#pragma once
#include "Voice.h"

class VoiceManager
{
public:
  void Init(float sample_rate);

  void NoteOn(byte inChannel, byte inNote, byte inVelocity);
  void NoteOff(byte inChannel, byte inNote, byte inVelocity);

  void ProcessBlock(float **out, size_t size);
  void UpdateParamsFromHardware(const SynthHardware &hw);

private:
  Voice voice_;

  // LAST note priority stack
  byte current_note_ = 0;
  byte current_velo_ = 0;
  byte held_count_ = 0;
  byte top_ = 0;
  byte held_notes_values_[128];
  bool held_notes_tracker_[128];

  void PushNote(byte inNote);
  byte PopNote();
  void DeleteNote(byte inNote);
};
