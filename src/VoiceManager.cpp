#include "VoiceManager.h"

/*
 * ABSTRACTION / ENCAPSULATION MANAGEMENT
 */
void VoiceManager::Init(float sample_rate)
{
  voice_.Init(sample_rate);
}

void VoiceManager::ProcessBlock(float **out, size_t size)
{
  voice_.ProcessBlock(out, size);
}

void VoiceManager::UpdateParamsFromHardware(const SynthHardware &hw)
{
  voice_.UpdateParamsFromHardware(hw);
}

/**
 * ACTUAL VOICE MANAGEMENT
 * (handling of midi note priority and voice retrig)
 */
void VoiceManager::NoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  // Note Off can come in as Note On w/ 0 Velocity
  if (inVelocity == 0.f)
  {
    NoteOff(inChannel, inNote, inVelocity);
  }
  else
  {
    PushNote(inNote);
    current_velo_ = inVelocity;
    voice_.NoteOn(inChannel, inNote, current_velo_);
  }
}

void VoiceManager::NoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  if (current_note_ == inNote)
  {
    byte next_note = PopNote();
    if (held_count_ <= 0)
    {
      current_velo_ = 0;
      voice_.NoteOff(inChannel, inNote, current_velo_);
    }
    else
    {
      voice_.NoteOn(inChannel, next_note, current_velo_);
    }
  }
  else
  {
    DeleteNote(inNote);
  }
}

void VoiceManager::PushNote(byte inNote)
{
  if (held_notes_tracker_[inNote])
  {
    current_note_ = inNote;
    return;
  }
  current_note_ = inNote;
  held_notes_values_[top_] = inNote;
  held_notes_tracker_[inNote] = true;
  held_count_++;
  top_++;
}

byte VoiceManager::PopNote()
{
  held_count_--;
  held_notes_tracker_[current_note_] = false;
  if (held_count_ <= 0)
  {
    top_ = 0;
    return 0;
  }
  for (int i = top_ - 1; i >= 0; --i)
  {
    byte note = held_notes_values_[i];
    if (held_notes_tracker_[note])
    {
      current_note_ = note;
      top_ = i + 1;
      return note;
    }
  }
  held_count_ = 0;
  top_ = 0;
  return 0; // should never reach here...
}

void VoiceManager::DeleteNote(byte inNote)
{
  if (held_notes_tracker_[inNote])
  {
    held_notes_tracker_[inNote] = false;
    if (held_count_ > 0)
      held_count_--;
  }
}
