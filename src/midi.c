#include "midi.h"

bool is_midi_realtime_message(uint8_t status) {
    uint8_t channel = status & 0xF;
    return (status >> 4) == SYSTEM && channel >= MIDI_CLOCK;
}

uint16_t get_midi_word(WordMessage *const message) {
    uint16_t word = message->high;
    word <<= 7;
    word |= message->low;
    return word;
}