#ifndef FLOPPYORGAN_MIDI_H
#define FLOPPYORGAN_MIDI_H

#include <stdint.h>
#include <stdbool.h>

/** Maximum velocity value assigned to any midi note */
#define MIDI_FULL_VELOCITY 127

/** Default value of a centered pitch wheel in a midi channel */
#define MIDI_PITCH_WHEEL_CENTERED 0x2000

typedef enum MidiMessageType {
    NOTE_OFF = 0x8,
    NOTE_ON = 0x9,
    AFTER_TOUCH = 0xA,
    CONTROL_CHANGE = 0xB,
    PROGRAM_CHANGE = 0xC,
    CHANNEL_PRESSURE = 0xD,
    PITCH_WHEEL = 0xE,
    SYSTEM = 0xF,
} MidiMessageType;

typedef enum MidiSystemMessageType {
    EXCLUSIVE,
    QUARTER_FRAME_MESSAGE,
    SONG_POSITION_POINTER,
    SONG_SELECT,
    TUNE_REQUEST,
    EXCLUSIVE_END,
    MIDI_CLOCK,
    MIDI_START,
    MIDI_CONTINUE,
    MIDI_STOP,
    RESET
} MidiSystemMessageType;

/**
 * Message body with two bytes
 */
typedef struct WordMessage {
    uint8_t low;
    uint8_t high;
} WordMessage;

/**
 * Message body with one byte
 */
typedef struct ByteMessage {
    uint8_t value;
} ByteMessage;

/**
 * Message body union (either one or two byte body). Warning: Accessing the two-byte body of a one-byte message will
 * yield a random value (and not zero)!
 */
typedef union MessageBody {
    WordMessage word;
    ByteMessage byte;
} MessageBody;

/**
 * A midi message as a tagged union of `MessageBody`. The status byte offers the high and low 4-bit nibbles as `type` and
 * `channel`.
 */
typedef struct MidiMessage {
    union {
        uint8_t value;
        struct {
            uint8_t channel: 4;
            MidiMessageType type: 4;
        } nibbles;
    } status;
    MessageBody body;
} MidiMessage;

/**
 * @return whether the given status code is a System Realtime message
 */
bool is_midi_realtime_message(uint8_t status);

/**
 * @return the 14 bit word of a midi pitch message
 */
uint16_t get_midi_word(WordMessage *message);

#endif //FLOPPYORGAN_MIDI_H