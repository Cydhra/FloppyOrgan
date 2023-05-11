#ifndef FLOPPYORGAN_UART2MIDI_H
#define FLOPPYORGAN_UART2MIDI_H

#include "hardware/uart.h"
#include "midi.h"

#define MIDI_STATE_BUFFER_SIZE 16

/**
* A state machine that handles buffering and composing of MidiMessages from incoming UART data
*/
typedef struct MidiStateMachine {
    /** if true, all incoming data is ignored until a new non-realtime status byte is received */
    uint8_t ignore_current_message;

    /** Message that is currently being constructed */
    MidiMessage incomplete_message;

    /** how many bytes of the current message body have been received yet */
    uint8_t recv_bytes;

    /** ring buffer for received midi packets. */
    MidiMessage receive_buffer[MIDI_STATE_BUFFER_SIZE];

    /** ring buffer head */
    uint8_t head;

    /** ring buffer tail */
    uint8_t tail;
} MidiStateMachine;

/**
 * Initialize a midi device on top of a given uart device
 *
 * @param uart_instance the uart handle to use for the midi handling
 * @param tx_pin GPIO pin reserved for transmission
 * @param rx_pin GPIO pin reserved for receiving
 */
void init_midi(uint tx_pin, uint rx_pin);

/**
 * Try reading a midi message from a midi device
 *
 * @param midi midi device
 *
 * @return a reference to a new midi message from the device's buffer or NULLPTR if no new message is available.
 * This reference gets invalid, as soon as the buffer overflows.
 */
MidiMessage *try_read_midi(void);

#endif //FLOPPYORGAN_UART2MIDI_H