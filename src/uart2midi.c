#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <stdio.h>
#include <string.h>
#include "midi.h"
#include "uart2midi.h"

#define MIDI_BAUD_RATE 31250

/** Midi state machine for uart0 */
MidiStateMachine midi_state;

/**
 * Finalize the incomplete message by appending it to the midi message ring buffer, then reset the internal state to an
 * incomplete message (but keep the header, in case of Running Status).
 *
 * @param midi_state the midi state machine
 */
void finalize_message(void) {
    // invalid message. This indicates a critical error, but because we are in an interrupt, we will fail silently and
    // hope for a quick recovery
    if (midi_state.incomplete_message.status.nibbles.type == SYSTEM) {
        // debug message (usually you don't want this in an IRQ, but we are already in critical state)
        printf("invalid msg finalized!\n");
        return;
    }

    // append to buffer (overwriting the oldest message within the buffer, if necessary)
    // we do not care for errors here: we can't do anything but complain anyway, and checking for errors
    // risks false positives through race conditions
    memcpy(&midi_state.receive_buffer[midi_state.head], &midi_state.incomplete_message, sizeof(MidiMessage));
    midi_state.head = (midi_state.head + 1) % MIDI_STATE_BUFFER_SIZE;

    // reset state machine body to zero, and assume `Running Status`
    midi_state.recv_bytes = 0;
}

/**
 * Interrupt handler delegate for uart0/midi0
 */
void on_uart0_irq(void) {
    while (uart_is_readable(uart0)) {
        char byte = uart_getc(uart0);

        // check whether this is a status byte
        if (byte & 0x80) {
            midi_state.incomplete_message.status.value = (uint8_t) byte;

            // ignore all system messages. We do not define any exclusive ones, and we ignore all realtime messages,
            // including Active Sense. If it is not a realtime message, we ignore potential further data bytes until
            // we find another status code
            if (midi_state.incomplete_message.status.nibbles.type == SYSTEM) {
                if (!is_midi_realtime_message(midi_state.incomplete_message.status.value)) {
                    // ignore the body of the current non-realtime system message
                    midi_state.ignore_current_message = 1;
                } // else: keep current ignore-state
            } else {
                midi_state.ignore_current_message = 0;
                midi_state.recv_bytes = 0;
            }
        } else { // not a status byte
            if (midi_state.ignore_current_message) {
                return;
            }

            switch (midi_state.incomplete_message.status.nibbles.type) {
                case NOTE_OFF:
                case NOTE_ON:
                case AFTER_TOUCH:
                case PITCH_WHEEL:
                    if (!midi_state.recv_bytes++) {
                        midi_state.incomplete_message.body.word.low = byte;
                    } else {
                        midi_state.incomplete_message.body.word.high = byte;
                        finalize_message();
                    }
                    break;
                case CONTROL_CHANGE:
                case PROGRAM_CHANGE:
                case CHANNEL_PRESSURE:
                    midi_state.incomplete_message.body.byte.value = byte;
                    finalize_message();
                    break;
                default:
                    panic("unreachable");
            }
        }
    }
}

void init_midi(uint tx_pin, uint rx_pin) {
    // Set up our selected UART and pins
    uart_init(uart0, MIDI_BAUD_RATE);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    // setup midi parameters for uart
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    irq_set_exclusive_handler(UART0_IRQ, on_uart0_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    midi_state.ignore_current_message = true; // Running Status is invalid yet, so we ignore up until the first status byte
    midi_state.recv_bytes = 0;
    midi_state.incomplete_message.status.nibbles.type = SYSTEM; // invalidate structure by setting invalid value to the tag
}

MidiMessage *try_read_midi(void) {
    // potential race condition on ring buffer overflow
    if (midi_state.tail != midi_state.head) {
        MidiMessage *next = &midi_state.receive_buffer[midi_state.tail++];
        midi_state.tail %= MIDI_STATE_BUFFER_SIZE;
        return next;
    }

    return NULL;
}