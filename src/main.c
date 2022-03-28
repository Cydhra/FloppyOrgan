#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include "uart2midi.h"
#include "hardware/irq.h"
#include "floppy.pio.h"
#include "scheduler.h"

#define MIDI_IN_TX 0
#define MIDI_IN_RX 1

void uart_start_note(uint midi_note) {
    uart_putc_raw(uart1, NOTE_ON << 4);
    uart_putc_raw(uart1, midi_note);
    uart_putc_raw(uart1, 127);
}

void uart_stop_note(uint midi_note) {
    uart_putc_raw(uart1, NOTE_OFF << 4);
    uart_putc_raw(uart1, midi_note);
    uart_putc_raw(uart1, 127);
}

/// A simulated midi input repeating a simple melody on the 0th channel. Outputs the midi messages on pin 4, which can
/// then be fed into the MIDI input set in `init_midi`
_Noreturn void midi_simulation() {
    // Set up our selected UART and pins
    uart_init(uart1, 31250);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    uart_set_irq_enables(uart1, true, false);

    unsigned char notes[27] = {60, 62, 64, 65, 67, 67, 69, 69, 69, 69, 67, 69, 69, 69, 69, 67, 65, 65, 65, 65, 64, 64,
                               62, 62, 62, 62, 60};

    unsigned int speeds[27] = {500, 500, 500, 500, 1000, 1000, 500, 500, 500, 500, 2000, 500, 500, 500, 500, 2000, 500,
                               500, 500, 500, 1000, 1000, 500, 500, 500, 500, 2000};


    int i = 0;
    uart_start_note(notes[0] - 12);
    sleep_ms(speeds[0]);
    while (true) {
        uart_stop_note(notes[i] - 12);
        sleep_ms(50);

        if (++i >= 27)
            i = 0;

        uart_start_note(notes[i] - 12);
        sleep_ms(speeds[i]);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(4000);

    init_midi(MIDI_IN_TX, MIDI_IN_RX);
    printf("midi0 initialized (tx: pin%d, rx: pin%d)\n", MIDI_IN_TX, MIDI_IN_RX);

    multicore_launch_core1(midi_simulation);

    // initialize floppy pio handler
    init_scheduler();
    scheduler_loop();
}
