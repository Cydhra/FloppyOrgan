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

    unsigned char notes[32] = {68, 60, 63, 65,68, 60, 63, 65,68, 60, 63, 65,68, 60, 63, 65,
                               68, 60, 64, 65,68, 60, 64, 65,68, 60, 64, 65,68, 60, 64, 65,};

    unsigned int speeds[32] = {500, 500, 250, 250,500, 500, 250, 250,500, 500, 250, 250,500, 500, 250, 250,
                               500, 500, 250, 250,500, 500, 250, 250,500, 500, 250, 250,500, 500, 250, 250,};

    static int OFFSET = 0;

    int i = 1;
    uart_start_note(notes[0]- OFFSET);
    uart_start_note(60- OFFSET);
    uart_start_note(55- OFFSET);
    uart_start_note(48- OFFSET);

    sleep_ms(speeds[0]);
    while (true) {
        uart_stop_note(notes[i - 1]- OFFSET);
        if (i % 4 == 0) {
            uart_stop_note(60- OFFSET);
            uart_stop_note(55- OFFSET);
            uart_stop_note(48- OFFSET);
        }
        sleep_ms(50);

        if (i == 32) i = 0;

        uart_start_note(notes[i]- OFFSET);
        if (i % 4 == 0) {
            uart_start_note(60- OFFSET);
            uart_start_note(55- OFFSET);
            uart_start_note(48- OFFSET);
        }
        sleep_ms(speeds[i]);
        i++;
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
