#include <sys/cdefs.h>
#include "scheduler.h"
#include "midi.h"
#include "uart2midi.h"
#include "hardware/pio.h"
#include "floppy.pio.h"
#include "pico/printf.h"
#include <math.h>

#define HARDWARE_ALARM_NUM 4
#define MAX_NOTES_AT_ONCE MAX_NOTES_PER_CHANNEL
#define USE_CUSTOM_ALARM_POOL MAX_NOTES_AT_ONCE >= PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS

/**
 * The PIO state machine index to use for the pio program
 */
#define PIO_STATE_MACHINE 0

#define PIO_OUTPUT_START 6
#define PIO_SIDE_PIN 17

/**
 * Frequency lookup table
 */
double freq_lookup_table[128] = {8.1757989156, 8.6619572180, 9.1770239974, 9.7227182413, 10.3008611535, 10.9133822323,
                                 11.5623257097, 12.2498573744, 12.9782717994, 13.7500000000, 14.5676175474,
                                 15.4338531643, 16.3515978313, 17.3239144361, 18.3540479948, 19.4454364826,
                                 20.6017223071, 21.8267644646, 23.1246514195, 24.4997147489, 25.9565435987,
                                 27.5000000000, 29.1352350949, 30.8677063285, 32.7031956626, 34.6478288721,
                                 36.7080959897, 38.8908729653, 41.2034446141, 43.6535289291, 46.2493028390,
                                 48.9994294977, 51.9130871975, 55.0000000000, 58.2704701898, 61.7354126570,
                                 65.4063913251, 69.2956577442, 73.4161919794, 77.7817459305, 82.4068892282,
                                 87.3070578583, 92.4986056779, 97.9988589954, 103.8261743950, 110.0000000000,
                                 116.5409403795, 123.4708253140, 130.8127826503, 138.5913154884, 146.8323839587,
                                 155.5634918610, 164.8137784564, 174.6141157165, 184.9972113558, 195.9977179909,
                                 207.6523487900, 220.0000000000, 233.0818807590, 246.9416506281, 261.6255653006,
                                 277.1826309769, 293.6647679174, 311.1269837221, 329.6275569129, 349.2282314330,
                                 369.9944227116, 391.9954359817, 415.3046975799, 440.0000000000, 466.1637615181,
                                 493.8833012561, 523.2511306012, 554.3652619537, 587.3295358348, 622.2539674442,
                                 659.2551138257, 698.4564628660, 739.9888454233, 783.9908719635, 830.6093951599,
                                 880.0000000000, 932.3275230362, 987.7666025122, 1046.5022612024, 1108.7305239075,
                                 1174.6590716696, 1244.5079348883, 1318.5102276515, 1396.9129257320, 1479.9776908465,
                                 1567.9817439270, 1661.2187903198, 1760.0000000000, 1864.6550460724, 1975.5332050245,
                                 2093.0045224048, 2217.4610478150, 2349.3181433393, 2489.0158697766, 2637.0204553030,
                                 2793.8258514640, 2959.9553816931, 3135.9634878540, 3322.4375806396, 3520.0000000000,
                                 3729.3100921447, 3951.0664100490, 4186.0090448096, 4434.9220956300, 4698.6362866785,
                                 4978.0317395533, 5274.0409106059, 5587.6517029281, 5919.9107633862, 6271.9269757080,
                                 6644.8751612791, 7040.0000000000, 7458.6201842894, 7902.1328200980, 8372.0180896192,
                                 8869.8441912599, 9397.2725733570, 9956.0634791066, 10548.0818212118, 11175.3034058561,
                                 11839.8215267723, 12543.8539514160};

/**
 * The timer pool used in scheduling the square waves for the floppy drives
 */
alarm_pool_t *pool;


/**
 * The main floppy organ channel
 */
FloppyRegister channel;

/**
 * The pio interface to use for communication with floppy drives
 */
PIO pio;

/**
 * Find a free floppy drive that can play a note
 *
 * @return index of free drive in the notes-array of the channel
 */
int find_free_drive() {
    int i;
    for (i = 0; i < MAX_NOTES_PER_CHANNEL && channel.notes[i].midi_note > 0; i++);
    return i;
}

/**
 * Reset the internal drive state. This method assumes that the drives are already in neutral position, only the
 * internal state is reset.
 *
 * @param organRegister the internal state
 */
void reset_floppy_register(FloppyRegister *organRegister) {
    organRegister->channel_pressure = 127;
    organRegister->pitch = MIDI_PITCH_WHEEL_CENTERED;

    // set all notes to invalid midi note and reset motor states to 0. Note that only the internal state of the motors
    // gets reset, the motors aren't actually moved.
    for (int i = 0; i < MAX_NOTES_PER_CHANNEL; i++) {
        organRegister->notes[i].index = i;
        organRegister->notes[i].midi_note = -1;
        for (int j = 0; j < MAX_DRIVES_PER_NOTE; j++) organRegister->notes[i].motor_states[j] = 0;
        for (int j = 0; j < MAX_DRIVES_PER_NOTE; j++) organRegister->notes[i].motor_directions[j] = 0;
    }
}


void init_scheduler(void) {
    // create a custom alarm pool if we use more slots than available in the default one
#if USE_CUSTOM_ALARM_POOL
    pool = alarm_pool_create(HARDWARE_ALARM_NUM, MAX_NOTES_AT_ONCE);
    printf("scheduler: custom alarm pool initialized for hw alarm %d with %d queue positions\n",
           HARDWARE_ALARM_NUM, MAX_NOTES_AT_ONCE);
#else
    alarm_pool_init_default();
    pool = alarm_pool_get_default();
    printf("scheduler: use default alarm pool\n");
#endif
    // init pio interface
    pio = pio0;
    uint offset = pio_add_program(pio, &floppy_multiplexer_program);
    floppy_pio_program_init(pio, PIO_STATE_MACHINE, offset, PIO_OUTPUT_START, PIO_SIDE_PIN);
    printf("scheduler: pio initialized (output beginning with: pin%d, side pin: pin%d)\n", PIO_OUTPUT_START,
           PIO_SIDE_PIN);

    reset_floppy_register(&channel);
}

void destroy_scheduler(void) {
#if USE_CUSTOM_ALARM_POOL
    alarm_pool_destroy(pool);
#endif
}

/**
 * Callback for the scheduled alarm that will inform the PIO state machine to send a signal to a floppy drive
 *
 * @param id the triggering alarm
 * @param this the `Note` that is requesting the interrupt
 */
int64_t note_scheduling_callback(__attribute__((unused)) alarm_id_t id, void *this) {
    FloppyDriveState *state = (FloppyDriveState *) this;
    uint32_t multiplexed_command = 0;

    // multiplex index
    multiplexed_command |= state->index;

    // volume flags (e.g.: a value of 7 will result in 3 extra-drives being played (as 7 is 111 in binary)
    uint8_t extra_drives = (uint8_t) round(
            ((double) (MAX_DRIVES_PER_NOTE - 1) / (double) MIDI_FULL_VELOCITY) * (double) state->velocity);
    uint8_t volume_flags = (1 << extra_drives) - 1;
    multiplexed_command |= volume_flags << 3;

    // motor directions
    for (int i = 0; i < MAX_DRIVES_PER_NOTE; i++) {
        uint32_t direction = ((uint32_t) (state->motor_directions[i] & 0x1)) << (3 + MAX_DRIVES_PER_NOTE - 1 + i);
        multiplexed_command |= direction;

        // update active drives' states
        if (i <= extra_drives) {
            state->motor_states[i] += 1;

            // revert motor direction if end of track is reached
            if (state->motor_states[i] == MAX_MOTOR_TRACKS) {
                state->motor_directions[i] = 1 - state->motor_directions[i];
                state->motor_states[i] = 0;
            }
        }
    }

    floppy_pio_exec(pio, PIO_STATE_MACHINE, multiplexed_command);
    return state->inverse_frequency;
}

/**
 * Cancel a previously scheduled note
 *
 * @param midi_note the midi number for the note to cancel (0-127)
 * @param velocity the release velocity (0-127)
 */
void cancel_note(uint8_t midi_note, uint8_t velocity) {
    int index;
    for (index = 0; index < MAX_NOTES_PER_CHANNEL && channel.notes[index].midi_note != midi_note; index++);

    if (index < MAX_NOTES_PER_CHANNEL) {
        alarm_pool_cancel_alarm(pool, channel.notes[index].alarm_id);
        channel.notes[index].midi_note = -1;
    }
}

/**
 * Convert a midi note into a organ note and schedule it on a floppy drive
 *
 * @param midi_note the midi number for the note to play (0-127)
 * @param velocity the midi note-on velocity (0-127). A velocity of 0 will be considered equal to a call to `cancel_note`
 */
void schedule_note(uint8_t midi_note, uint8_t velocity) {
    // a midi Note-On with a velocity of 0 is considered equivalent to a Note-Off message.
    if (velocity == 0) {
        cancel_note(midi_note, velocity);
        return;
    }

    int free_drive = find_free_drive();
    if (free_drive == MAX_NOTES_PER_CHANNEL) {
        // can't schedule note. do nothing.
        // TODO: schedule in some kind of queue and play it once another one stops just like in monophonic mode
        return;
    }

    assert(midi_note > 0);
    channel.notes[free_drive].midi_note = (int8_t) midi_note;
    channel.notes[free_drive].inverse_frequency = (uint32_t) (1000000.0 / freq_lookup_table[midi_note]);
    channel.notes[free_drive].velocity = velocity;
    channel.notes[free_drive].alarm_id = alarm_pool_add_alarm_in_us(pool, channel.notes[free_drive].inverse_frequency,
                                                                    note_scheduling_callback,
                                                                    &channel.notes[free_drive], true);
}

/**
 * Periodically check for incoming midi messages and schedule them as floppy commands accordingly (or update the state
 * machine accordingly)
 */
_Noreturn void scheduler_loop(void) {
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    int status = 1;

    while (true) {
        // status led blinking
        gpio_put(LED_PIN, (status & 0xF0) > 0);
        status = (status + 1) % 256;

        MidiMessage *message = try_read_midi();

        if (message != NULL) {
            switch (message->status.nibbles.type) {
                case NOTE_OFF:
                    cancel_note(message->body.word.low, message->body.word.high);
                    break;
                case NOTE_ON:
                    schedule_note(message->body.word.low, message->body.word.high);
                    break;
                case AFTER_TOUCH:
                    break;
                case CONTROL_CHANGE:
                    break;
                case PROGRAM_CHANGE:
                    break;
                case CHANNEL_PRESSURE:
                    break;
                case PITCH_WHEEL:
                    break;
                case SYSTEM:
                    break;
            }
        }
    }
}