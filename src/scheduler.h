#ifndef FLOPPYORGAN_SCHEDULER_H
#define FLOPPYORGAN_SCHEDULER_H

#include <sys/cdefs.h>
#include <stdint.h>
#include "pico/time.h"

//
// The scheduler transforms the incoming midi stream into an internal representation.
//

/**
 * how many notes can be played at once. This is currently set to 8 rather arbitrarily (a larger or additional demuxer
 * is required for more than 8 notes simultaneously).
 */
#define MAX_NOTES_PER_CHANNEL 8

/**
 * How many drives can be utilized for volume control of a single note at once
 */
#define MAX_DRIVES_PER_NOTE 4

/**
 * How many steps a motor in a floppy drive can perform before it reaches the end
 */
#define MAX_MOTOR_TRACKS 80

/**
 * Internal representation of a floppy drive (or set of drives actuated simultaneously)
 */
typedef struct FloppyDriveState {
    /** multiplex-index of the floppy drive */
    uint8_t index;

    /** midi representation of the note for identification when cancelling. Set to a negative value for invalid notes */
    int8_t midi_note;

    /** number of micro seconds between peaks of the note's square wave */
    uint32_t inverse_frequency;

    /** midi note pressure */
    uint8_t velocity;

    /** id of the scheduled alarm */
    alarm_id_t alarm_id;

    /**
     * state of the stepping motor per associated drive. Those states differ because different volumes access multiple
     * drives at once
     */
    uint8_t motor_states[MAX_DRIVES_PER_NOTE];

    /**
     * current direction the stepping motors are traveling in. Inverted if the respective `motor_state` reaches 80.
     */
    uint8_t motor_directions[MAX_DRIVES_PER_NOTE];
} FloppyDriveState;

/**
 * Internal representation of a midi channel. It contains the state machines for a fixed number of floppy drives
 */
typedef struct FloppyRegister {
    /** an array of all notes playable at once. each entry correspondents to a (set of) floppy drive(s) */
    // TODO: we need to keep track of the driver state for more than one drive at a time if we access multiple drives
    //  on higher volume
    FloppyDriveState notes[MAX_NOTES_PER_CHANNEL];

    /** midi channel pitch */
    uint16_t pitch;

    /** midi channel pressure */
    uint8_t channel_pressure;
} FloppyRegister;

/**
 * Reset the main channel to its default state (can be used for initialization too)
 */
void reset_floppy_register(FloppyRegister *organRegister);

/**
 * Initialize the scheduler
 */
void init_scheduler(void);

/**
 * Free resources allocated by the scheduler
 */
void destroy_scheduler(void);

/**
 * The main loop of the scheduler. It constantly trys pulling midi messages and handles them if it finds any.
 */
_Noreturn void scheduler_loop(void);

#endif //FLOPPYORGAN_SCHEDULER_H
