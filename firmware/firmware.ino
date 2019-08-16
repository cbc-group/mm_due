#include <DueTimer.h>

// numeric firmware version (not ASCII)
const uint8_t VERSION = 3;

// serial timeout
# define TIMEOUT          500

// sequence buffer
#define SEQ_BUF_SIZE      16

// camera control signal
#define CAMERA_FIRE_ALL   46

// galvo DAC
#define X_GALVO_DAC       DAC1

union GalvoWaveform {
    struct {
        uint16_t p0;
        uint16_t p1;
        uint16_t dt;
    };
    uint16_t arr[3];
};

typedef struct {
    uint16_t value;
    GalvoWaveform waveform;
} Galvo;

typedef struct {
    uint8_t activate : 1;

    uint8_t index;
    uint8_t len;

    uint8_t patterns[SEQ_BUF_SIZE];
} LaserSequence;

typedef struct {
    uint8_t blanking : 1;
    uint8_t blankOnHigh : 1;

    uint8_t pattern;
    LaserSequence sequence;

    uint8_t skip_n;
    int32_t counter;
} Laser;

enum CameraEvent {
    NONE = 1,
    EXPOSURE_STARTED,
    EXPOSURE_FINISHED
};

typedef struct {
    CameraEvent event;
} Camera;

enum State {
    RESET = 1,

    WAIT_TRIGGER,

    EXPOSURE__STARTED,
    EXPOSURE__IN_PROGRESS,
    EXPOSURE__STOPPED
};

typedef struct {
    int state;

    Galvo x_galvo;
    Laser laser;
    Camera camera;
} States;

volatile States states = { 0 };

void init_laser() {
    // power on PIOD clock
    PMC->PMC_PCER0 |= PMC_PCER0_PID14;
    // digital output control
    PIOD->PIO_PER |= 0xFF;
    PIOD->PIO_OER |= 0xFF;
    // allow synchronous set-clear
    PIOD->PIO_OWER |= 0xFF;
}

void set_laser_pattern() {
    PIOD->PIO_ODSR = states.laser.pattern;
}

void clear_laser_pattern() {
    PIOD->PIO_CODR = 0xFF;
}

bool waitForSerial(unsigned long timeout) {
    unsigned long t0 = millis();
    do {
        if (Serial.available() > 0) {
            return true;
        }
    } while (millis() - t0 < timeout);
    return false;
}

void camera_event() {
    if ((digitalRead(CAMERA_FIRE_ALL) ^ states.laser.blankOnHigh) & 0x01) {
        states.camera.event = EXPOSURE_STARTED;
    } else {
        states.camera.event = EXPOSURE_FINISHED;
    }
}

void updateXGalvo() {
    analogWrite(X_GALVO_DAC, states.x_galvo.value);

    // prevent overflow
    uint32_t next_value = (uint32_t)states.x_galvo.value + (uint32_t)states.x_galvo.waveform.dt;
    uint32_t max_value = (uint32_t)states.x_galvo.waveform.p0 + (uint32_t)states.x_galvo.waveform.p1;
    if (next_value >= max_value) {
        next_value = max_value;
    }
    states.x_galvo.value = (uint16_t)(next_value & 0xFFFF);
}

void setup() {
    Serial.begin(57600);

    // galvo DAC control
    analogWriteResolution(12);
    // 100us update rate
    Timer2.attachInterrupt(updateXGalvo).setPeriod(100);

    // laser
    init_laser();
    clear_laser_pattern();

    // camera trigger
    pinMode(CAMERA_FIRE_ALL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAMERA_FIRE_ALL), camera_event, CHANGE);
    states.camera.event = NONE;

    // init state
    states.state = RESET;
}

void loop() {
    if (Serial.available() > 0) {
        switch (Serial.read()) {
            /*  Set digital output command: 1p
                  Where p is the desired digital pattern.

                  Returns 1 to indicate succesfull execution.
            */
            case 1:
                if (waitForSerial(TIMEOUT)) {
                    states.laser.pattern = Serial.read();
                } else {
                    break;
                }

                Serial.write(1);
                break;

            /*  Get Digital output command: 2

                  Returns 2p. Where p is the current digital output pattern
            */
            case 2:
                Serial.write(2);
                Serial.write(states.laser.pattern);
                break;

            /*  X galvo control command 3xvv
                  Where x is the register (0: start, 1: end, 2: step) and vv is the output in
                  a 12-bit significant number.

                  Returns 3xvv
            */
            case 3:
                {
                    uint8_t channel = -1;
                    uint16_t value;

                    if (waitForSerial(TIMEOUT)) {
                        channel = Serial.read();
                    } else {
                        break;
                    }
                    if (channel >= 3) {
                        break;
                    }

                    // retrieve 2-byte payload
                    if (waitForSerial(TIMEOUT)) {
                        value = ((uint16_t)Serial.read()) << 8;
                        if (waitForSerial(TIMEOUT)) {
                            value |= (uint16_t)Serial.read();
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }

                    states.x_galvo.waveform.arr[channel] = value;
                    if (channel == 0) {
                        // update immediately, neutral position
                        states.x_galvo.value = value;
                        analogWrite(X_GALVO_DAC, value);
                    }

                    Serial.write(3);
                    Serial.write(channel);
                    Serial.write(value >> 8);
                    Serial.write(value & 0xFF);
                    break;
                }

            /*  Set digital pattern for triggerd mode: 5xd
                  Where x is the number of the pattern (currrently, 16 patterns can be stored).
                  and d is the digital pattern to be stored at that position. Note that x should be
                  real number (i.e., not ASCI encoded)

                  Returns 5xd
            */
            case 5:
                {
                    uint8_t index = -1;
                    uint8_t pattern = 0;

                    if (waitForSerial(TIMEOUT)) {
                        index = Serial.read();
                    } else {
                        break;
                    }
                    if (index >= SEQ_BUF_SIZE) {
                        break;
                    }

                    if (waitForSerial(TIMEOUT)) {
                        pattern = Serial.read();
                    } else {
                        break;
                    }

                    states.laser.sequence.patterns[index] = pattern;

                    Serial.write(5);
                    Serial.write(index);
                    Serial.write(pattern);
                }
                break;

            /*  Set the Number of digital patterns to be used: 6x
                  x indicates how many digital patterns will be used.

                Returns 6x
            */
            case 6:
                if (waitForSerial(TIMEOUT)) {
                    states.laser.sequence.len = Serial.read();
                } else {
                    break;
                }

                Serial.write(6);
                Serial.write(states.laser.sequence.len);
                break;

            /*  Skip tirgger: 7x
                  Where x indicates how many digital change events on the trigger input pin
                  will be ignored.

                  Returns 7x
            */
            case 7:
                if (waitForSerial(TIMEOUT)) {
                    states.laser.skip_n = Serial.read();
                } else {
                    break;
                }

                Serial.write(7);
                Serial.write(states.laser.skip_n);
                break;

            /*  Start sequence mode: 8

                  Returns 8 to indicate start of trigger mode
            */
            case 8:
                // reset index
                states.laser.sequence.index = 0;
                // reset acquisition count
                states.laser.counter = -states.laser.skip_n;

                states.laser.sequence.activate = true;

                Serial.write(8);
                break;

            /*  Stop sequence mode: 9
                  Returns 9x where x is the number of triggers received during the last
                  trigger mode run
            */
            case 9:
                states.laser.sequence.activate = false;

                Serial.write(9);
                Serial.write(states.laser.counter);
                break;

            /*  Start blanking mode: 20
                  In blanking mode, zeroes will be written on the output pins when the trigger pin
                  is low, when the trigger pin is high, the pattern set with command 1 will be
                  applied to the output pins.

                  Returns 20
            */
            case 20:
                states.laser.blanking = true;

                Serial.write(20);
                break;

            /*  Stop blanking mode: 21
                  Stopts blanking mode

                  Returns 21
            */
            case 21:
                states.laser.blanking = false;

                Serial.write(21);
                break;

            /*  Set trigger polarity: 22
                  0 for negative TTL (disable output, aka, blanking, when TTL is high).

                  Returns 22 to indicate succesfull execution.
            */
            case 22:
                if (waitForSerial(TIMEOUT)) {
                    states.laser.blankOnHigh = (Serial.read() == 0);
                } else {
                    break;
                }

                Serial.write(22);
                break;


            /*  Get Identifcatio: 30
                  Returns in ASCI "MM-Ard\r\n"
            */
            case 30:
                Serial.println("MM-Ard");
                break;

            /*  Get Version: 31
                  Returns: {numeric version number}"\r\n"
            */
            case 31:
                Serial.println(VERSION);
                break;
        }
    }

    switch (states.state) {
        case RESET:
            // reset galvo
            states.x_galvo.value = states.x_galvo.waveform.p0;

            states.state = WAIT_TRIGGER;
            break;

        case WAIT_TRIGGER:
            if (states.camera.event == EXPOSURE_STARTED) {
                states.camera.event = NONE;
                states.state = EXPOSURE__STARTED;
            }
            break;

        case EXPOSURE__STARTED:
            // skip first N triggers
            if (states.laser.counter < 0) {
                ++states.laser.counter;

                states.state = WAIT_TRIGGER;
                break;
            }

            // set pattern
            set_laser_pattern();

            // fire galvo
            Timer2.start();

            states.state = EXPOSURE__IN_PROGRESS;
            break;

        case EXPOSURE__IN_PROGRESS:
            if (states.camera.event == EXPOSURE_FINISHED) {
                states.camera.event = NONE;
                states.state = EXPOSURE__STOPPED;
            }
            break;

        case EXPOSURE__STOPPED:
            // stop timer
            Timer2.stop();

            // stop output
            if (states.laser.blanking) {
                clear_laser_pattern();
            }
            // update pattern if sequence mode is activated
            if (states.laser.sequence.activate) {
                // update next pattern
                ++states.laser.sequence.index;
                if (states.laser.sequence.index >= states.laser.sequence.len) {
                    states.laser.sequence.index = 0;
                }
                states.laser.pattern = states.laser.sequence.patterns[states.laser.sequence.index];
            }

            states.state = RESET;
            break;
    }
}
