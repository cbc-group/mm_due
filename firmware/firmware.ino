#include <DueTimer.h>

// firmware version (not ASCII)
const uint8_t VERSION = 3;

// serial timeout
# define TIMEOUT  1000

// DEBUG
#define LED     13

// sequence buffer
#define SEQ_BUF_SIZE      16

// camera control signal
#define CAMERA_FIRE_ALL   46

// galvo DAC
#define X_GALVO_DAC       DAC1

union GalvoWaveform {
    struct {
        uint16_t amplitude;
        uint16_t offset;
        uint16_t dt;
    };
    uint16_t arr[3];
};

typedef struct {
    uint16_t value;
    GalvoWaveform waveform;
} Galvo;

typedef struct {
    uint8_t skip_n;

    uint8_t index;
    uint8_t len;

    uint8_t patterns[SEQ_BUF_SIZE];
} LaserSequence;

typedef struct {   
    uint8_t pattern : 8;
    LaserSequence sequence;

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
    READY = 1,

    TRIGGER_MODE__START,
    TRIGGER_MODE__WAITING,
    TRIGGER_MODE__EXPOSURE_START,
    TRIGGER_MODE__EXPOSING,
    TRIGGER_MODE__EXPOSURE_STOP,
    TRIGGER_MODE__STOP,

    BLANKING_MODE__WAITING,
    BLANKING_MODE__ACTIVE,
    BLANKING_MODE__INACTIVE,
    BLANKING_MODE__STOP
};

typedef struct {
    int state;
    uint8_t blankOnHigh : 1;

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

bool waitForSerial(unsigned long timeOut) {
    unsigned long startTime = millis();
    while (Serial.available() == 0 && (millis() - startTime < timeOut) ) {}
    if (Serial.available() > 0)
        return true;
    return false;
}

void camera_event() {
    if (digitalRead(CAMERA_FIRE_ALL)) {
        digitalWrite(LED, HIGH);
        states.camera.event = EXPOSURE_STARTED;
    } else {
        digitalWrite(LED, LOW);
        states.camera.event = EXPOSURE_FINISHED;
    }
}

void updateXGalvo() {
    analogWrite(X_GALVO_DAC, states.x_galvo.value);

    // prevent overflow
    uint32_t next_value = (uint32_t)states.x_galvo.value + (uint32_t)states.x_galvo.waveform.dt;
    uint32_t max_value = (uint32_t)states.x_galvo.waveform.offset + (uint32_t)states.x_galvo.waveform.amplitude;
    if (next_value >= max_value) {
        next_value = max_value;
    }
    states.x_galvo.value = (uint16_t)(next_value & 0xFFFF);
}

void setup() {
    Serial.begin(57600);

    init_laser();
    clear_laser_pattern();
    
    // camera trigger
    pinMode(CAMERA_FIRE_ALL, INPUT_PULLUP);

    // galvo DAC control
    analogWriteResolution(12);
    // 100us update rate
    Timer2.attachInterrupt(updateXGalvo).setPeriod(100);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // init state
    states.state = READY;
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

                if (states.state == READY) {
                    set_laser_pattern();
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
                  Where x is the register (0: amplitude, 1: offset) and vv is the output in
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
                    if (channel == 1) {
                        // update offset immediately, neutral position
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
                  Where x is the number of the pattern (currrently, 12 patterns can be stored).
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
                {
                    uint8_t len;
                    if (waitForSerial(TIMEOUT)) {
                        len = Serial.read();
                    } else {
                        break;
                    }

                    states.laser.sequence.len = len;

                    Serial.write(6);
                    Serial.write(len);
                    break;
                }

            /*  Skip tirgger: 7x
                  Where x indicates how many digital change events on the trigger input pin
                  will be ignored.

                  Returns 7x
            */
            case 7:
                {
                    uint8_t skip_n;
                    if (waitForSerial(TIMEOUT)) {
                        skip_n = Serial.read();
                    } else {
                        break;
                    }

                    states.laser.sequence.skip_n = skip_n;

                    Serial.write(7);
                    Serial.write(skip_n);
                    break;
                }

            /*  Start trigger mode: 8
                  Trigger mode will supersede blanking mode.

                  Returns 8 to indicate start of trigger mode
            */
            case 8:
                if (states.laser.sequence.len <= 0) {
                    break;
                }
                states.laser.sequence.index = 0;
                states.laser.counter = -states.laser.sequence.skip_n;

                states.state = TRIGGER_MODE__START;

                Serial.write(8);
                break;

            /*  Stop Trigger mode: 9
                  Returns 9x where x is the number of triggers received during the last
                  trigger mode run
            */
            case 9:
                states.state = TRIGGER_MODE__STOP;

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
                states.state = BLANKING_MODE__WAITING;
                Serial.write(20);
                break;

            /*  Stop blanking mode: 21
                 Stopts blanking mode

                 Returns 21
            */
            case 21:
                states.state = BLANKING_MODE__STOP;
                Serial.write(21);
                break;

            /*  Set blanking mode polarity: 22
                 Sets 'polarity' of input TTL for blanking mode. 0 for positive level.

                 Returns 22 to indicate succesfull execution.
            */
            case 22:
                if (waitForSerial(TIMEOUT)) {
                    states.blankOnHigh = (Serial.read() == 0);
                    Serial.write(22);
                } break;


            /*  Get Identifcatio: 30
                 Returns in ASCI MM-Ard\r\n
            */
            case 30:
                Serial.println("MM-Ard");
                break;

            /*  Get Version: 31
                 Returns: version number in ASCI \r\n
            */
            case 31:
                Serial.println(VERSION);
                break;
        }
    }

    switch (states.state) {
        case READY:
            break;

        /*
            Trigger mode
        */
        case TRIGGER_MODE__START:
            states.camera.event = NONE;
            attachInterrupt(digitalPinToInterrupt(CAMERA_FIRE_ALL), camera_event, CHANGE);
            states.state = TRIGGER_MODE__WAITING;
            break;

        case TRIGGER_MODE__WAITING:
            if (states.camera.event == EXPOSURE_STARTED) {
                states.camera.event = NONE;
                states.state = TRIGGER_MODE__EXPOSURE_START;
            }
            break;

        case TRIGGER_MODE__EXPOSURE_START:
            // skip first N triggers
            if (states.laser.counter < 0) {
                ++states.laser.counter;

                states.state = TRIGGER_MODE__WAITING;
                break;
            }

            // set pattern
            set_laser_pattern();
            // update next pattern
            ++states.laser.sequence.index;
            if (states.laser.sequence.index >= states.laser.sequence.len) {
                states.laser.sequence.index = 0;
            }
            states.laser.pattern = states.laser.sequence.patterns[states.laser.sequence.index];

            // reset galvo
            states.x_galvo.value = states.x_galvo.waveform.offset;
            // fire galvo
            Timer2.start();

            states.state = TRIGGER_MODE__EXPOSING;
            break;

        case TRIGGER_MODE__EXPOSING:
            if (states.camera.event == EXPOSURE_FINISHED) {
                states.camera.event = NONE;
                states.state = TRIGGER_MODE__EXPOSURE_STOP;
            }
            break;

        case TRIGGER_MODE__EXPOSURE_STOP:
            // stop timer
            Timer2.stop();
            // stop output
            clear_laser_pattern();
            // reset galvo
            states.x_galvo.value = states.x_galvo.waveform.offset;

            states.state = TRIGGER_MODE__WAITING;
            break;

        case TRIGGER_MODE__STOP:
            detachInterrupt(digitalPinToInterrupt(CAMERA_FIRE_ALL));
            states.state = READY;
            break;

        /*
            Blanking mode
        */
        case BLANKING_MODE__WAITING:
            if ((digitalRead(CAMERA_FIRE_ALL) ^ states.blankOnHigh) & 0x01) {
                states.state = BLANKING_MODE__INACTIVE;
            } else {
                states.state = BLANKING_MODE__ACTIVE;              
            }
            break;

        case BLANKING_MODE__ACTIVE:
            // disable output when active (blank the output)
            clear_laser_pattern();
            states.state = BLANKING_MODE__WAITING;
            break;

        case BLANKING_MODE__INACTIVE:
            set_laser_pattern();
            states.state = BLANKING_MODE__WAITING;
            break;

        case BLANKING_MODE__STOP:
            set_laser_pattern();
            states.state = READY;
            break;
    }
}
