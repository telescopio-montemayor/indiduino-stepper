#undef DEBUG
#ifdef DEBUG
  #include <SoftwareSerial.h>
#endif

#include <Firmata.h>
#include <AccelStepper.h>


// Fake pins to send to host via Firmata.

#define FIRMATA_TRACK_AXIS_0        2
#define FIRMATA_SLEW_AXIS_0         3
#define FIRMATA_AXIS_0_DIRECTION    4
#define FIRMATA_AXIS_0_RESERVED     5

#define FIRMATA_TRACK_AXIS_1        6
#define FIRMATA_SLEW_AXIS_1         7
#define FIRMATA_AXIS_1_DIRECTION    8
#define FIRMATA_AXIS_1_RESERVED     9

#define FIRMATA_TRACK_SPEED_NUM     10
#define FIRMATA_TRACK_SPEED_DEN     11
#define FIRMATA_SLEW_SPEED_NUM      12
#define FIRMATA_SLEW_SPEED_DEN      13
#define FIRMATA_ACCELERATION        14

#define FIRMATA_AXIS_0_POSITION     15
#define FIRMATA_AXIS_1_POSITION     16

#define LAST_PIN                    FIRMATA_AXIS_1_POSITION
#define TOTAL_PINS                  15
int pinState[LAST_PIN + 1];
byte pinConfig[LAST_PIN + 1];

#define MINIMUM_SAMPLING_INTERVAL 1000
#define AXIS_UPDATE_INTERVAL      100


// Real Arduino pins.
#define AXIS_0_PIN_STEP             11
#define AXIS_0_PIN_DIR              12
#define AXIS_0_PIN_ENABLE           13
AccelStepper motor0(AccelStepper::DRIVER, AXIS_0_PIN_STEP, AXIS_0_PIN_DIR);

#ifdef DEBUG
  #define DEBUG_PORT_RX             4
  #define DEBUG_PORT_TX             3
  // rx, tx, invert
  SoftwareSerial debugPort(DEBUG_PORT_RX, DEBUG_PORT_TX, true);
#endif



typedef struct _AxisState {
  AccelStepper  *motor;
  unsigned long last_update_time;
  float         acceleration;
  float         current_speed;
  float         target_speed;
  bool          isTracking;
  bool          isSlewing;
  bool          direction_cw;
} AxisState;

unsigned long currentMillis;
unsigned long previousMillis;
int samplingInterval = MINIMUM_SAMPLING_INTERVAL;

unsigned int track_speed_num = 200;
unsigned int track_speed_den = 1;
unsigned int slew_speed_num  = 400;
unsigned int slew_speed_den  = 1;
unsigned int acceleration    = 1;
float track_speed = (float)track_speed_num / track_speed_den;
float slew_speed  = (float)slew_speed_num  / slew_speed_den;

/* forward declarations */
static inline void custom_analog_input();
static inline void custom_firmata_loop();
static inline void custom_loop();
void set_axis_slew (AxisState *axis, bool slew);
void set_axis_track(AxisState *axis, bool track);
static inline void compute_axis_speed(AxisState *axis);
void update_axis_speeds(AxisState *axis);
void setPinValueCallback(byte pin, int value);

AxisState axis0;

static inline
void compute_axis_speed(AxisState *axis)
{
  float current_speed, target_speed, new_speed;
  unsigned long now;
  signed long   interval;

  if (!axis)  {
    return;
  }

  now = millis();

  interval = now - axis->last_update_time;
  interval = interval > 0 ? interval : -interval;
  if (interval < AXIS_UPDATE_INTERVAL) {
    axis->motor->runSpeed();
    return;
  }

  axis->last_update_time = now;

  current_speed = axis->current_speed;
  target_speed  = axis->direction_cw ? axis->target_speed : -1.0f * axis->target_speed;

  if (current_speed != target_speed) {
    if (current_speed < target_speed) {
      new_speed = current_speed + axis->acceleration;
      if (new_speed > target_speed) {
        new_speed = target_speed;
      }
    } else {
      new_speed = current_speed - axis->acceleration;
      if (new_speed < target_speed) {
        new_speed = target_speed;
      }
    }
    axis->current_speed = new_speed;
  }

  axis->motor->setSpeed(axis->current_speed);
  axis->motor->runSpeed();
}

void
update_axis_speeds(AxisState *axis)
{
#ifdef DEBUG
  char buf[40];
  sprintf(buf, "update_axis_speeds() track: %d slew: %d\n", axis->isTracking, axis->isSlewing);
  debugPort.println(buf);
#endif

  axis->acceleration = acceleration * 0.001 * AXIS_UPDATE_INTERVAL;

  if (axis->isTracking) {
    axis->target_speed = track_speed;
    return;
  }

  if (axis->isSlewing) {
    axis->target_speed = slew_speed;
    return;
  }

  axis->target_speed = 0;
}

void
set_axis_track(AxisState *axis, bool track)
{
#ifdef DEBUG
  char buf[40];
  sprintf(buf, "set_axis_track() %d\n", track);
  debugPort.println(buf);
#endif

  axis->isTracking = track;
  update_axis_speeds(axis);
}

void
set_axis_slew(AxisState *axis, bool slew)
{
#ifdef DEBUG
  char buf[40];
  sprintf(buf, "set_axis_slew() %d\n", slew);
  debugPort.println(buf);
#endif

  axis->isSlewing = slew;
  update_axis_speeds(axis);
}

static inline
void custom_loop() {
   compute_axis_speed(&axis0);
}

static inline
void custom_firmata_loop () {
  while(Firmata.available()) {
    Firmata.processInput();
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis = currentMillis;
    custom_analog_input();
  }
}

static inline
void custom_analog_input() {
  Firmata.sendAnalog(FIRMATA_AXIS_0_POSITION - FIRMATA_AXIS_0_POSITION, motor0.currentPosition());
//XXX FIXME: add the other axis later.
  //Firmata.sendAnalog(FIRMATA_AXIS_1_POSITION - FIRMATA_AXIS_0_POSITION, motor1.currentPosition());
}

void analogWriteCallback(byte pin, int value)
{
#ifdef DEBUG
  char buf[40];
  sprintf(buf, "analogWrite() %i %i\n", pin, value);
  debugPort.println(buf);
#endif

  if (!(pin < LAST_PIN)) {
    return;
  }

  switch (pin) {
    case FIRMATA_TRACK_SPEED_NUM:
      track_speed_num = value;
      track_speed = (float)track_speed_num / track_speed_den;
      update_axis_speeds(&axis0);
      pinState[pin] = value;
      break;

    case FIRMATA_TRACK_SPEED_DEN:
      track_speed_den = value > 0 ? value : 1;
      track_speed = (float)track_speed_num / track_speed_den;
      update_axis_speeds(&axis0);
      pinState[pin] = value;
      break;

    case FIRMATA_SLEW_SPEED_NUM:
      slew_speed_num = value;
      slew_speed  = (float)slew_speed_num  / slew_speed_den;
      update_axis_speeds(&axis0);
      pinState[pin] = value;
      break;

    case FIRMATA_SLEW_SPEED_DEN:
      slew_speed_den = value > 0 ? value : 1;
      slew_speed  = (float)slew_speed_num  / slew_speed_den;
      update_axis_speeds(&axis0);
      pinState[pin] = value;
      break;

    case FIRMATA_ACCELERATION:
      acceleration = value;
      update_axis_speeds(&axis0);
      pinState[pin] = value;
      break;

    default:
      break;
  }
}

void setPinValueCallback(byte pin, int value)
{
#ifdef DEBUG
  char buf[40];
  sprintf(buf, "setPinValue() %i %i\n", pin, value);
  debugPort.println(buf);
#endif

  switch (pin) {
    case FIRMATA_TRACK_AXIS_0:
      set_axis_track(&axis0, value);
      break;

    case FIRMATA_SLEW_AXIS_0:
      set_axis_slew(&axis0, value);
      break;

    case FIRMATA_AXIS_0_DIRECTION:
      axis0.direction_cw = value ? true : false;
      break;

    default:
      break;
  }

  if (IS_PIN_DIGITAL(pin)) {
    if (pinConfig[pin] == OUTPUT) {
      pinState[pin] = value;
    }
  }
}


void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask=1, pinWriteMask=0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port*8+8;
    if (lastPin > LAST_PIN){
      lastPin = LAST_PIN;
    }

    for (pin=port*8; pin < lastPin; pin++) {

      byte pinValue = ((byte)value & mask) ? 1 : 0;
      setPinValueCallback(pin, pinValue);
      mask = mask << 1;
    }
  }
}


void sysexCallback(byte command, byte argc, byte *argv)
{
  switch (command) {

    case SAMPLING_INTERVAL:
      break;

    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;

    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);

      for (byte p=FIRMATA_TRACK_AXIS_0; p <=FIRMATA_AXIS_1_RESERVED ; p++) {
          pinConfig[p] = OUTPUT;
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
          Firmata.write(127);
      }

      for (byte p=FIRMATA_TRACK_SPEED_NUM; p <= FIRMATA_SLEW_SPEED_DEN; p++) {
          pinConfig[p] = PWM;
          Firmata.write((byte)PWM);
          Firmata.write(14);
          Firmata.write(127);
      }

      for (byte p=FIRMATA_AXIS_0_POSITION; p <= FIRMATA_AXIS_1_POSITION; p++) {
          pinConfig[p] = ANALOG;
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(14);
          Firmata.write(127);
      }

      Firmata.write(END_SYSEX);
      break;

    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < LAST_PIN) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;

    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = FIRMATA_TRACK_AXIS_0; pin < LAST_PIN; pin++) {
        if ( (pin >= FIRMATA_AXIS_0_POSITION) && (pin <= FIRMATA_AXIS_1_POSITION) ) {
            Firmata.write(pin - FIRMATA_AXIS_0_POSITION);
        } else {
            Firmata.write(127);
        }
      }
      Firmata.write(END_SYSEX);
      break;
  }
}


void setup()
{
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);


  Firmata.attach(ANALOG_MESSAGE,        analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE,       digitalWriteCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX,           sysexCallback);

  for (byte p=FIRMATA_TRACK_AXIS_0; p <= FIRMATA_AXIS_1_RESERVED; p++) {
    Firmata.setPinMode(p, OUTPUT);
  }

  for (byte p=FIRMATA_TRACK_SPEED_NUM; p <= FIRMATA_ACCELERATION; p++) {
    Firmata.setPinMode(p, PWM);
  }

  pinMode(AXIS_0_PIN_STEP,   OUTPUT);
  pinMode(AXIS_0_PIN_DIR,    OUTPUT);
  pinMode(AXIS_0_PIN_ENABLE, OUTPUT);

#ifdef DEBUG
  pinMode(DEBUG_PORT_TX, OUTPUT);
  pinMode(DEBUG_PORT_RX, INPUT);
  debugPort.begin(9600);
#endif

  motor0.setEnablePin(AXIS_0_PIN_ENABLE);
  motor0.setMinPulseWidth(500);
  motor0.setMaxSpeed(400.0);
  motor0.setAcceleration(acceleration);
  motor0.setCurrentPosition(0);
  motor0.enableOutputs();

  axis0.motor         = &motor0;
  axis0.current_speed = 0;
  axis0.direction_cw  = false;
  update_axis_speeds(&axis0);

  Firmata.begin(57600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }
}

void loop()
{
  custom_loop();
  custom_firmata_loop();
}
