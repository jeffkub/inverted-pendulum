#include <Arduino.h>

#include <Adafruit_MotorShield.h>

#define LED_PIN         LED_BUILTIN
#define ENCODER1_A_PIN  2
#define ENCODER1_B_PIN  5
#define ENCODER2_A_PIN  3
#define ENCODER2_B_PIN  4
#define LIMIT1_PIN      10
#define LIMIT2_PIN      11

#define LED_PERIOD      250
#define MOTOR_TIMEOUT   1000

#define BUF_LEN         32

static void encoder1APinChange();
static void encoder2APinChange();

static void handleBlink();

static void cmdGetInputs();
static void cmdSetMotor(long speed);
static void handleLine(const char * line);
static void handleSerial();

static void handleMotorTimeout();

static Adafruit_MotorShield motor_shield = Adafruit_MotorShield();
static Adafruit_DCMotor * motor = motor_shield.getMotor(1);

static volatile long encoder1 = 0;
static volatile long encoder2 = 0;

static unsigned long last_motor_update = 0;

static void encoder1APinChange()
{
    encoder1 += (digitalRead(ENCODER1_A_PIN) == digitalRead(ENCODER1_B_PIN)) ? +1 : -1;
}

static void encoder2APinChange()
{
    encoder2 += (digitalRead(ENCODER2_A_PIN) == digitalRead(ENCODER2_B_PIN)) ? +1 : -1;
}

static void handleBlink()
{
    static unsigned long last = 0;
    static uint8_t state = LOW;

    if(millis() - last < LED_PERIOD)
        return;

    last += LED_PERIOD;

    state = (state == LOW) ? HIGH : LOW;
    digitalWrite(LED_PIN, state);
}

static void cmdGetInputs()
{
    long encoder1_temp;
    long encoder2_temp;
    int limit1_temp;
    int limit2_temp;

    noInterrupts();
    encoder1_temp = encoder1;
    encoder2_temp = encoder2;
    interrupts();

    limit1_temp = digitalRead(LIMIT1_PIN);
    limit2_temp = digitalRead(LIMIT2_PIN);

    Serial.print(encoder1_temp);
    Serial.print("\t");
    Serial.print(encoder2_temp);
    Serial.print("\t");
    Serial.print(limit1_temp);
    Serial.print("\t");
    Serial.println(limit2_temp);
    Serial.flush();
}

static void cmdSetMotor(long speed)
{
    /* Clamp speed value */
    speed = (speed >  2047) ?  2047 : speed;
    speed = (speed < -2047) ? -2047 : speed;

    if(speed == 0)
    {
        motor->setSpeed(0);
        motor->run(RELEASE);
    }
    else if(speed > 0)
    {
        motor->setSpeed(speed << 1);
        motor->run(FORWARD);
    }
    else
    {
        motor->setSpeed((-speed) << 1);
        motor->run(BACKWARD);
    }

    last_motor_update = millis();
}

static void handleLine(const char * line)
{
    long val;

    if(strcmp(line, "R") == 0)
    {
        cmdGetInputs();
    }
    else if(sscanf(line, "M1 : %ld", &val) == 1)
    {
        cmdSetMotor(val);
    }
}

static void handleSerial()
{
    static char buf[BUF_LEN+1] = {0};
    static unsigned len = 0;
    char input;

    while(Serial.available())
    {
        input = Serial.read();

        if(len == 0 && isspace(input))
            continue;

        if(input == '\r' || input == '\n')
        {
            buf[len] = '\0';
            handleLine(buf);
            len = 0;
            break;
        }
        else if(len < BUF_LEN)
        {
            buf[len] = input;
            len++;
        }
    }
}

static void handleMotorTimeout()
{
    unsigned long cur;

    cur = millis();

    if(cur - last_motor_update >= MOTOR_TIMEOUT)
    {
        last_motor_update = cur;

        /* Disable the motor if no command has been received in a while */
        motor->setSpeed(0);
        motor->run(RELEASE);
    }
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
    pinMode(LIMIT1_PIN, INPUT_PULLUP);
    pinMode(LIMIT2_PIN, INPUT_PULLUP);

    Serial.begin(115200);

    motor_shield.begin();

    attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1APinChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2APinChange, CHANGE);
}

void loop()
{
    handleBlink();
    handleSerial();
    handleMotorTimeout();
}
