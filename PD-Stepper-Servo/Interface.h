//
// Constants
//

enum MODE
{
  POSITION,   // PID with position command
  VELOCITY,   // Velocity command (microsteps)
  MANUAL      // Manual control via buttons
};

enum VOLTAGE
{
  VOLTAGE_5V = 5,
  VOLTAGE_9V = 9,
  VOLTAGE_12V = 12,
  VOLTAGE_15V = 15,
  VOLTAGE_20V = 20
};

enum STANDSTILL
{
  NORMAL = 0,
  FREEWHEELING = 1,
  STRONG_BRAKING = 2,
  BRAKING = 3
};

//
// Structures
//

struct Status
{
  MODE mode;
  bool enabled;

  int rawPosition;
  float position;

  int velocity;

  float voltage;
  bool overTemp;
  bool overTempShutdown;
  bool stalled;
};

struct PositionFeedback
{
  float goal;
  float position;
  float error;
  float integralError;
  float derivativeError;
};

struct Settings
{
  int voltage;
  int current;
  int microsteps;
  int stallThreshold;
  STANDSTILL standstillMode;
  int buttonVelocity;

  int encoderMin;
  int encoderMax;
  float positionMin;
  float positionMax;
  int velocityMin;
  int velocityMax;

  float Kp;
  float Ki;
  float Kd;
  float iMin;
  float iMax;
  float tolerance;
};

//
// Types
//

typedef void (*EnabledCommandPtr)(bool);
typedef void (*PositionCommandPtr)(float);
typedef void (*VelocityCommandPtr)(int);
typedef void (*SettingsCommandPtr)(const Settings&);
typedef void (*StatusFeedbackPtr)(Status&);
typedef void (*PositionFeedbackPtr)(PositionFeedback&);
typedef void (*VelocityFeedbackPtr)(int&);
typedef void (*SettingsFeedbackPtr)(Settings&);
