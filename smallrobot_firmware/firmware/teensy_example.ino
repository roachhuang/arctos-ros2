#define COMMAND_TIMEOUT 500  // ms, timeout for new commands
// code to receive and parse position/velocity/effort commands from ROS 2:

unsigned long last_command_time = 0;

struct JointCommand {
  float position = 0.0;
  float velocity = 0.0;
  float effort = 0.0;
};

JointCommand joints[6]; // Assume up to 6 joints
String inputString = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Teensy Advanced Ready!");
}

void loop() {
  readSerial();
  updateMotors();
  checkTimeout();
}

void readSerial() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n') {
      parseAndExecute(inputString);
      inputString = "";
    }
  }
}

void parseAndExecute(const String& cmd) {
  if (cmd.length() < 4) return;

  char mode = cmd.charAt(0);  // P / V / E
  int joint_index = cmd.substring(1, cmd.indexOf(':')).toInt(); // 1-based index
  float value = cmd.substring(cmd.indexOf(':') + 1).toFloat();

  if (joint_index < 1 || joint_index > 6) {
    Serial.println("Invalid joint index");
    return;
  }

  last_command_time = millis();  // reset timeout

  switch (mode) {
    case 'P':
      joints[joint_index - 1].position = value;
      break;
    case 'V':
      joints[joint_index - 1].velocity = value;
      break;
    case 'E':
      joints[joint_index - 1].effort = value;
      break;
    default:
      Serial.println("Unknown mode");
      break;
  }
}

void updateMotors() {
  for (int i = 0; i < 6; ++i) {
    applyPosition(i, joints[i].position);
    // In future you can apply velocity/effort if you want
  }
}

void applyPosition(int joint_idx, float position) {
  // Your real motor control here
  // Example: setServo(joint_idx, position);
  Serial.print("Moving Joint ");
  Serial.print(joint_idx_
