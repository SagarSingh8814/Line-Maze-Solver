/*
  xpecto-lms — Line Maze Solver
  ------------------------------
  Board: ESP32-DEVKITC
  Motor driver: L293D
  Sensors: 5x IR line sensors
  Start button: GPIO32 (debounced push button)

  Pin mapping taken from Schematic_xpecto-lms schematic (Sheet_2):

    Line sensors (P6-P10, header 1x3: gpio/gnd/vcc):
      SENSOR_1 -> GPIO27
      SENSOR_2 -> GPIO14
      SENSOR_3 -> GPIO23
      SENSOR_4 -> GPIO22
      SENSOR_5 -> GPIO21

    Start/mode button (KEY1):
      BUTTON -> GPIO32

    Status LED / indicator (R1 pull-up):
      STATUS -> GPIO2

    Motor driver (U4, L293D):
      Motor A (left):  IN1 -> GPIO16, IN2 -> GPIO19
      Motor B (right): IN3 -> GPIO18, IN4 -> GPIO5
      Motor A output -> m1a / m1b (CN3)
      Motor B output -> m2a / m2b (CN4)

    NOTE: double-check exact IN/EN GPIO numbers against your schematic —
    some silkscreen labels (gpio4/gpio5/gpio13/gpio15) were hard to
    read at full resolution; confirm before flashing to hardware.

  ESP32 uses ledcWrite for PWM (no analogWrite like AVR boards).
*/

// ---------------- Pin definitions ----------------
const int SENSOR_PINS[5] = {27, 14, 23, 22, 21}; // left to right

const int BUTTON_PIN = 32;
const int STATUS_LED = 2;

// L293D control pins
const int MOTOR_L_IN1 = 16;
const int MOTOR_L_IN2 = 19;
const int MOTOR_R_IN1 = 18;
const int MOTOR_R_IN2 = 5;

// PWM (ESP32 LEDC channels)
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;      // 8-bit: 0-255
const int PWM_CH_L = 0;
const int PWM_CH_R = 1;
const int MOTOR_L_EN = 17;   // route to EN1 on L293D if not tied high
const int MOTOR_R_EN = 4;    // route to EN2 on L293D if not tied high

// ---------------- PID constants ----------------
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

int baseSpeed = 150; // 0-255
int maxSpeed  = 220;

float lastError = 0;
float integral  = 0;

// ---------------- Maze grid (flood-fill) ----------------
const int GRID_SIZE = 16;
int distGrid[GRID_SIZE][GRID_SIZE];
bool wallN[GRID_SIZE][GRID_SIZE];
bool wallE[GRID_SIZE][GRID_SIZE];
bool wallS[GRID_SIZE][GRID_SIZE];
bool wallW[GRID_SIZE][GRID_SIZE];

int robotX = 0, robotY = 0;
int goalX = GRID_SIZE - 1, goalY = GRID_SIZE - 1;
int heading = 0; // 0=N, 1=E, 2=S, 3=W

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);

  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_EN, PWM_CH_L);
  ledcAttachPin(MOTOR_R_EN, PWM_CH_R);

  initFloodFill();

  // wait for start button press
  digitalWrite(STATUS_LED, HIGH);
  while (digitalRead(BUTTON_PIN) == HIGH) { delay(10); }
  digitalWrite(STATUS_LED, LOW);
  delay(300); // debounce
}

void loop() {
  exploreMaze();
  computeFloodFill();
  runShortestPath();

  while (true) { stopMotors(); }
}

// ---------------- Line following (PID) ----------------
void lineFollowStep() {
  int sensorVal[5];
  for (int i = 0; i < 5; i++) sensorVal[i] = digitalRead(SENSOR_PINS[i]);

  int weights[5] = {-2, -1, 0, 1, 2};
  long weightedSum = 0;
  int activeCount = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorVal[i] == HIGH) { // adjust HIGH/LOW to match your sensor's logic
      weightedSum += weights[i];
      activeCount++;
    }
  }

  float error = (activeCount > 0) ? (float)weightedSum / activeCount : lastError;

  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed  = constrain(baseSpeed - correction, -maxSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, -maxSpeed, maxSpeed);

  setMotor(true,  leftSpeed);
  setMotor(false, rightSpeed);
}

void setMotor(bool isLeft, int speed) {
  int in1 = isLeft ? MOTOR_L_IN1 : MOTOR_R_IN1;
  int in2 = isLeft ? MOTOR_L_IN2 : MOTOR_R_IN2;
  int ch  = isLeft ? PWM_CH_L : PWM_CH_R;

  digitalWrite(in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2, speed >= 0 ? LOW  : HIGH);
  ledcWrite(ch, constrain(abs(speed), 0, 255));
}

void stopMotors() {
  ledcWrite(PWM_CH_L, 0);
  ledcWrite(PWM_CH_R, 0);
}

// ---------------- Maze exploration ----------------
void exploreMaze() {
  while (!(robotX == goalX && robotY == goalY)) {
    lineFollowStep();
    if (atIntersection()) {
      int turn = decideTurn();
      logWall(turn);
      applyTurn(turn);
      updatePosition();
    }
  }
}

bool atIntersection() {
  // placeholder: replace with real junction detection
  // (e.g. all 5 sensors HIGH = cross junction)
  return false;
}

int decideTurn() {
  // placeholder: left-hand-rule / wall-following decision logic
  return 0;
}

void logWall(int turn) {
  // placeholder: mark wallN/E/S/W[robotX][robotY] based on heading + turn
}

void applyTurn(int turn) {
  // placeholder: pivot in place using motors, update `heading`
}

void updatePosition() {
  // placeholder: advance robotX/robotY one cell based on heading
}

// ---------------- Flood-fill shortest path ----------------
void initFloodFill() {
  for (int x = 0; x < GRID_SIZE; x++)
    for (int y = 0; y < GRID_SIZE; y++)
      distGrid[x][y] = abs(x - goalX) + abs(y - goalY);
}

void computeFloodFill() {
  bool visited[GRID_SIZE][GRID_SIZE] = {false};
  int qx[GRID_SIZE * GRID_SIZE], qy[GRID_SIZE * GRID_SIZE];
  int head = 0, tail = 0;

  distGrid[goalX][goalY] = 0;
  qx[tail] = goalX; qy[tail] = goalY; tail++;
  visited[goalX][goalY] = true;

  while (head < tail) {
    int cx = qx[head], cy = qy[head]; head++;
    int d = distGrid[cx][cy];

    tryVisit(cx, cy - 1, d, !wallN[cx][cy], visited, qx, qy, tail);
    tryVisit(cx + 1, cy, d, !wallE[cx][cy], visited, qx, qy, tail);
    tryVisit(cx, cy + 1, d, !wallS[cx][cy], visited, qx, qy, tail);
    tryVisit(cx - 1, cy, d, !wallW[cx][cy], visited, qx, qy, tail);
  }
}

void tryVisit(int nx, int ny, int d, bool open, bool visited[GRID_SIZE][GRID_SIZE],
              int qx[], int qy[], int &tail) {
  if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) return;
  if (!open || visited[nx][ny]) return;
  distGrid[nx][ny] = d + 1;
  visited[nx][ny] = true;
  qx[tail] = nx; qy[tail] = ny; tail++;
}

void runShortestPath() {
  robotX = 0; robotY = 0; heading = 0;
  while (!(robotX == goalX && robotY == goalY)) {
    int bestX = robotX, bestY = robotY, bestDist = distGrid[robotX][robotY];

    if (!wallN[robotX][robotY] && distGrid[robotX][robotY - 1] < bestDist) {
      bestX = robotX; bestY = robotY - 1; bestDist = distGrid[bestX][bestY];
    }
    if (!wallE[robotX][robotY] && distGrid[robotX + 1][robotY] < bestDist) {
      bestX = robotX + 1; bestY = robotY; bestDist = distGrid[bestX][bestY];
    }
    if (!wallS[robotX][robotY] && distGrid[robotX][robotY + 1] < bestDist) {
      bestX = robotX; bestY = robotY + 1; bestDist = distGrid[bestX][bestY];
    }
    if (!wallW[robotX][robotY] && distGrid[robotX - 1][robotY] < bestDist) {
      bestX = robotX - 1; bestY = robotY; bestDist = distGrid[bestX][bestY];
    }

    driveToCell(bestX, bestY);
    robotX = bestX; robotY = bestY;
  }
}

void driveToCell(int nx, int ny) {
  // placeholder: turn to face (nx,ny) relative to heading, then
  // lineFollowStep() forward one cell length
}
