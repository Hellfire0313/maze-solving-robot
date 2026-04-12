#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <Adafruit_VL53L1X.h>
#include <math.h>

// Motor pin assignments
#define PWMA 5      // Right motor PWM
#define AIN1 6      // Right motor forward control
#define AIN2 7      // Right motor reverse control
#define PWMB 2      // Left motor PWM
#define BIN1 4      // Left motor forward control
#define BIN2 3      // Left motor reverse control

#define RESET_BUTTON_PIN 35
#define CALIBRATION_BUTTON_PIN 40
#define LSRB_BUTTON_PIN 39  // Button for LSRB algorithm
#define RSLB_BUTTON_PIN 38  // Button for RSLB algorithm
#define OPTIMIZE_BUTTON_PIN 36  // Button for path optimization
#define START_OPTIMIZED_PATH_PIN 37  // Button to start optimized path

// Add these global variables
bool optimizationRequested = false;
bool startOptimizedPath = false;


// Add these constants at the top with other defines
#define WALL_TARGET_DISTANCE 130  // Target distance from walls in mm
#define WALL_MAX_DISTANCE 275     // Maximum distance to consider wall present
#define WALL_TOLERANCE 10         // Tolerance for wall distance variations

// Wall following mode enum
enum WallFollowMode {
    BOTH_WALLS,
    RIGHT_WALL,
    LEFT_WALL,
    NO_WALLS
};


// Compass direction thresholds (for simplicity)
float northAngle;
float eastAngle;
float southAngle;
float westAngle;
bool calibrationMode = false;
bool calibrationComplete = true;  // Start as true so robot can run without calibration
int calibrationStep = 0;
float tempNorthAngle = 0;
float tempWestAngle = 0;
float tempSouthAngle = 0;
float tempEastAngle = 0;
bool isLSRB = true;  // Default to LSRB algorithm


// VL53L1X sensor objects for four directions (center, left, right, additional)
Adafruit_VL53L1X vl53_center = Adafruit_VL53L1X();
Adafruit_VL53L1X vl53_right = Adafruit_VL53L1X();
Adafruit_VL53L1X vl53_left = Adafruit_VL53L1X();

// Multiplexer channel definitions
#define CENTER_SENSOR_CHANNEL 0
#define RIGHT_SENSOR_CHANNEL 1
#define LEFT_SENSOR_CHANNEL 2
#define BUTTON_PIN 37

// Path variables
String path = "";  // Store path in string format (L, R, S, U)
int paths = 0;  // To track if we detect an intersection or a path choice
bool endFound = false;  // Flag to indicate if the endpoint is found

// Current orientation of the robot
enum Orientation {
  NORTH,
  EAST,
  SOUTH,
  WEST
};

// Global variables for movement control
float targetAngle = 350.0;  // Default target angle (True North)
float tolerance = 2.0;      // Tolerance for alignment (in degrees)
Orientation currentOrientation = NORTH;  // Initially facing North

// Create an instance of the BMX160 sensor
DFRobot_BMX160 bmx160;
sBmx160SensorData_t Omagn, Ogyro, Oaccel;

// Function to select sensor channel via multiplexer
void selectMuxChannel(uint8_t channel) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// Function to initialize the sensors
void initializeSensor(Adafruit_VL53L1X &sensor, uint8_t channel) {
    selectMuxChannel(channel);
    if (!sensor.begin()) {
        Serial.print("Failed to initialize sensor on channel ");
        Serial.println(channel);
        while (1);
    } else {
        Serial.print("Sensor initialized successfully on channel ");
        Serial.println(channel);
    }
    sensor.startRanging();
    sensor.setTimingBudget(50);
}

// Function to read sensor distance
uint16_t readDistance(Adafruit_VL53L1X &sensor, uint8_t channel) {
    selectMuxChannel(channel);
    if (sensor.dataReady()) {
        uint16_t distance = sensor.distance();
        sensor.clearInterrupt();
        return distance;
    }
    return 9999;  // Return a high value if no reading available
}


// Setup function
void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Existing button
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);  // Add reset button setup
  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LSRB_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RSLB_BUTTON_PIN, INPUT_PULLUP);
  pinMode(OPTIMIZE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_OPTIMIZED_PATH_PIN, INPUT_PULLUP);

  // Initialize BMX160 sensor
  if (!bmx160.begin()) {
    Serial.println("Failed to initialize BMX160 sensor.");
    while (1);
  }

  // Set the sensor's operating mode
  bmx160.setAccelRange(2);  // 2G, 4G, 8G, 16G
  bmx160.setGyroRange(500); // 250DPS, 500DPS, 1000DPS, 2000DPS
  
  // Initialize motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Set the button pin to input with pull-up resistor

  initializeSensor(vl53_center, CENTER_SENSOR_CHANNEL);
  initializeSensor(vl53_right, RIGHT_SENSOR_CHANNEL);
  initializeSensor(vl53_left, LEFT_SENSOR_CHANNEL);
  
  Serial.println("BMX160 sensor initialized.");
  Serial.println("Please calibrate angles using button on pin 40");
  Serial.println("Press button when facing NORTH");
  while (!calibrationComplete) {
    handleCalibration();
    delay(100);
  }
}

void handleCalibration() {
  static unsigned long lastButtonPress = 0;
  const unsigned long debounceDelay = 500; // Debounce time in milliseconds
  
  // Check for button press
  if (digitalRead(CALIBRATION_BUTTON_PIN) == LOW) {
    // Debounce check
    if ((millis() - lastButtonPress) > debounceDelay) {
      // If not in calibration mode, enter it
      if (!calibrationMode) {
        calibrationMode = true;
        calibrationComplete = false;
        calibrationStep = 0;
        Serial.println("\n=== Entering Calibration Mode ===");
        Serial.println("Face NORTH and press button");
      } else {
        // Already in calibration mode, record angles
        float currentAngle = getBMXAngle();
        
        switch (calibrationStep) {
          case 0: // North
            tempNorthAngle = currentAngle;
            Serial.print("North angle recorded: ");
            Serial.println(tempNorthAngle);
            Serial.println("Now turn to WEST and press button");
            calibrationStep++;
            break;
            
          case 1: // West
            tempWestAngle = currentAngle;
            Serial.print("West angle recorded: ");
            Serial.println(tempWestAngle);
            Serial.println("Now turn to SOUTH and press button");
            calibrationStep++;
            break;
            
          case 2: // South
            tempSouthAngle = currentAngle;
            Serial.print("South angle recorded: ");
            Serial.println(tempSouthAngle);
            Serial.println("Now turn to EAST and press button");
            calibrationStep++;
            break;
            
          case 3: // East
            tempEastAngle = currentAngle;
            Serial.print("East angle recorded: ");
            Serial.println(tempEastAngle);
            Serial.println("Calibration complete!");
            
            // Update the actual angle constants
            northAngle = tempNorthAngle;
            westAngle = tempWestAngle;
            southAngle = tempSouthAngle;
            eastAngle = tempEastAngle;
            
            calibrationComplete = true;
            calibrationMode = false;
            calibrationStep = 0;
            Serial.println("=== Exiting Calibration Mode ===\n");
            break;
        }
      }
      lastButtonPress = millis();
    }
  }
}

void resetSystem() {
  // Reset all important variables
  path = "";
  paths = 0;
  endFound = false;
  currentOrientation = NORTH;
  targetAngle = northAngle;
  
  // Stop motors
  stopMotors();
  
  // Print reset message
  Serial.println("System Reset!");
  
  // Small delay to prevent immediate re-trigger
  delay(2000);
  
  // Move forward after reset
  updateOrientationAndTarget('F');
  holdPosition();
  handleAlignment();
  moveForward();
  stopMotors();
  delay(100);
  path += "S";  // Store initial forward movement in path
}

// Function to get the current angle from the BMX160 sensor
float getBMXAngle() {
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  float angle = atan2(Omagn.y, Omagn.x) * 180 / PI;  // Calculate angle from magnetometer data

  // Normalize the angle to the range [0, 360]
  if (angle < 0) angle += 360;
  return angle;
}

// Function to calculate the shortest path difference
float calculateShortestPath(float current, float target) {
  float diff = target - current;

  // Normalize the difference to the range [-180, 180]
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  return diff;
}

// Function to align left with varying speed based on angle difference
void aligningLeft(float angleDiff) {
  int power = map(abs(angleDiff), 0, 180, 30, 160);  // Adjust speed based on the angle difference
  analogWrite(PWMA, power-5);   // Right motor
  analogWrite(PWMB, power-5);   // Left motor
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);   // Right motor backward
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);  // Left motor forward
  Serial.print("Turning left. Angle Diff: ");
  Serial.println(angleDiff);
}

// Function to align right with varying speed based on angle difference
void aligningRight(float angleDiff) {
  int power = map(abs(angleDiff), 0, 180, 30, 160);  // Adjust speed based on the angle difference
  analogWrite(PWMA, power);   // Right motor
  analogWrite(PWMB, power);   // Left motor
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);   // Right motor forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);    // Left motor backward
  Serial.print("Turning right. Angle Diff: ");
  Serial.println(angleDiff);
}

void handleAlignment() {
  float currentAngle = getBMXAngle();
  float angleDiff = calculateShortestPath(currentAngle, targetAngle);

    // Continuously align to the target angle while stationary
    if (abs(angleDiff) > tolerance) {
      Serial.println("Aligning...");
      alignRobot(angleDiff);  // Keep aligning if it's not within tolerance
    } else {
      Serial.println("Aligned. Holding position.");
      stopMotors();  // Once aligned, stop the motors to resist deviation
    }
  }
// Modified alignRobot function to incorporate wall following
void alignRobot(float angleDiff) {
  uint16_t leftDist = readDistance(vl53_left, LEFT_SENSOR_CHANNEL);
  uint16_t rightDist = readDistance(vl53_right, RIGHT_SENSOR_CHANNEL);
  
  // Calculate correction factors
  int bmxCorrection = map(abs(angleDiff), 0, 180, 30, 160);
  
  
}

// Function to stop the motors
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Modified moveForward function with wall following
void moveForward() {
    int baseSpeed = 90;  // Base forward speed
    int leftSpeed, rightSpeed;
    
    
    
    
    // Forward movement with wall following only
    unsigned long startTime = millis();
    while (millis() - startTime < 605) {
        uint16_t leftDist = readDistance(vl53_left, LEFT_SENSOR_CHANNEL);
        uint16_t rightDist = readDistance(vl53_right, RIGHT_SENSOR_CHANNEL);
        
        WallFollowMode mode = determineWallMode(rightDist, leftDist);
        
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed;
        
        // Apply wall following correction based on mode
        switch (mode) {
            case BOTH_WALLS: {
                int lateralDiff = (rightDist - leftDist);
                if (abs(lateralDiff) > WALL_TOLERANCE) {
                    int wallCorrection = map(abs(lateralDiff), 0, 130, 3, 9);
                    if (lateralDiff > 0) {  // Too far from left wall
                        leftSpeed += wallCorrection;
                        rightSpeed -= wallCorrection;
                    } else {  // Too far from right wall
                        leftSpeed -= wallCorrection;
                        rightSpeed += wallCorrection;
                    }
                }
                break;
            }
            
            case RIGHT_WALL: {
                int distError = rightDist - WALL_TARGET_DISTANCE;
                if (abs(distError) > WALL_TOLERANCE) {
                    int wallCorrection = map(abs(distError), 0, 130, 3, 9);
                    if (distError > 0) {  // Too far from wall
                        leftSpeed += wallCorrection;
                        rightSpeed -= wallCorrection;
                    } else {  // Too close to wall
                        leftSpeed -= wallCorrection;
                        rightSpeed += wallCorrection;
                    }
                }
                break;
            }
            
            case LEFT_WALL: {
                int distError = leftDist - WALL_TARGET_DISTANCE;
                if (abs(distError) > WALL_TOLERANCE) {
                    int wallCorrection = map(abs(distError), 0, 130, 3, 9);
                    if (distError > 0) {  // Too far from wall
                        leftSpeed -= wallCorrection;
                        rightSpeed += wallCorrection;
                    } else {  // Too close to wall
                        leftSpeed += wallCorrection;
                        rightSpeed -= wallCorrection;
                    }
                }
                break;
            }
            
            case NO_WALLS: {
                // Use BMX alignment when no walls are present
                float currentAngle = getBMXAngle();
                float angleDiff = calculateShortestPath(currentAngle, targetAngle);
                
                if (abs(angleDiff) > tolerance) {
                    // Apply BMX-based correction
                    int bmxCorrection = map(abs(angleDiff), 0, 180, 0, 90);
                    if (angleDiff > 0) {  // Need to turn right
                        leftSpeed += bmxCorrection;
                        rightSpeed -= bmxCorrection;
                    } else {  // Need to turn left
                        leftSpeed -= bmxCorrection;
                        rightSpeed += bmxCorrection;
                    }
                }
                break;
            }
        }
        
        // Ensure speeds stay within reasonable bounds
        leftSpeed = constrain(leftSpeed, 0, 110);
        rightSpeed = constrain(rightSpeed, 0, 110);
        
        // Apply speeds to motors
        analogWrite(PWMA, rightSpeed);
        analogWrite(PWMB, leftSpeed + 1);
        
        // Keep motors in forward direction
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        
        delay(50);
    }
    
    stopMotors();
    
    // Final BMX alignment using holdPosition
    holdPosition();
    
    
    Serial.println("Forward movement complete");
}

// Helper function to determine wall following mode
WallFollowMode determineWallMode(uint16_t rightDist, uint16_t leftDist) {
    // Create a binary number representing wall presence
    // bit 1: right wall, bit 0: left wall
    uint8_t wallStatus = 0;
    
    if (rightDist < WALL_MAX_DISTANCE) wallStatus |= 0b10;
    if (leftDist < WALL_MAX_DISTANCE) wallStatus |= 0b01;
    
    switch (wallStatus) {
        case 0b11:  // Both walls (right and left)
            return BOTH_WALLS;
            
        case 0b10:  // Only right wall
            return RIGHT_WALL;
            
        case 0b01:  // Only left wall
            return LEFT_WALL;
            
        case 0b00:  // No walls
        default:
            return NO_WALLS;
    }
}
// Modified holdPosition function with tighter tolerance
void holdPosition() {
  unsigned long holdStartTime = millis();
  float currentAngle, angleDiff;
  float holdingTolerance = 2;  // Tighter tolerance for holding position
  
  while (millis() - holdStartTime < 200) {
    currentAngle = getBMXAngle();
    angleDiff = calculateShortestPath(currentAngle, targetAngle);

    if (abs(angleDiff) > holdingTolerance) {
      if (angleDiff > 0) {
        aligningRight(angleDiff);
      } else {
        aligningLeft(angleDiff);
      }
    } else {
      stopMotors();
    }
    delay(5);  // Reduced delay for more frequent corrections
  }
  stopMotors();
}
//robot in reverse for 10 cm
void moveReverse() {
  int reverseSpeed = 40;  // Set reverse speed (40 PWM)
  analogWrite(PWMA, reverseSpeed+4);
  analogWrite(PWMB, reverseSpeed);

  digitalWrite(AIN1, LOW);   // Right motor backward
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);   // Left motor backward
  digitalWrite(BIN2, HIGH);

  Serial.println("Moving backward...");

  delay(500);  // Simulate moving backward for 10 cm (adjust this value based on actual movement distance)

  stopMotors();
}

// Function to handle turning left, right, and U-turn based on current orientation
void updateOrientationAndTarget(char turnDirection) {
  switch (turnDirection) {
    case 'L':  // Turn Left
      if (currentOrientation == NORTH) {
        currentOrientation = WEST;
        targetAngle = westAngle;
      } else if (currentOrientation == WEST) {
        currentOrientation = SOUTH;
        targetAngle = southAngle;
      } else if (currentOrientation == SOUTH) {
        currentOrientation = EAST;
        targetAngle = eastAngle;
      } else if (currentOrientation == EAST) {
        currentOrientation = NORTH;
        targetAngle = northAngle;
      }
      Serial.println("Turning Left");
      break;

    case 'R':  // Turn Right
      if (currentOrientation == NORTH) {
        currentOrientation = EAST;
        targetAngle = eastAngle;
      } else if (currentOrientation == EAST) {
        currentOrientation = SOUTH;
        targetAngle = southAngle;
      } else if (currentOrientation == SOUTH) {
        currentOrientation = WEST;
        targetAngle = westAngle;
      } else if (currentOrientation == WEST) {
        currentOrientation = NORTH;
        targetAngle = northAngle;
      }
      Serial.println("Turning Right");
      break;

    case 'U':  // U-turn
      if (currentOrientation == NORTH) {
        currentOrientation = SOUTH;
        targetAngle = southAngle;
      } else if (currentOrientation == SOUTH) {
        currentOrientation = NORTH;
        targetAngle = northAngle;
      } else if (currentOrientation == EAST) {
        currentOrientation = WEST;
        targetAngle = westAngle;
      } else if (currentOrientation == WEST) {
        currentOrientation = EAST;
        targetAngle = eastAngle;
      }
      Serial.println("U-turn");
      break;
    case 'F':  // Forward (no turn)
      if (currentOrientation == NORTH) {
        currentOrientation = NORTH;
        targetAngle = northAngle;
      } else if (currentOrientation == SOUTH) {
        currentOrientation = SOUTH;
        targetAngle = southAngle;
      } else if (currentOrientation == EAST) {
        currentOrientation = EAST;
        targetAngle = eastAngle;
      } else if (currentOrientation == WEST) {
        currentOrientation = WEST;
        targetAngle = westAngle;
      }
      Serial.println("Forward");
      break;
  }

  
}

void optimizePath() {
    bool changed;
    do {
        changed = false;
        // Store the original path length
        int originalLength = path.length();
        
        // Apply all the specified replacements
        path.replace("RUR", "S");   
        path.replace("LUL", "S");
        path.replace("SUR", "L");
        path.replace("RUS", "L");
        path.replace("RUL", "U");
        path.replace("LUR", "U"); 
        path.replace("LUS", "R");
        path.replace("SUL", "R");
        path.replace("SUS", "U");
        
        // Check if any replacements were made
        if (path.length() != originalLength) {
            changed = true;
        }
        
        // Print current state for debugging
        if (changed) {
            Serial.print("Current path: ");
            Serial.println(path);
        }
        
    } while (changed); // Continue until no more changes can be made
    
    Serial.println("Optimization complete");
    Serial.print("Final path: ");
    Serial.println(path);
}


// Function to follow the optimized path
void followOptimizedPath() {
    for (int i = 0; i < path.length(); i++) {
        char move = path.charAt(i);
        switch (move) {
            case 'L':  // Turn left
                updateOrientationAndTarget('L');
                holdPosition();
                handleAlignment();
                updateOrientationAndTarget('F');
                holdPosition();
                handleAlignment();
                moveForward();
                stopMotors();
                delay(100);
                break;
            case 'S':  // Move straight
                updateOrientationAndTarget('F');
                holdPosition();
                handleAlignment();
                moveForward();
                stopMotors();
                delay(100);
                break;
            case 'R':  // Turn right
                updateOrientationAndTarget('R');
                holdPosition();
                handleAlignment();
                updateOrientationAndTarget('F');
                holdPosition();
                handleAlignment();
                moveForward();
                stopMotors();
                delay(100);
                break;
            case 'U':  // U-turn
                updateOrientationAndTarget('U');
                holdPosition();
                handleAlignment();
                updateOrientationAndTarget('F');
                holdPosition();
                handleAlignment();
                moveForward();
                stopMotors();
                delay(100);
                break;
        }
        delay(100);  // Add a small delay between movements
    }
}

// Main loop
void loop() {
  // Check algorithm selection buttons
  if (digitalRead(LSRB_BUTTON_PIN) == LOW) {
    isLSRB = true;
    Serial.println("Switched to LSRB algorithm");
    delay(500);  // Debounce delay
  }
  if (digitalRead(RSLB_BUTTON_PIN) == LOW) {
    isLSRB = false;
    Serial.println("Switched to RSLB algorithm");
    delay(500);  // Debounce delay
  }

  // Always check for calibration button press
  handleCalibration();
  
  // Only proceed with maze solving if not in calibration mode
  if (calibrationMode || !calibrationComplete) {
    return;
  }

  // Check for reset button press
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    resetSystem();
    return;
  }

  // Check for optimization button press (Button 36)
  if (digitalRead(OPTIMIZE_BUTTON_PIN) == LOW && !optimizationRequested) {
    stopMotors();  // Stop the robot
    optimizationRequested = true;  // Set flag
    Serial.println("Original Path: " + path);
    optimizePath();  // Optimize the current path
    Serial.println("Optimized Path: " + path);
    delay(500);  // Debounce delay
    return;
  }

  // Check for start optimized path button press (Button 37)
  if (digitalRead(START_OPTIMIZED_PATH_PIN) == LOW && optimizationRequested && !startOptimizedPath) {
    startOptimizedPath = true;
    Serial.println("Starting optimized path in 1 second...");
    delay(1000);  // Wait 1 second before starting
    followOptimizedPath();  // Follow the optimized path
    while(true);  // Stop after completing the optimized path
  }

  // Only proceed with normal maze solving if optimization hasn't been requested
  if (!optimizationRequested) {
    // Read sensor distances
    uint16_t leftDistance = readDistance(vl53_left, LEFT_SENSOR_CHANNEL);
    uint16_t frontDistance = readDistance(vl53_center, CENTER_SENSOR_CHANNEL);
    uint16_t rightDistance = readDistance(vl53_right, RIGHT_SENSOR_CHANNEL);

    // Print sensor data for monitoring
    Serial.print("Left: ");
    Serial.print(leftDistance);
    Serial.print(" | Center: ");
    Serial.print(frontDistance);
    Serial.print(" | Right: ");
    Serial.print(rightDistance);

    // Modified path detection based on current algorithm
    if (frontDistance < 20) {
      paths = 5;  // Move Backward (highest priority)
    } 
    else if (isLSRB) {
      // LSRB Algorithm
      if (leftDistance > 270) {
        paths = 1;  // Left turn available
      } else if (frontDistance > 270) {
        paths = 2;  // Move forward available
      } else if (rightDistance > 270) {
        paths = 3;  // Right turn available
      } else if (leftDistance <= 270 && frontDistance <= 200 && rightDistance <= 270) {
        paths = 4;  // No path ahead, U-turn
      }
    } 
    else {
      // RSLB Algorithm
      if (rightDistance > 270) {
        paths = 3;  // Right turn available
      } else if (frontDistance > 270) {
        paths = 2;  // Move forward available
      } else if (leftDistance > 270) {
        paths = 1;  // Left turn available
      } else if (leftDistance <= 270 && frontDistance <= 200 && rightDistance <= 270) {
        paths = 4;  // No path ahead, U-turn
      }
    }

    // Act on the detected paths
    switch (paths) {
      case 1:  // Turn left
        updateOrientationAndTarget('L');
        holdPosition();
        handleAlignment();
        updateOrientationAndTarget('F');
        holdPosition();
        handleAlignment();
        moveForward();
        stopMotors();
        delay(100);
        path += "L";  // Store left turn in path
        break;

      case 2:  // Move forward
        updateOrientationAndTarget('F');
        holdPosition();
        handleAlignment();
        moveForward();
        stopMotors();
        delay(100);
        path += "S";  // Store straight movement in path
        break;

      case 3:  // Turn right
        updateOrientationAndTarget('R');
        holdPosition();
        handleAlignment();
        updateOrientationAndTarget('F');
        holdPosition();
        handleAlignment();
        moveForward();
        stopMotors();
        delay(100);
        path += "R";  // Store right turn in path
        break;

      case 4:  // U-turn
        updateOrientationAndTarget('U');
        holdPosition();
        handleAlignment();
        updateOrientationAndTarget('F');
        holdPosition();
        handleAlignment();
        moveForward();
        stopMotors();
        delay(100);
        path += "U";  // Store U-turn in path
        break;

      case 5:  // Move Backward action
        moveReverse();
        stopMotors();
        delay(100);
        break;
    }

    delay(100);  // Small delay between motor actions
  }
}