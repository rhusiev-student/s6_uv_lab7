#include <Arduino.h>
#include "motor.hpp"
#include "delays.hpp"
#include "leds.hpp"
#include "meow.hpp"

void reset() {
  Motor_Move(0, 0, 0, 0);
  Servo_1_Angle(90);
  Servo_2_Angle(0);
  off_front_left();
  off_front_right();
  off_back_left();
  off_back_right();
}

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(255);

  PCA9685_Setup();
  reset();
  Buzzer_Setup();
  Ultrasonic_Setup();
  blocking_blink(1);
  // Buzzer_Alert(2, 1);
}

constexpr uint32_t TIME_ROTATE_90 = 72;
constexpr uint32_t TIME_ROTATE_90_LEFT = 57;
constexpr uint32_t TIME_UNWIND = 100;
constexpr uint32_t TIME_GO_X = 100;
constexpr uint32_t TIME_GO_Y = 100;

constexpr uint32_t SPEED = 1000;

constexpr int32_t GOAL_DISTANCE = 600;

constexpr uint32_t SONAR_STOP = 55;

void loop() {
  static uint32_t i = 0;
  static int32_t went_x = 0;
  static int32_t went_y = 0;
  static int32_t since_last_rotation = 0;
  static uint32_t since_last_bypass = 0;
  static uint32_t rotating_to_x_plus = 0;
  static uint32_t rotating_to_x_minus = 0;
  static uint32_t rotating_to_y_plus = 0;
  static uint32_t rotating_to_y_minus = 0;
  static uint32_t unwinding_to_x_plus = 0;
  static uint32_t unwinding_to_x_plus_after = 0;
  static uint32_t unwinding_to_x_minus = 0;
  static uint32_t unwinding_to_x_minus_after = 0;
  static uint32_t unwinding_to_y_plus = 0;
  static uint32_t unwinding_to_y_plus_after = 0;
  static uint32_t unwinding_to_y_minus = 0;
  static uint32_t unwinding_to_y_minus_after = 0;
  static bool goal_reached = false;
  i++; // Will wrap to 0 after large number
  delay(single_delay);
  float distance = Get_Sonar();

  off_front_left();
  off_front_right();
  off_back_left();
  off_back_right();
  ledcWriteTone(BUZZER_CHN, 0);

  if (goal_reached) {
    Motor_Move(0, 0, 0, 0);
    front();
    back();
    strip.show();
    return;
  }

  if (unwinding_to_x_plus) {
    if (unwinding_to_x_plus < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_x_plus++;
    } else {
      unwinding_to_x_plus = 0;
      rotating_to_x_plus = 1;
    }
    brake();
    strip.show();
    return;
  }
  if (unwinding_to_x_minus) {
    if (unwinding_to_x_minus < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_x_minus++;
    } else {
      unwinding_to_x_minus = 0;
      rotating_to_x_minus = 1;
    }
    brake();
    strip.show();
    return;
  }
  if (unwinding_to_y_plus) {
    if (unwinding_to_y_plus < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_y_plus++;
    } else {
      unwinding_to_y_plus = 0;
      rotating_to_y_plus = 1;
    }
    brake();
    strip.show();
    return;
  }
  if (unwinding_to_y_minus) {
    if (unwinding_to_y_minus < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_y_minus++;
    } else {
      unwinding_to_y_minus = 0;
      rotating_to_y_minus = 1;
    }
    brake();
    strip.show();
    return;
  }
  if (unwinding_to_x_plus_after) {
    if (unwinding_to_x_plus_after < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_x_plus_after++;
    } else {
      unwinding_to_x_plus_after = 0;
      since_last_rotation = 1;
    }
    front();
    strip.show();
    return;
  }
  if (unwinding_to_x_minus_after) {
    if (unwinding_to_x_minus_after < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_x_minus_after++;
    } else {
      unwinding_to_x_minus_after = 0;
      since_last_rotation = -1;
    }
    front();
    strip.show();
    return;
  }
  if (unwinding_to_y_plus_after) {
    if (unwinding_to_y_plus_after < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_y_plus_after++;
    } else {
      unwinding_to_y_plus_after = 0;
      if (went_x != 0) {
        since_last_bypass = 1;
      }
    }
    front();
    strip.show();
    return;
  }
  if (unwinding_to_y_minus_after) {
    if (unwinding_to_y_minus_after < TIME_UNWIND) {
      Motor_Move(0, 0, 0, 0);
      unwinding_to_y_minus_after++;
    } else {
      unwinding_to_y_minus_after = 0;
      if (went_x != 0) {
        since_last_bypass = 1;
      }
    }
    front();
    strip.show();
    return;
  }

  if (rotating_to_x_plus) {
    if (rotating_to_x_plus < TIME_ROTATE_90) {
      Motor_Move(SPEED, SPEED, -SPEED, -SPEED);
      rotating_to_x_plus++;
    } else {
      rotating_to_x_plus = 0;
      unwinding_to_x_plus_after = 1;
    }
    turn_right(false, false, false, i, false);
    strip.show();
    return;
  }
  if (rotating_to_x_minus) {
    if (rotating_to_x_minus < TIME_ROTATE_90_LEFT) {
      Motor_Move(-SPEED, -SPEED, SPEED, SPEED);
      rotating_to_x_minus++;
    } else {
      rotating_to_x_minus = 0;
      unwinding_to_x_minus_after = 1;
    }
    turn_left(false, false, false, i, false);
    strip.show();
    return;
  }
  if (rotating_to_y_plus) {
    if (rotating_to_y_plus < TIME_ROTATE_90_LEFT) {
      Motor_Move(-SPEED, -SPEED, SPEED, SPEED);
      rotating_to_y_plus++;
    } else {
      rotating_to_y_plus = 0;
      unwinding_to_y_plus_after = 1;
    }
    turn_left(false, false, false, i, false);
    strip.show();
    return;
  }
  if (rotating_to_y_minus) {
    if (rotating_to_y_minus < TIME_ROTATE_90) {
      Motor_Move(SPEED, SPEED, -SPEED, -SPEED);
      rotating_to_y_minus++;
    } else {
      rotating_to_y_minus = 0;
      unwinding_to_y_minus_after = 1;
    }
    turn_right(false, false, false, i, false);
    strip.show();
    return;
  }
  strip.show();

  if (since_last_rotation > 0) {
    if (since_last_rotation == TIME_GO_X) {
      since_last_rotation = 0;
      unwinding_to_y_plus = 1;
    } else {
      Motor_Move(SPEED, SPEED, SPEED, SPEED);
      went_x++;
      since_last_rotation++;
      // ledcWriteTone(BUZZER_CHN, 1500);
      Serial.println(went_x);
    }
    return;
  }
  if (since_last_rotation < 0) {
    if (went_x == 0 || distance < SONAR_STOP) {
      since_last_rotation = 0;
      unwinding_to_y_minus = 1;
    } else {
      Motor_Move(SPEED, SPEED, SPEED, SPEED);
      since_last_rotation--;
      went_x--;
      // ledcWriteTone(BUZZER_CHN, 700);
      Serial.println(went_x);
    }
    return;
  }

  if (went_y >= GOAL_DISTANCE && went_y < GOAL_DISTANCE * 1.2) {
    goal_reached = true;
    return;
  }
  if (went_y > GOAL_DISTANCE * 1.5) {
    for (int8_t k = 0; k < 2; ++k) {
      for (int32_t j = 0; j < TIME_ROTATE_90; ++j) {
        Motor_Move(SPEED, SPEED, -SPEED, -SPEED);
        delay(single_delay);
      }
      for (int32_t j = 0; j < TIME_UNWIND; ++j) {
        Motor_Move(0, 0, 0, 0);
        delay(single_delay);
      }
    }
    for (int32_t j = went_y; j > GOAL_DISTANCE; --j) {
      Motor_Move(SPEED, SPEED, SPEED, SPEED);
      delay(single_delay);
    }
    goal_reached = true;
    return;
  }

  if (since_last_bypass) {
    if (since_last_bypass == TIME_GO_Y) {
      since_last_bypass = 0;
      unwinding_to_x_minus = 1;
      return;
    }
    since_last_bypass++;
  }

  Motor_Move(SPEED, SPEED, SPEED, SPEED);
  went_y++;
  
  if (distance < SONAR_STOP) {
    unwinding_to_x_plus = 1;
    since_last_bypass = 0;
    return;
  }
}
