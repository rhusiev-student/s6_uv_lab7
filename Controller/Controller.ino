#include <Arduino.h>
#include "gamepad.hpp"
#include "motor.hpp"
#include "movement_logic.hpp"
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

constexpr uint32_t TIME_ROTATE_90 = 10;
constexpr uint32_t TIME_UNWIND = 10;
constexpr uint32_t TIME_GO_X = 500;
constexpr uint32_t TIME_GO_Y = 500;

constexpr int32_t GOAL_DISTANCE = 20000;

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

  if (goal_reached) {
    Motor_Move(0, 0, 0, 0);
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
    return;
  }

  if (rotating_to_x_plus) {
    if (rotating_to_x_plus < TIME_ROTATE_90) {
      Motor_Move(900, 900, -700, -700);
      rotating_to_x_plus++;
    } else {
      rotating_to_x_plus = 0;
      unwinding_to_x_plus_after = 1;
    }
    return;
  }
  if (rotating_to_x_minus) {
    if (rotating_to_x_plus < TIME_ROTATE_90) {
      Motor_Move(-700, -700, 900, 900);
      rotating_to_x_minus++;
    } else {
      rotating_to_x_minus = 0;
      unwinding_to_x_minus_after = 1;
    }
    return;
  }
  if (rotating_to_y_plus) {
    if (rotating_to_y_plus < TIME_ROTATE_90) {
      Motor_Move(-700, -700, 900, 900);
      rotating_to_y_plus++;
    } else {
      rotating_to_y_plus = 0;
      unwinding_to_y_plus_after = 1;
    }
    return;
  }
  if (rotating_to_y_minus) {
    if (rotating_to_y_minus < TIME_ROTATE_90) {
      Motor_Move(900, 900, -700, -700);
      rotating_to_y_minus++;
    } else {
      rotating_to_y_minus = 0;
      unwinding_to_y_minus_after = 1;
    }
    return;
  }

  if (since_last_rotation > 0) {
    if (since_last_rotation == TIME_GO_X) {
      since_last_rotation = 0;
      unwinding_to_y_plus = 1;
    } else {
      Motor_Move(700, 700, 700, 700);
      went_x++;
      since_last_rotation++;
    }
    return;
  }
  if (since_last_rotation < 0) {
    if (went_x == 0 || Get_Sonar() < 35) {
      since_last_rotation = 0;
      unwinding_to_y_minus = 1;
      return;
    }
    since_last_rotation--;
    went_x--;
    return;
  }

  if (went_y == GOAL_DISTANCE) {
    goal_reached = true;
    return;
  }
  if (went_y > GOAL_DISTANCE * 1.5) {
    for (int8_t k = 0; k < 2; ++k) {
      for (int32_t j = 0; j < TIME_ROTATE_90; ++j) {
        Motor_Move(900, 900, -700, -700);
        delay(single_delay);
      }
      for (int32_t j = 0; j < TIME_UNWIND; ++j) {
        Motor_Move(0, 0, 0, 0);
        delay(single_delay);
      }
    }
    for (int32_t j = went_y; j > GOAL_DISTANCE; --j) {
      Motor_Move(700, 700, 700, 700);
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

  Motor_Move(700, 700, 700, 700);
  went_y++;

  float distance = Get_Sonar();
  if (distance < 50) {
    unwinding_to_x_plus = 1;
    since_last_bypass = 0;
    return;
  }

  i++; // Will wrap to 0 after large number
  delay(single_delay);
}
