#include "ObstacleAvoidance.h"

#include <Arduino_AVRSTL.h>

#include "Arduino.h"

#define THROTTLE_PIN 2
#define SERVO_PIN 5

#define IR_OFFSET 11
#define IR_MAX 150 + IR_OFFSET
#define IR_MIN 20 + IR_OFFSET
#define THROTTLE_FORWARD 220
#define THROTTLE_IDLE 0
#define SERVO_LEFT 800
#define SERVO_CENTER 0
#define SERVO_RIGHT -800

ObstacleAvoidance::ObstacleAvoidance() {}

void ObstacleAvoidance::ir_callback() {
  get_result();
  update_state();
  get_target_angle();
  get_target_speed();
  follow_goal();
}

void ObstacleAvoidance::get_result() {
  // read analog value
  for (int i = 0; i < 9; i++) {
    adc_history[i][get_result_clk] = analogRead(pin_list[i]);
  }
  long_adc_history[get_result_clk] = analogRead(A15);

  for (int i = 0; i < 9 + 1; i++) {
    if (i < 9) {
      // kill spike and average
      int sum = 0, x = 0, high = 0, low = 0, avg = 0;
      high = low = adc_history[i][0];
      for (int j = 0; j < loopcount; j++) {
        x = adc_history[i][j];
        sum += x;
        if (x > high) high = x;
        if (x < low) low = x;
      }
      avg = (sum - high - low) / (loopcount - 2);

      // convert voltage to centimeter
      if (sensor_type[i] == 0) {
        ir[i] = float(10650.08 * pow(avg, -0.935) - 3.937 + IR_OFFSET);
      }

      // limit the range of IR distance
      ir[i] = std::max(std::min(ir[i], float(IR_MAX)), float(IR_MIN));
    } else if (i == 9) {
      // kill spike and average
      int sum = 0, x = 0, high = 0, low = 0, avg = 0;
      high = low = long_adc_history[0];
      for (int j = 0; j < loopcount; j++) {
        x = long_adc_history[i][j];
        sum += x;
        if (x > high) high = x;
        if (x < low) low = x;
      }
      avg = (sum - high - low) / (loopcount - 2);

      // convert voltage to centimeter
      long_ir = 1 / ((avg - 1125) / 137500);

      // limit the range of IR distance
      long_ir = std::max(std::min(long_ir, float(550)), float(150));
    }
  }
  walls = split_and_merge.grabData(ir);

  float emergency_thres = float(IR_MIN + 10);
  if (ir[3] < emergency_thres || ir[4] < emergency_thres || ir[5] < emergency_thres) {
    emergency_stop = 1;
  }

  get_result_clk++;
  if (get_result_clk == loopcount) get_result_clk = 0;
  for (int i = 0; i < 9; i++) {
    Serial.print(ir[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void ObstacleAvoidance::update_state() {
  if (emergency_stop) {
    state = -1;
    return;
  }
  if (long_ir == 550) {
    state = 4;
    return;
  }
  if (walls.size() == 0) {
    state = 0;
  } else if (walls.size() == 1) {
    // angle between wall and car
    float angle = 0;
    float y_avg = 0;
    for (int i = 0; i < walls[0].size(); i++) {
      y_avg += walls[0][i].y;
    }
    y_avg /= walls[0].size();
    if (y_avg != 0) {
      angle =
          atan((walls[0][walls[0].size() - 1].y - walls[0][0].y) / (walls[0][walls[0].size() - 1].x - walls[0][0].x)) *
          180.0 / 3.14159;
    }
    if (isnan(angle)) angle = 0;
    if (abs(angle) < 45) {
      state = 1;
    } else {
      state = 1;
    }
  } else if (walls.size() == 2) {
    // angle between wall and car
    float angle1 =
        atan2(walls[0][walls[0].size() - 1].y - walls[0][0].y, walls[0][walls[0].size() - 1].x - walls[0][0].x) *
        180.0 / 3.14159;  // should be the left side
    float angle2 =
        atan2(walls[1][0].y - walls[1][walls[1].size() - 1].y, walls[1][0].x - walls[1][walls[1].size() - 1].x) *
        180.0 / 3.14159;  // should be the right side
    if (isnan(angle1)) angle1 = 0;
    if (isnan(angle2)) angle2 = 0;
    if (abs(angle1) < 45 && abs(angle2) < 45 && walls[0][0].y < 0 && walls[1][0].y > 0) {
      state = 2;
    } else {
      state = 3;
    }
  } else {
    state = 3;
  }
}

void ObstacleAvoidance::get_target_angle() {
  target_angle = 0;
  switch (state) {
    case 0:
      target_angle = 0;
      break;
    case 1:
      follow_wall_single();
      break;
    case 2:
      follow_wall_double();
      break;
    case 3:
      guide_to_empty_space();
      break;
    case 4:
      target_angle = 0;
      break;
  }
}

void ObstacleAvoidance::follow_wall_single() {
  // angle between wall and car
  float angle = 0;
  float dist_diff = 0;
  float y_avg = 0;
  for (int i = 0; i < walls[0].size(); i++) {
    y_avg += walls[0][i].y;
  }
  y_avg /= walls[0].size();
  if (y_avg != 0) {
    angle =
        atan((walls[0][walls[0].size() - 1].y - walls[0][0].y) / (walls[0][walls[0].size() - 1].x - walls[0][0].x)) *
        180.0 / 3.14159;
  }
  if (isnan(angle)) angle = 0;
  if (y_avg > 0) {
    dist_diff = walls[0][0].y - 100.0;
  } else if (y_avg < 0) {
    dist_diff = walls[0][walls[0].size() - 1].y + 100.0;
  }  // else if the wall is only at front, just go straight by doing nothing

  target_angle = angle + dist_diff;
}

void ObstacleAvoidance::follow_wall_double() {
  float angle1 =
      atan((walls[0][walls[0].size() - 1].y - walls[0][0].y) / (walls[0][walls[0].size() - 1].x - walls[0][0].x)) *
      180.0 / 3.14159;  // should be on the left side
  float angle2 =
      atan((walls[1][0].y - walls[1][walls[1].size() - 1].y) / (walls[1][0].x - walls[1][walls[1].size() - 1].x)) *
      180.0 / 3.14159;  // should be on the right side
  if (isnan(angle1)) angle1 = 0;
  if (isnan(angle2)) angle2 = 0;
  float ave_angle = (angle1 + angle2) * 0.5;
  float dist_diff = (walls[0][0].y + walls[1][walls[1].size() - 1].y) * 0.5;
  target_angle = ave_angle + dist_diff;
}

void ObstacleAvoidance::guide_to_empty_space() {
  if (ir[3] == IR_MAX && ir[4] == IR_MAX && ir[5] == IR_MAX) {
    // if front three irs are all max, go straight
    target_angle = 0;
    return;
  }

  float max_distance = 0;
  std::vector<int> max_group;

  // find the IR sensors with furthest distance
  for (int i = 0; i < 9; i++) {
    if (ir[i] > max_distance) {
      max_distance = ir[i];
      max_group.clear();
      max_group.push_back(i);
    } else if (ir[i] == max_distance) {
      max_group.push_back(i);
    }
  }
  // print the index of IR sensors with furthest distance
  //   for (int i = 0; i < max_group.size(); i++) {
  //     std::printf("%d  ", max_group[i]);
  //   }
  //   std::printf("\n");

  if (max_group.size() > 0) {
    // group the consecutive numbers. e.g. (2,3), (5,6,7,8)
    std::vector<std::vector<int>> groups;
    std::vector<int> group;

    group.push_back(max_group[0]);

    for (int i = 1; i < max_group.size(); i++) {
      if (max_group[i] == group.back() + 1) {
        group.push_back(max_group[i]);
      } else {
        groups.push_back(group);
        group.clear();
        group.push_back(max_group[i]);
      }
    }
    groups.push_back(group);

    // find the biggest group
    int max_group_size = 0;
    int max_group_idx = 0;
    for (int i = 0; i < groups.size(); i++) {
      std::vector<int> cur_group = groups[i];
      int cur_group_size = cur_group.size();
      if (cur_group_size > max_group_size) {
        max_group_size = cur_group_size;
        max_group_idx = i;
      } else if (cur_group_size == max_group_size) {
        std::vector<int> max_adjacent_idx;
        std::vector<int> cur_adjacent_idx;
        float max_group_adjacent_distance = 0;
        float cur_group_adjacent_distance = 0;
        if (groups[max_group_idx][0] != 0) max_adjacent_idx.push_back(groups[max_group_idx][0] - 1);
        if (groups[max_group_idx][groups[max_group_idx].size() - 1] != 8)
          max_adjacent_idx.push_back(groups[max_group_idx][groups[max_group_idx].size() - 1] + 1);
        if (cur_group[0] != 0) cur_adjacent_idx.push_back(cur_group[0] - 1);
        if (cur_group[cur_group.size() - 1] != 8) cur_adjacent_idx.push_back(cur_group[cur_group.size() - 1] + 1);

        for (int j = 0; j < max_adjacent_idx.size(); j++) {
          max_group_adjacent_distance += ir[max_adjacent_idx[j]];
        }
        max_group_adjacent_distance /= max_adjacent_idx.size();

        for (int j = 0; j < cur_adjacent_idx.size(); j++) {
          cur_group_adjacent_distance += ir[cur_adjacent_idx[j]];
        }
        cur_group_adjacent_distance /= cur_adjacent_idx.size();

        if (cur_group_adjacent_distance > max_group_adjacent_distance) {
          max_group_size = cur_group_size;
          max_group_idx = i;
        }

        // max_group_idx = ir[3] > ir[5] ? 0 : 1;
        // if (abs(ir[3] - ir[5]) < 5) max_group_idx = prev_turn;
        // prev_turn = max_group_idx;
      }
    }

    // find the center of the biggest group
    float max_group_center = 0;
    for (int i = 0; i < groups[max_group_idx].size(); i++) {
      max_group_center += groups[max_group_idx][i];
    }
    max_group_center /= groups[max_group_idx].size();
    target_angle = (max_group_center - 4) * 22.5;  // multiply 30 instead of 22.5
  }
}

void ObstacleAvoidance::get_target_speed() {
  if (state == -1) {
    target_angle = 0;
    target_speed = 0;
    return;
  } else if (state == 4) {
    target_angle = 0;
    target_speed = 1.5;
    return;
  }
  float angle_rad = target_angle * 3.14159 / 180.0;
  float a = 1.25, b = 1.0;
  float r = (a * b) / sqrt(b * b * cos(angle_rad) * cos(angle_rad) + a * a * sin(angle_rad) * sin(angle_rad));
  target_speed = r;
}

void ObstacleAvoidance::control_once(int throttle, int servo) {
  analogWrite(THROTTLE_PIN, throttle + 3000);
  analogWrite(SERVO_PIN, servo + 3003);
}

void ObstacleAvoidance::follow_goal() {
  target_angle = std::min(std::max(target_angle, float(-90.0)), float(90.0));
  int throttle = int(target_speed * THROTTLE_FORWARD);
  int servo =
      int(SERVO_LEFT * target_angle /
          90.0);  // left turn is positive in code, but negative in rc car
  control_once(throttle, servo);
//  Serial.print("state: ");
//  Serial.print(state);
//  Serial.print("\t");
//  Serial.print("speed: ");
//  Serial.print(target_speed);
//  Serial.print("\t");
//  Serial.print("angle: ");
//  Serial.print(target_angle);
//  Serial.print("\n");
}
