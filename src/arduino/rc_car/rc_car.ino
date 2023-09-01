#include <Arduino_AVRSTL.h>

#define THROTTLE_PIN 2
#define SERVO_PIN 5

#define IR_MAX 80.0
#define IR_MIN 20.0
#define THROTTLE_FORWARD 200
#define THROTTLE_IDLE 0
#define SERVO_LEFT -800
#define SERVO_CENTER 0
#define SERVO_RIGHT 800

#define FRONT_LIMIT 80
#define NOISE_ALLOWANCE 5
#define MAX_THROTTLE 3250

int pin_list[11] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};
int sensor_type[11] = {1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};  // 0: long, 1: short
int get_result_clk = 0;
const int loopcount = 5;  // how many data to save for each sensor in killspike
int adc_history[11][loopcount];
float ir[11] = {70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0};  // array for IR distance values

float local_goal_speed = 0;
float local_goal_angle = 0;
int emergency_stop = 0;
int prev_turn = 0;
// analogWrite(PinNum, desired us / 2 -> (Ex: To get 900us -> 1800))
// Analog Read 10 bit (based 5V), so result value 1023 is 5V

// THR: 2000(Reverse) ~ 3000(Stay) ~ 4094(Full)
// SER: 2240(R) ~ 3003(Center) ~ 3858(L)

void setup() {
  // RTM_TimerCalc 1.40, RuntimeMicro.com
  // Timer-3 16-bit, Mode-14 Fast, Top=ICR
  // 50 Hz Frequency, Clock is 16 MHz

  TCCR3B = 0x18;  // 0001 1000, Disable Timer
  TCCR3A = 0xA2;  // 1010 0010

  ICR3 = 40000 - 1;
  OCR3A = (int)(ICR3 * 0.25);
  OCR3B = (int)(ICR3 * 0.50);
  TCNT3 = 0x0;

  pinMode(2, OUTPUT);  // OC3b
  pinMode(5, OUTPUT);  // OC3a

  TCCR3B |= 2;  // Prescale=8, Enable Timer

  // For Serial print
  Serial.begin(115200);

  for (int i = 0; i < 20000; i++) {
    analogWrite(2, 3000);
  }
  emergency_stop = 0;
}

void loop() {
  get_result();
  find_local_goal();
  adjust_wall_distance();
  adjust_wall_parallel();
  get_local_goal_speed();
  follow_goal();
}

void control_once(int throttle, int servo) {
  analogWrite(THROTTLE_PIN, throttle + 3000);
  analogWrite(SERVO_PIN, servo + 3003);
}

void get_result() {
  // read analog value
  for (int i = 0; i < 11; i++) {
    adc_history[i][get_result_clk] = analogRead(pin_list[i]);
  }

  for (int i = 0; i < 11; i++) {
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
      ir[i] = float(10650.08 * pow(avg, -0.935) - 3.937);
    } else {
      ir[i] = float((27.61 / (avg * 5000.0 / 1023.0 - 0.1696)) * 1000.0);
    }

    // limit the range of IR distance
    if (i == 5)
      ir[i] = std::max(std::min(ir[i], float(FRONT_LIMIT)), float(IR_MIN));
    else
      ir[i] = std::max(std::min(ir[i], float(IR_MAX)), float(IR_MIN));
  }

  if (ir[5] < 30 || ir[3] < 24 || ir[7] < 24 || ir[6] < 30 || ir[4] < 30) {
    emergency_stop += 1;
  } else if (emergency_stop < 2) {
    emergency_stop = 0;
  }

  Serial.print(emergency_stop);
  Serial.println();

  get_result_clk++;
  if (get_result_clk == loopcount) get_result_clk = 0;
}

void find_local_goal() {
  if (ir[4] == IR_MAX && ir[5] == IR_MAX && ir[6] == IR_MAX) {
    // if front three irs are all max, go straight
    local_goal_angle = 0;
    return;
  }

  if (ir[4] < IR_MAX && ir[5] < IR_MAX && ir[6] < IR_MAX) {
    if (ir[1] > ir[9]) {
      local_goal_angle = -90;
      Serial.print("a");
    } else if (ir[1] < ir[9]) {
      local_goal_angle = 90;
      Serial.print("b");
    }

    local_goal_angle = ir[4] > ir[6] ? -90 : 90;
    return;
  }

  float max_distance = 0;
  std::vector<int> max_group;

  // find the IR sensors with furthest distance
  for (int i = 1; i < 10; i++) {
    if (ir[i] > max_distance) {
      max_distance = ir[i];
      max_group.clear();
      max_group.push_back(i);
    } else if (ir[i] >= max_distance - NOISE_ALLOWANCE) {
      max_group.push_back(i);
    }
  }
  // print the index of IR sensors with furthest distance
  //  for (int i = 0; i < max_group.size(); i++) {
  //    Serial.print(max_group[i]);
  //    Serial.print("  ");
  //  }
  //  Serial.println();

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
    //    Serial.print("number of groups: ");
    //    Serial.print(groups.size());
    //    Serial.println();

    // find the biggest group
    int max_group_size = 0;
    int max_group_idx = 0;
    for (int i = 0; i < groups.size(); i++) {
      if (groups[i].size() > max_group_size) {
        max_group_size = groups[i].size();
        max_group_idx = i;
      }
    }

    //    Serial.print("biggest group index: ");
    //    Serial.print(max_group_idx);
    //    Serial.println();

    // find the center of the biggest group
    int max_group_center = 0;
    for (int i = 0; i < groups[max_group_idx].size(); i++) {
      max_group_center += groups[max_group_idx][i];
    }
    max_group_center /= groups[max_group_idx].size();
    local_goal_angle = (max_group_center - 5) * 22.5;

    //    Serial.print("local_goal_angle: ");
    //    Serial.print(local_goal_angle);
    //    Serial.println();
  }
}

void adjust_wall_distance() {
  if ((ir[1] == IR_MAX) != (ir[9] == IR_MAX)) return;  // better to follow wall
  float diff = (ir[1] - ir[9]) * 0.5;
  local_goal_angle -= diff;
  //  Serial.print("distance adjust amount: ");
  //  Serial.print(diff);
  //  Serial.println();
}

void adjust_one_side_parallel(int first, int second, int third, int direction) {
  if (ir[first] == IR_MAX || ir[second] == IR_MAX || ir[third] == IR_MAX || ir[first] == IR_MIN ||
      ir[second] == IR_MIN || ir[third] == IR_MIN)
    return;  // cannot determine car-wall angle

  float a = ir[first], b1 = ir[second], b2 = ir[third];
  float c1, c2;
  float theta = 22.5 * 3.14159 / 180.0;

  // triangle between ir[0] and ir[1]
  c1 = sqrt(a * a + b1 * b1 - 2 * a * b1 * cos(theta));
  float alpha = acos((a * a + c1 * c1 - b1 * b1) / (2 * a * c1)) * 180.0 / 3.14159;

  // triangle between ir[0] and ir[2]
  c2 = sqrt(a * a + b2 * b2 - 2 * a * b2 * cos(theta * 2));
  float beta = acos((a * a + c2 * c2 - b2 * b2) / (2 * a * c2)) * 180.0 / 3.14159;

  float ave_angle;

  if (abs(alpha - beta) < 20) {
    ave_angle = (alpha + beta) * 0.5;
  } else {
    ave_angle = alpha;
  }

  float diff_angle = direction * (ave_angle - 90) * 0.5;  // left wall: +, right wall: -
  local_goal_angle -= diff_angle;
  //  Serial.print("parallel adjust amount: ");
  //  Serial.print(diff_angle);
  //  Serial.println();
}

void adjust_wall_parallel() {
  adjust_one_side_parallel(1, 2, 3, 1);
  adjust_one_side_parallel(9, 8, 7, -1);
}

void avoid_front_wall() {}

void get_local_goal_speed() {
  float angle_rad = local_goal_angle * 3.14159 / 180.0;
  float a = 1.25, b = 1.0;
  float r = (a * b) / sqrt(b * b * cos(angle_rad) * cos(angle_rad) + a * a * sin(angle_rad) * sin(angle_rad));
  local_goal_speed = r;
}

void follow_goal() {
  local_goal_angle = std::min(std::max(local_goal_angle, float(-90.0)), float(90.0));
  int throttle = int(local_goal_speed * THROTTLE_FORWARD);
  int servo = SERVO_LEFT * local_goal_angle / 90.0;
  //  Serial.print(emergency_stop);
  //  Serial.println();
  if (emergency_stop >= 2) {
    control_once(THROTTLE_IDLE, servo);
  } else {
    control_once(throttle, servo);
  }

  //  Serial.print("final_goal_speed: ");
  //  Serial.print(local_goal_speed);
  //  Serial.println();
  //  Serial.print("final_goal_angle: ");
  //  Serial.print(local_goal_angle);
  //  Serial.println();
}
