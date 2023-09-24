#include <Arduino_AVRSTL.h>

#include "SplitAndMerge.h"

#define IR_OFFSET 11
#define IR_MAX 150 + IR_OFFSET

class ObstacleAvoidance {
 private:
  std::vector<float> ir = {IR_MAX, IR_MAX, IR_MAX, IR_MAX, IR_MAX,
                           IR_MAX, IR_MAX, IR_MAX, IR_MAX};  // Distance from the "center" of IR sensors
  float long_ir = 550;
  int state = 0;
  // 0: straight
  // 1: wall parallel - adjust wall parallel
  // 2: two wall parallel - adjust wall parallel and distance
  // 3: else - find local goal & avoid front obstacle
  float target_speed = 0;
  float target_angle = 0;
  int emergency_stop = 0;
  bool direction_lock = false;
  const int loopcount = 5;
  int adc_history[9][5];
  int long_adc_history[5];
  int prev_turn = 0;
  int get_result_clk = 0;
  int pin_list[9] = {A1, A2, A3, A4, A5, A6, A7, A8, A9};
  int sensor_type[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // 0: long
  SplitAndMerge split_and_merge;
  std::vector<std::vector<Point>> walls;

 public:
  ObstacleAvoidance();

  void ir_callback();

  void get_result();

  void update_state();

  void get_target_angle();

  void follow_wall_single();

  void follow_wall_double();

  void guide_to_empty_space();

  void get_target_speed();

  void control_once(int throttle, int servo);

  void follow_goal();
};
