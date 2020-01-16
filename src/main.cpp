#include <mbed.h>
#define DEBOUNCE_TIME 200
#define CAPTURE_NUM 500

#define DT 0.01

#define KP 0.01
#define TI 1
#define TD 1

#define P_SEIGYO 0
#define PI_SEIGYO 1
#define PID_SEIGYO 2

float g_SV;  // 目標値
float g_PV;  // 制御量

float g_MV0;  // 現在の操作量 MV(n)
float g_MV1;  // 前回の操作量 MV(n-1)
float g_MVd;  // 操作量の差 ⊿MV(n)

float g_ER0;  // 現在の偏差
float g_ER1;  // 1つ前の偏差
float g_ER2;  // 2つ前の偏差

long g_IREval;     // ロータリーエンコーダの値
long g_IREold[5];  // 過去のロータリーエンコーダの値

uint8_t pid_select;

Ticker timer_10ms;
Ticker timer_dt;
Ticker timer_debounce;  //チャタリング対策用タイマー

DigitalOut check_encoder_pin(D13);
DigitalOut led_green(D8);  //緑色LED
DigitalOut led_red(D10);   //赤色LED

PwmOut md_in1(D6);  //モータドライバのIN1に接続されている。
PwmOut md_in2(D5);  //モータドライバのIN2に接続されている。

InterruptIn button(USER_BUTTON);
InterruptIn input_A(D2);
InterruptIn input_B(D3);

float data[CAPTURE_NUM] = {0};
uint16_t i = 0;  // i回目の測定
uint16_t time_until_before_button_pushed =
    0;  //前回にボタンを押されてからの時間(ms)

bool flag_capture = false;   //キャプチャをするかどうか
bool flag_finished = false;  //キャプチャが終了したかどうか

//関数プロトタイプ宣言
void motor_pwm(float);
void motor_stop();
void motor_cw(float);
void motor_ccw(float);
void motor_pwm(float);
void init_func();
float smooth_diff_5();
float cal_pid();

void time_update_pid() {
  g_IREold[4] = g_IREold[3];
  g_IREold[3] = g_IREold[2];
  g_IREold[2] = g_IREold[1];
  g_IREold[1] = g_IREold[0];
  g_IREold[0] = g_IREval;

  g_PV = smooth_diff_5();
  g_ER2 = g_ER1;
  g_ER1 = g_ER0;
  g_ER0 = g_SV - g_PV;

  g_MVd = cal_pid();

  g_MV0 = g_MV1 + g_MVd;
  g_MV1 = g_MV0;

  motor_pwm(g_MV0);
}

float smooth_diff_5(void) {
  float frequency =
      (float)((-2 * g_IREold[4]) + (-1 * g_IREold[3]) + (0 * g_IREold[2]) +
              (1 * g_IREold[1]) + (2 * g_IREold[0])) /
      (float)(10 * 10 * 0.001);

  return ((frequency / 12) / 27) * 2 * 3.14;
}

float sigma(void) {
  static float answer = 0;
  answer += g_ER0 + g_ER1;
  return answer;
}

float cal_pid(void) {
  float mv = 0;

  switch (pid_select) {
    case P_SEIGYO:
      mv = KP * g_ER0;
      break;
    case PI_SEIGYO:
      g_MV1 =
          KP * (g_ER1 + DT / (2 * TI) * sigma());
      g_MVd = KP * ((g_ER0 - g_ER1) + DT / (2 * TI) * (g_ER0 + g_ER1) +
                    TD / DT * (g_ER0 - 2 * g_ER1 + g_ER2));
      mv = g_MV1 + g_MVd;
      break;
    case PID_SEIGYO:
      g_MV1 =
          KP * (g_ER1 + DT / (2 * TI) * sigma() + TD / DT * (g_ER1 - g_ER2));
      g_MVd = KP * ((g_ER0 - g_ER1) + DT / (2 * TI) * (g_ER0 + g_ER1) +
                    TD / DT * (g_ER0 - 2 * g_ER1 + g_ER2));
      mv = g_MV1 + g_MVd;
      break;
  }

  return mv;
}

/*
 モータを動作させる関数。
 param : _per デューティ比
 return : void
*/
void motor_pwm(float _per) {
  if (_per > 0) {
    motor_cw(_per);
  } else if (_per < 0) {
    motor_ccw(abs(_per));
  } else {
    motor_stop();
  }
}

/*
 モータを停止する関数
*/
void motor_stop() {
  md_in1 = 1;
  md_in2 = 1;
}

/*
 モータを正転（Clockwise）させる関数
 param: speed モータの速度(範囲0~1,1で最高速度)
*/
void motor_cw(float speed) {
  led_green = 0;
  led_red = 1;
  if (speed > 1) speed = 1;  //速度が1以上で指定された場合、1にセットする。
  speed = 1 - speed;
  md_in1.write(abs(speed));
  md_in2.write(1);
}

/*
 モータを逆転（Counter Clockwise）させる関数
 param: speed モータの速度(範囲0~1,1で最高速度)
 return : void
*/
void motor_ccw(float speed) {
  led_green = 1;
  led_red = 0;
  if (speed > 1) speed = 1;  //速度が1以上で指定された場合、1にセットする。
  speed = 1 - speed;
  md_in1.write(1);
  md_in2.write(speed);
}

void init_g_var(void) {
  g_SV = 0;
  g_PV = 0;

  g_MV0 = 0;
  g_MV1 = 0;
  g_MVd = 0;

  g_ER0 = 0;
  g_ER1 = 0;
  g_ER2 = 0;

  g_IREval = 0;
  for (int i = 0; i < 5; i++) {
    g_IREold[i] = 0;
  }

  pid_select = P_SEIGYO;
}

/*
チャタリング対策用関数。1msごとにtickerによって呼び出され、
time_until_before_button_pushedが加算される。
*/
void button_timer() { time_until_before_button_pushed++; }

/*
 ボタン割り込みが発生すると呼び出される関数
 状態遷移のみを実行する。
*/
void init_func() {
  if (time_until_before_button_pushed < DEBOUNCE_TIME) return;
  time_until_before_button_pushed = 0;

  g_SV = 6;
  flag_capture = true;
}

void re_trigger(void) {
  check_encoder_pin = !check_encoder_pin;
  if (input_B) {
    g_IREval++;
  } else {
    g_IREval--;
  }
}

void reset_input_a(void) { check_encoder_pin = 0; }

void capture_count() {
  if (!flag_capture) return;
  if (i >= CAPTURE_NUM) {
    flag_capture = false;
    flag_finished = true;
    return;
  }
  if (i < CAPTURE_NUM) {
    data[i] = g_PV;
    i++;
  }
}

int main() {
  init_g_var();
  timer_dt.attach_us(&time_update_pid, 10 * 1000);
  timer_10ms.attach_us(&capture_count, 10 * 1000);
  timer_debounce.attach_us(&button_timer, 1 * 1000);  // 1msごとに呼び出す。
  button.rise(&init_func);
  input_A.rise(&re_trigger);
  input_A.fall(&reset_input_a);
  // while (1) {
  //   printf("g_MV0=%f\t", g_MV0);
  //   printf("g_PV=%f\t\n", g_PV);

  //   wait_us(100 * 1000);
  // }
  while (!flag_finished) {
    wait_us(1);  // <-ないとフリーズした…
  }
  printf("capture finished.\n");

  for (int j = 0; j < CAPTURE_NUM; j++) {
    printf("%d,%f\n", j, data[j]);
  }
}