#include <M5StickC.h>
#include "BalaC.h"

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float target = -96.5;
float P, I, D, preP;

float power = 0;
float dt, preTime;

bool first = true;
bool started = false;

float roll, pitch, yaw;
float standard;
float now;
float loopfreq;

void drawScreen() {
 M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("S:%6.1f\n", target);
  M5.Lcd.printf("roll:%6.1f\n", roll);
  M5.Lcd.printf("now:%4.1f\n", target - roll);
  //    M5.Lcd.printf("Power:%5.1f\n", power);
  //    M5.Lcd.printf("Kp:%5.1f\n", Kp);
  //    M5.Lcd.printf("Ki:%5.1f\n", Ki);
  //    M5.Lcd.printf("Kd:%6.3f\n", Kd);
  M5.Lcd.printf("Bat:%5.3fV\n", M5.Axp.GetBatVoltage());
  M5.Lcd.printf("Cur:%5.1f\n", M5.Axp.GetBatCurrent());
  M5.Lcd.printf("freq:%5.2f        ",loopfreq);
  
}

void setup() {
  Serial.begin(151200);

  M5.begin();
  M5.Lcd.setRotation(2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Axp.ScreenBreath(8);
  M5.Axp.SetChargeCurrent(0x01);
  M5.MPU6886.Init();
  BalaC_Init();

  pinMode(GPIO_NUM_10, OUTPUT);
  digitalWrite(GPIO_NUM_10, HIGH);
  drawScreen();

  Kp = 900.0;
  Ki = 90.0;
  Kd = 6.0;
}

static  int Duty;
bool led = false;

void loop() {
  M5.update();
  if ( M5.BtnB.wasReleased() ) {
    // ボタンBを押すと再起動
    esp_restart();
  }

  if ( M5.BtnA.wasReleased() ) {
    // ボタンAを押して3秒後にモーター駆動開始
    Serial.println("BtnA.wasReleased() == TRUE");

    if (!first)
    {
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("3");
      delay(1000);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("2");
      delay(1000);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("1");
      // ボタンを押したら直立させて待ち残り1秒になったら現在の角度を基準に設定する
      float temp = 0.0;
      for (int i = 0; i < 10; i++) {
        M5.MPU6886.getAhrsData(&pitch, &roll, &yaw, 100.0f);
        temp += roll;
        delay(10);
      }
      target = temp / 10.0;
    }
    first = false;

    delay(800);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("0");
    preTime = micros();
    started = true;

  }

  dt = (micros() - preTime) / 1000000; // 処理時間を求める
  preTime = micros(); // 処理時間を記録

  loopfreq = 1/dt;
  
  M5.MPU6886.getAhrsData(&pitch, &roll, &yaw, loopfreq);

  //  Serial.printf("%8.1f,%8.1f,%8.1f\n",roll,pitch,yaw);
  drawScreen();
  //delay(2);
  if (!started) {
    return;
  }

  now = target - roll;
  

  // 目標角度から現在の角度を引いて偏差を求める
  // -90~90を取るので90で割って-1.0~1.0にする
  P  = (target - roll) / 90;
  // 偏差を積分する
  I += P * dt;
  // 偏差を微分する
  D  = (P - preP) / dt;
  // 偏差を記録する
  preP = P;

  // 積分部分が大きくなりすぎると出力が飽和するので大きくなり過ぎたら0に戻す(アンチワインドアップ)
  if (100 < abs(I * Ki)) {
    I = 0;
    led = !led;
    digitalWrite(GPIO_NUM_10, led ? LOW : HIGH);

    // anti windup
  }

  // 出力を計算する
  power = Kp * P + Ki * I + Kd * D;

  power += 9 * (power / abs(power));

  power = constrain(power, -100, 100);


  // モーターの出力を決定する(90で静止+-それぞれが90より離れると速度アップ)

  //Duty = (int)(80 * (power / 100));
  Duty = (int)power;

  // +-40度を越えたら倒れたとみなす
  if (abs(now) <= 50 )
  {
    BalaC_SetPowerA(-Duty);
    BalaC_SetPowerB(Duty);
  }
  else {
    // 静止
    BalaC_SetPowerA(0);
    BalaC_SetPowerB(0);

    power = 0;
    I = 0;
    led = false;
  }
}
