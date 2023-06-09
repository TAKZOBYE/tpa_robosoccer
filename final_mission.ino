#include <POP32.h>
#include <POP32_Pixy2.h>
POP32_Pixy2 pixy;

#define degToRad 0.0174f
#define sin30 sin(30.f * degToRad)
#define cos30 cos(30.f * degToRad)

// Rotate Variable
#define rotationKp 0.35
#define rotationKi 0.0
#define rotationKd 0.0
#define rotationErrorGap 15
float rotationError, rotationPvError, rotationI, rotationD, rotationW;

// fling Variable
#define flingKp 1.5
#define flingKi 0.0
#define flingKd 0.0
#define flingErrorGap 20
float setpointFling = 180;
float flingError, flingPvError, flingI, flingD, flingSpeed;

// align Variable
#define alignKp 2.75
#define alignKd 0.0
#define alignErrorGap 3
float alignError, alignPvError, alignD, alignVec;

bool isDribble = false;

// Ball Shoot Variable
#define limPin PA0
#define reloadSpd 60

int timer = 0;

void zeroYaw() {
  Serial1.begin(115200);
  delay(100);
  // Sets data rate to 115200 bps
  Serial1.write(0XA5);
  Serial1.write(0X54);
  delay(100);
  // pitch correction roll angle
  Serial1.write(0XA5);
  Serial1.write(0X55);
  delay(100);
  // zero degree heading
  Serial1.write(0XA5);
  Serial1.write(0X52);
  delay(100);
  // automatic mode
}

volatile float currentYaw, lastYaw;
volatile uint8_t rxCnt = 0, rxBuf[8];
bool getIMU() {
  while (Serial1.available()) {
    rxBuf[rxCnt] = Serial1.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA) return;
    rxCnt++;
    if (rxCnt == 8) {  // package is complete
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) {  // data package is correct
        currentYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        return true;
      }
    }
  }
  return false;
}

float getMostNumber(float num1, float num2, float num3) {
  float largest;

  if (num1 > num2 && num1 > num3) {
    largest = num1;
  } else if (num2 > num1 && num2 > num3) {
    largest = num2;
  } else if (num3 > num1 && num3 > num2) {
    largest = num3;
  } else {
    largest = num1;
  }

  return largest;
}

void shoot() {
  motor(4, reloadSpd);
  delay(150);
  motor(4, 0);
  delay(50);
}

void reload() {
  motor(4, reloadSpd);

  int timer = 0;

  for (int i = 0; i < 2000; i++) {
    timer++;
    if (in(limPin)) break;
    delay(1);
  }

  if (timer >= 2000) {
    motor(4, -reloadSpd);  // เลื่อนก้านยิ่งไปข้างหน้า
    delay(500);            //ก่อน 0.5 วินาที
    motor(4, reloadSpd);
    timer = 0;
    for (int i = 0; i < 2000; i++) {
      timer++;
      if (in(limPin)) break;
      delay(1);
    }
  }

  motor(4, 0);
  delay(50);
}

void wheel(int s1, int s2, int s3) {
  // Must to check if motor not equal
  motor(1, s1);
  motor(2, s2);
  motor(3, s3);
}

#define headKp 2.3f
#define headKi 0.001f
#define headKd 0.5f
float headError, headPvError, headW, headD, headI;
void heading(float speed, float theta, float spYaw) {
  headError = spYaw - currentYaw;
  headI = headI + headError;
  headI = constrain(headI, -180, 180);
  headD = headError - headPvError;
  headW = (headError * headKp) + (headI * headKi) + (headD * headKd);
  headW = constrain(headW, -100, 100);
  headPvError = headError;
  holonomic(speed, theta, headW);
}

void holonomic(float spd, float theta, float omega) {
  float thetaRad = theta * degToRad;
  float vx = spd * cos(thetaRad);
  float vy = spd * sin(thetaRad);
  float speed1 = vy * cos30 - vx * sin30 + omega;
  float speed2 = -vy * cos30 - vx * sin30 + omega;
  float speed3 = vx + omega;

  float mostSpeed = getMostNumber(speed1, speed2, speed3);
  if (mostSpeed > 100) {
    float ratio = 100 / mostSpeed;

    speed1 *= ratio;
    speed2 *= ratio;
    speed3 *= ratio;
  }

  wheel(speed1, speed2, speed3);
}

void checkWhiteLine() {
  int backSpeed = 50;

  int middle1 = 1820;
  int middle2 = 1555;
  int middle3 = 577;

  float sensor1 = analog(1);
  float sensor2 = analog(2);
  float sensor3 = analog(3);

  if (sensor1 > middle1) {
    holonomic(backSpeed, 270, 0);
  } else if (sensor2 > middle2) {
    holonomic(backSpeed, 45, 0);
  } else if (sensor3 > middle3) {
    holonomic(backSpeed, 135, 0);
  }

  // oled.clear();
  // oled.text(1, 0, "%f          ", sensor1);
  // oled.text(2, 0, "%f          ", sensor2);
  // oled.text(3, 0, "%f          ", sensor3);
  // oled.show();
}

/*
void penalty() {
	// Yellow = 2
	// Blue 3

	int penaltyColor = 3;  // Make Ternary Condition More

	while (true) {
		if (pixy.updateBlocks() && pixy.sigSize[penaltyColor]) {
			float screenWidth = 315.0f;

			signature firstSig = pixy.sigInfo[penaltyColor][0];
			signature secondSig = pixy.sigInfo[penaltyColor][1];

			signature mostWidthObject = firstSig.width > secondSig.width ? firstSig : secondSig;

			bool isLeftSide = (screenWidth / 2) > mostWidthObject.x;

			int diff = 0;
			float targetX = isLeftSide ? mostWidthObject.x - (mostWidthObject.width / 2) + diff : mostWidthObject.x + (mostWidthObject.width / 2) - diff;

			float angle = (targetX / screenWidth * 60.0f) - 30.0f;
			int xDiff = abs(targetX - (screenWidth / 2));

			if (xDiff < 20) {
				zeroYaw();

				while (true) {
					for (int i = 0; i < 8; i++) { getIMU(); }

					int ballPosX = pixy.sigInfo[1][0].x;
					int ballPosY = pixy.sigInfo[1][0].y;

					float alignError = ballPosY - setpointFling;
					float alignD = alignError - alignPvError;
					float alignVec = alignError * alignKp + alignD * alignKd;
					alignPvError = alignError;

					float theta = lastYaw < 0 ? -alignVec : 270 + alignVec;
					float omega = lastYaw < 0 ? 15 : -15;

					holonomic(40, theta, omega);

					if (abs(currentYaw) < alignErrorGap) return;
				}
			}

			holonomic(xDiff, isLeftSide ? 0 : 180, -angle);
			delay(350);
			ao();
		}
	}
}
*/

void normalPlay() {
  oled.clear();
  oled.text(1, 0, "%d     ", isDribble ? 1 : 0);
  oled.show();

  // ไม่เจอบอล
  if (!pixy.updateBlocks() || !pixy.sigSize[1]) {
    int idleSpeed = 40;

    holonomic(0, 0, ((rotationError / abs(rotationError)) || 1) * idleSpeed);
  } else {
    // เจอ Ball
    int ballPosX = pixy.sigInfo[1][0].x;
    int ballPosY = pixy.sigInfo[1][0].y;

    // หาค่า Yaw ปัจจุบัน
    for (int i = 0; i < 8; i++) { getIMU(); }

    // เลี้ยงบอล
    if (isDribble) {
      // ปรับทิศหุ่นให้ลูกบอลอยู่ตรงกลาง
      float alignError = ballPosY - setpointFling;
      float alignD = alignError - alignPvError;
      float alignVec = alignError * alignKp + alignD * alignKd;
      alignPvError = alignError;

      float theta = lastYaw < 0 ? -alignVec : 270 + alignVec;
      float omega = lastYaw < 0 ? 15 : -15;

      holonomic(40, theta, omega);

      if ((abs(157 - ballPosX) < rotationErrorGap) && (abs(setpointFling - ballPosY) < flingErrorGap)) {
        beep();

        holonomic(0, 0, 0);
        isDribble = false;

          
        unsigned long loopTimer = millis();
        while (1) {
          getIMU();
          heading(100, 90, 0);
          if (millis() - loopTimer >= 250) break;
        }

        shoot();
        reload();
      }
    } else {
      // หมุนหาบอล
      rotationError = 157 - ballPosX;
      rotationD = rotationD + rotationError;
      rotationD = constrain(rotationD, -100, 100);
      rotationD = rotationError - rotationPvError;
      rotationPvError = rotationError;
      rotationW = (rotationError * rotationKp) + (rotationI * rotationKi) + (rotationD * rotationKd);
      rotationW = constrain(rotationW, -100, 100);

      flingError = setpointFling - ballPosY;
      flingI = flingI + flingError;
      flingI = constrain(flingI, -100, 100);
      flingD = flingError - flingPvError;
      flingPvError = flingError;
      flingSpeed = flingError * flingKp + flingI * flingKi + flingD * flingKd;
      flingSpeed = constrain(flingSpeed, -100, 100);

      holonomic(flingSpeed, 90, rotationW);

      // เงื่อนไขเข้าสู่การเลี้ยงลูกบอล
      // แกน X ใกล้ลูกบอล กับ แกน Y ใก้ลูกบอล
      if ((abs(rotationError) < rotationErrorGap) && (abs(flingError) < flingErrorGap)) {
        wheel(0, 0, 0);
        lastYaw = currentYaw;
        isDribble = true;
      }
    }

    checkWhiteLine();
  }
}

void setup() {
  // Serial.begin(9600);
  oled.mode(2);
  pixy.init();
  zeroYaw();
  reload();

  while (!SW_A()) {
    if (SW_B()) {
      zeroYaw();
    }

    getIMU();
    oled.text(0, 0, "Yaw=%f     ", currentYaw);
    oled.show();
  }

  // penalty();

  // while (1) {
  //   if (SW_A()) {
  //     reload();
  //   }
  //   if (SW_B()) {
  //     shoot();
  //   }
  //   if (SW_OK()) {
  //     reload();
  //     shoot();
  //   }
  // }
}

void loop() {
  normalPlay();

  // oled.clear();
  // if (pixy.updateBlocks()) {
  //   oled.text(1, 0, "%d     ", pixy.sigInfo[5][0].width);
  // } else {
  //   oled.text(1, 0, "Not Found");
  // }
  // oled.show();
}