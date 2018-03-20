/*

180205 : 로봇의 방향과 이동 정보를 추가해서 로봇의 위치를 확인할수 있게 하자
180217 : 센서의 필터 추가, OLED 정보 표시 ( 배터리 전압, 센서값, 모터 펄스값
180219 ; 벽 보정 방식을 변경 : 속도를 변경하는 방식에 문제가 있었음. 스텝수를 추가하여 회전하도록 함
180228 : 보정값을 디파인으로 변경함
180301 : 벽 보정값을 조정해서 직진 동작이 좀 나아짐 벽에 너무 가까이 붙었을때는 제대로 회전하지 못하는듯.
180305 : 현재까지는 직진할때도 몇개의 스텝(엔코더 수)마다 방향을 판단하여 양쪽 모터의 스텝수를 변경하면서 직진 보정을 하도록 했다.
스텝수로 보정하지 않고, 직진할때는 계속 센서값을 보면서 모터 속도만 조정하도록 했다.
별개의 프로그램으로 모터 회전 속도를 제어해봤을때 6v 정도를 입력하면 70~250의 PWM 값에서 모터가 정상적으로 회전했다. (60에서는 한쪽만 회전함. 오른쪽 모터가 좀더 빠르게 돈다)
*/
#include "U8glib.h"

// 상수 define //////////////////////////////////////////////

#define FRONT 0
#define LEFT 1
#define RIGHT 2

//지도에서 보는 로봇의 방향 
#define NORTH 1
#define SOUTH 2
#define WEST 3
#define EAST 4

#define TRUE 1
#define FALSE 0

// PORT 설정 //////////////////////////////////////////////
#define FRONT_SENSOR A1
#define LEFT_SENSOR A2
#define RIGHT_SENSOR A0

#define LEFT_MOTOR_PWM 5
#define LEFT_MOTOR_DIR 3

#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIR 4

#define VOLTAGE_MONITOR A3

// 기본 이동용 파라미터 //////////////////////////////////////////////

#define TURN_90_STEP 12   //90도 회전 스텝
#define ONECELL_STEP 3  //기본 이동 거리

#define DEFAULT_SPEED 210
#define SPEED_OFFSET ((255-DEFAULT_SPEED)/4)
#define STEP_OFFSET 2

//기본 동작 변수////////////////////////////////////////////////////////////////////
int LeftSpeed = DEFAULT_SPEED;
int RightSpeed = DEFAULT_SPEED;

unsigned long LeftMotorTimer, RightMotorTimer, systemTimer;

//int a, b, c, d;
//엔코더 값 변경 저장
int LeftMotorEncoder;
int RightMotorEncoder;

//이동할 모터의 목표 카운터
int LeftMotorStepTarget;
int RightMotorStepTarget;

//실제 회전수 카운터
int LeftMotorStepCounter;
int RightMotorStepCounter;

//거리 계산을 위한 모터 회전 카운터
int LeftMotorDistanceStepCounter =0;
int RightMotorDistanceStepCounter=0;


//모터 On Off 설정
int IsMotor90Turn;			//90도 턴을 할때는 센서값을 판단하지 않도록 하는 변수 -> 90도 턴을 하는 동안 양쪽 모터의 회전이 끝났는지를 저장
int MotorAllOn;			//모터를 동작하게 해주는 변수

#define SIDE_DISTANCE_4CM (SIDE_DISTANCE_3CM - 50) 
#define SIDE_DISTANCE_3CM 460
#define SIDE_DISTANCE_2CM (SIDE_DISTANCE_3CM + 50)
#define SIDE_DISTANCE_1CM (SIDE_DISTANCE_2CM + 70)

int CalibrationDirection;
int CalibrationValue;

int Distance[3];    //front left fight 순서로 센서값 저장

#define FILTER_SIZE 5
int DistanceAverage[3][FILTER_SIZE];
int DistAvgIndex = 0;

//로봇 위치, 이동 방향 ///////////////////////////////////////////////////////
int RobotDir;

struct POS {
	int x;
	int y;
};

struct POS RobotPos = { 0,0 };

int IsTurning;
//OLED 설정 /////////////////////////////
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send AC

#define SMALL_LINE 8
#define LARGE_LINE 15

int voltage;
int VolategeAverage[FILTER_SIZE];

//벽 판단
#define SIDE_WALL_OFFSET 370
#define FRONT_WALL_DETECTION 390

//32cm 까지 센서값  : 3cm -> 640
int SensorToDistTable[] = { 9999,9999,9999,640,540,460,390,337,295,260,230,209,188,174,160,152,138,129,120,111,107,103,94,89,85,80,76,71,66,62,57,55 };

int CheckWall(int sensor, int LowLimit, int HighLimit) {    //특정 센서가 cm 범위에 있는지... ex) 3cm 이내인지 알고 싶다면, (FRONT, 0, 3)

	if (Distance[sensor] < SensorToDistTable[LowLimit] && Distance[sensor] > SensorToDistTable[HighLimit]) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}


void LeftMotor(bool dir, int vel) {
	if (MotorAllOn == 0) return;
	digitalWrite(LEFT_MOTOR_DIR, dir);
	//digitalWrite(LEFT_MOTOR_PWM, !dir & 0x01);
	analogWrite(LEFT_MOTOR_PWM, (dir) ? 255 - vel : vel);
}

void RightMotor(bool dir, int vel) {
	if (MotorAllOn == 0) return;
	digitalWrite(RIGHT_MOTOR_DIR, dir);
	//digitalWrite(RIGHT_MOTOR_PWM, !dir & 0x01);
	analogWrite(RIGHT_MOTOR_PWM, (dir) ? 255 - vel : vel);
}

void GoForward() {
	//LeftMotorStepTarget = RightMotorStepTarget = ONECELL_STEP;
	LeftMotor(1, LeftSpeed);
	RightMotor(1, RightSpeed);
}

void LeftTurn() {
	LeftMotorStepTarget = TURN_90_STEP;
	RightMotorStepTarget = TURN_90_STEP;
	LeftMotorStepCounter = 0;
	RightMotorStepCounter = 0;
	LeftMotor(0, LeftSpeed);
	RightMotor(1, RightSpeed);
	IsMotor90Turn = LEFT | RIGHT;

	LeftMotorDistanceStepCounter = 0;
	RightMotorDistanceStepCounter = 0;
}


void RightTurn() {
	LeftMotorStepTarget = TURN_90_STEP;
	RightMotorStepTarget = TURN_90_STEP;
	LeftMotorStepCounter = 0;
	RightMotorStepCounter = 0;
	LeftMotor(1, LeftSpeed);
	RightMotor(0, RightSpeed);
	IsMotor90Turn = LEFT | RIGHT;

	LeftMotorDistanceStepCounter = 0;
	RightMotorDistanceStepCounter = 0;
}

void StopMotor() {
	RightMotor(0, 0);
	LeftMotor(0, 0);
}


void PrintDataToOLED() {
	u8g.firstPage();
	do {

		u8g.setFont(u8g_font_6x10);
		u8g.setPrintPos(0, SMALL_LINE);
		u8g.print("Mouse ");
		//u8g.print(voltage,DEC);
		voltage = voltage / 2;
		u8g.print(voltage / 100, DEC); u8g.print(".");  u8g.print(voltage - ((voltage / 100) * 100), DEC); u8g.print("v"); //2로 나누면 전압 비슷하게 표시됨

		u8g.setPrintPos(0, SMALL_LINE + LARGE_LINE); u8g.print("Sn F "); u8g.print(Distance[FRONT], DEC);
		u8g.print(" L "); u8g.print(Distance[LEFT], DEC);
		u8g.print(" R "); u8g.print(Distance[RIGHT], DEC);

		u8g.setPrintPos(0, SMALL_LINE + LARGE_LINE * 2); u8g.print("Sp L "); u8g.print(LeftSpeed, DEC);
		u8g.print(" R "); u8g.println(RightSpeed, DEC);

		u8g.setPrintPos(0, SMALL_LINE + LARGE_LINE * 3); u8g.print("St L "); u8g.print(LeftMotorDistanceStepCounter, DEC);
		u8g.print(" R "); u8g.println(RightMotorDistanceStepCounter, DEC);

		u8g.setPrintPos(80, SMALL_LINE + LARGE_LINE * 3 + 3); u8g.println(IsMotor90Turn, DEC);
	} while (u8g.nextPage());
}

void SetupOLED() {
	//OLED u8glib 설정///////////////////////////////////
	if (u8g.getMode() == U8G_MODE_R3G3B2) {
		u8g.setColorIndex(255);     // white
	}
	else if (u8g.getMode() == U8G_MODE_GRAY2BIT) {
		u8g.setColorIndex(3);         // max intensity
	}
	else if (u8g.getMode() == U8G_MODE_BW) {
		u8g.setColorIndex(1);         // pixel on
	}
	else if (u8g.getMode() == U8G_MODE_HICOLOR) {
		u8g.setHiColorByRGB(255, 255, 255);
	}

}

void Sensor(void) {
	DistanceAverage[FRONT][DistAvgIndex] = analogRead(FRONT_SENSOR);
	DistanceAverage[LEFT][DistAvgIndex] = analogRead(LEFT_SENSOR);
	DistanceAverage[RIGHT][DistAvgIndex] = analogRead(RIGHT_SENSOR);
	VolategeAverage[DistAvgIndex] = analogRead(VOLTAGE_MONITOR);
	if (DistAvgIndex >= FILTER_SIZE - 1) DistAvgIndex = 0;
	else DistAvgIndex++;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < FILTER_SIZE; j++) {
			Distance[i] += DistanceAverage[i][j];

		}
		Distance[i] /= FILTER_SIZE;
	}
	for (int j = 0; j < FILTER_SIZE; j++) {
		voltage += VolategeAverage[j];
	}
	voltage /= FILTER_SIZE;
	//	Distance[FRONT] = analogRead(FRONT_SENSOR);
	//	Distance[LEFT] = analogRead(LEFT_SENSOR);
	//	Distance[RIGHT] = analogRead(RIGHT_SENSOR);

	if (millis() >= systemTimer + 500) { //500ms마다
		systemTimer = millis();
		PrintDataToOLED();
	}

}


void setup() {
	// initialize serial:
	Serial.begin(9600);
	// make the pins outputs:
	pinMode(LEFT_MOTOR_DIR, OUTPUT);
	pinMode(RIGHT_MOTOR_DIR, OUTPUT);

	pinMode(LEFT_MOTOR_PWM, OUTPUT);
	pinMode(RIGHT_MOTOR_PWM, OUTPUT);


	pinMode(9, INPUT);   //encoder                              //Initialize the module
	pinMode(10, INPUT);  //encoder
	pinMode(11, INPUT);  //encoder
	pinMode(12, INPUT);  //encoder

	pinMode(13, INPUT);  //switch  

	SetupOLED();

	noInterrupts();
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT1 | 1 << PCINT2 | 1 << PCINT3 | 1 << PCINT4);  //모터 엔코더 
	interrupts();

	while (digitalRead(13) == 0) {
		Sensor();
	}   //스위치가 눌릴때까지 대기 

	MotorAllOn = 1;
	systemTimer = millis();

	delay(500);
	LeftTurn();
	delay(2000);
	RightTurn();
	delay(2000);

	while (digitalRead(13) == 0) {
		Sensor();
	}   //스위치가 눌릴때까지 대기 


	RobotPos.x = 0; RobotPos.y = 0;


}


void loop() {
	Sensor();


	//if (digitalRead(13) == 1) while (1);   //스위치가 눌리면 정지

	if (IsMotor90Turn == 0) {
		//if( FALSE == CheckWall(FRONT,0,5)) {  //3cm 이내에 벽이 없으면
		if (Distance[FRONT] < FRONT_WALL_DETECTION) {
			CalibrationDirection = 0;
			CalibrationValue = 0;
			//if(TRUE == CheckWall(LEFT,4,5) ){
			if (Distance[LEFT] > Distance[RIGHT]) {	//가까운 벽 기준으로 보정
				if (Distance[LEFT] > SIDE_DISTANCE_1CM) {//왼쪽에서 1CM 이하로 떨어져 있음... 거의 벽쪽
					CalibrationDirection = RIGHT;			//왼쪽벽에 붙어있으므로 오른쪽 방향으로 이동
					CalibrationValue = 3;	//보정량
				}
				else if (Distance[LEFT] > SIDE_DISTANCE_2CM) {
					CalibrationDirection = RIGHT;
					CalibrationValue = 2;
				}
				else if (Distance[LEFT] > SIDE_DISTANCE_3CM) {		//벽에서 3cm 이하로 떨어져있음.. 거의 중간
					CalibrationDirection = 0;
					CalibrationValue = 0;	//보정량 1
				}
				else if (Distance[LEFT] > SIDE_DISTANCE_4CM) {
					CalibrationDirection = LEFT;
					CalibrationValue = 2;
				}
				else {
					CalibrationDirection = LEFT;
					CalibrationValue = 3;
				}
			}
			else {
				if (Distance[RIGHT] > SIDE_DISTANCE_1CM) {//왼쪽에서 1CM 이하로 떨어져 있음... 거의 벽쪽
					CalibrationDirection = LEFT;			//왼쪽벽에 붙어있으므로 오른쪽 방향으로 이동
					CalibrationValue = 3;	//보정량 3
				}
				else if (Distance[RIGHT] > SIDE_DISTANCE_2CM) {
					CalibrationDirection = LEFT;
					CalibrationValue = 2;
				}
				else if (Distance[RIGHT] > SIDE_DISTANCE_3CM) {		//벽에서 3cm 이하로 떨어져있음.. 거의 중간
					CalibrationDirection = 0;
					CalibrationValue = 0;	//보정량 1
				}
				else if (Distance[RIGHT] > SIDE_DISTANCE_4CM) {		//실제로 4cm가 아님. 로봇이 회전한 상태에서 사용
					CalibrationDirection = RIGHT;
					CalibrationValue = 2;	//보정량 1
				}
				else {
					CalibrationDirection = RIGHT;
					CalibrationValue = 3;	//보정량 0
				}
			}

			//lcd.setCursor(15, 1);
			RightSpeed = LeftSpeed = DEFAULT_SPEED;
			LeftMotorStepTarget = RightMotorStepTarget = ONECELL_STEP;
			if (CalibrationDirection == LEFT) {
				LeftSpeed = DEFAULT_SPEED - (SPEED_OFFSET * CalibrationValue);		//왼쪽으로 가려면 왼쪽 속도를 줄여야함.
				RightSpeed = DEFAULT_SPEED + (SPEED_OFFSET * CalibrationValue);
			}
			else if (CalibrationDirection == RIGHT) {
				RightSpeed = DEFAULT_SPEED - (SPEED_OFFSET * CalibrationValue);
				LeftSpeed = DEFAULT_SPEED + (SPEED_OFFSET * CalibrationValue);
			}
			else {
				//lcd.print("N");
			}
			GoForward();
		}
		else {
			LeftSpeed = DEFAULT_SPEED;
			RightSpeed = DEFAULT_SPEED;
			Serial.println("FrontWall detected");

			if (Distance[LEFT] < Distance[RIGHT]) {   //왼쪽에 벽이 없으면
				delay(500);
				LeftTurn();
			}
			else {
				//Serial.println("right");
				delay(500);
				RightTurn();
			}
		}
	}
}

ISR(PCINT0_vect)
{
	if (millis() >= systemTimer + 500) { //500ms마다
		systemTimer = millis();
		PrintDataToOLED();
	}
	int LeftTemp = digitalRead(9) << 1 | digitalRead(10);
	int RightTemp = digitalRead(11) << 1 | digitalRead(12);

	if (LeftTemp != LeftMotorEncoder) {
		LeftMotorStepCounter++;
		if (IsMotor90Turn == 0) LeftMotorDistanceStepCounter++;
		LeftMotorEncoder = LeftTemp;
		if (IsMotor90Turn & LEFT) {
			if (LeftMotorStepTarget < LeftMotorStepCounter) {
				LeftMotor(0, 0);
				IsMotor90Turn &= ~LEFT;
			}
		}
	}

	if (RightTemp != RightMotorEncoder) {
		RightMotorStepCounter++;
		if (IsMotor90Turn == 0) RightMotorDistanceStepCounter++;
		RightMotorEncoder = RightTemp;
		if (IsMotor90Turn & RIGHT) {
			if (RightMotorStepTarget < RightMotorStepCounter) {
				RightMotor(0, 0);
				IsMotor90Turn &= ~RIGHT;
			}
		}
	}

	if (LeftMotorDistanceStepCounter + RightMotorDistanceStepCounter > 30) {
		delay(1000);
	}
}

