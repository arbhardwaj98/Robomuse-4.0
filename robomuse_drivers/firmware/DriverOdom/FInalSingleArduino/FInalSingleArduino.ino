#include <USBSabertooth.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>
#include <TimerOne.h>
#include <robomuse/chapter2_msg1.h>

USBSabertoothSerial C(Serial2);             // Use SWSerial as the serial port.
USBSabertooth ST(C, 128);              // Use address 128.

Encoder knobRight(18, 19);
Encoder knobLeft(2, 3);
#define scale 700 // 70

int flag = 0;

long positionLeft  = -999, positionRight = -999;
long currentposLeft, currentposRight, newLeft, newRight;
double SpeedRight, SpeedLeft, SumErrorLeft, SumErrorRight;
double SetSpeedRight, SetSpeedLeft, ErrorLeft, ErrorRight;
double Kpl;
double Ki;
double Kd;

int distanceleft, distanceright, lastDl, lastDr;
double SetSpeedx = 0, SetSpeedy = 0;

ros::NodeHandle nh;
double x,y;
robomuse::chapter2_msg1 a;
ros::Publisher chatter("message", &a);
int flag1 = 0;

void messageCb(const geometry_msgs::Twist& msg) {
	if(msg.linear.x == 0 && msg.angular.z == 0)	{
		if ( flag1 == 2) {
		  SetSpeedx=msg.linear.x;
		  SetSpeedy=msg.angular.z;
		}
		else flag1++;
	}
	else {
		flag1 = 0;
		SetSpeedx=msg.linear.x;
		SetSpeedy=msg.angular.z;
	} 
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );
double Kpr;
void setup() {
	SabertoothTXPinSerial.begin(19200);
	nh.initNode();
	nh.subscribe(sub);  
	nh.initNode();
	nh.advertise(chatter);
	Serial3.begin(115200);
	Serial.begin(115200);
	Serial2.begin(19200);
	Serial.println("Code is started");
	pinMode(13, OUTPUT);
	Timer1.initialize(500000/2);         // initialize timer1, and set a 1/2 second period
	Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
	ST.setRamping(-5000);
	SetSpeedRight = 2;//dont cross 5
	SetSpeedLeft = 2;
	Kpl = 0.1;
	Kpr = 0.15;
	Ki = 0.003; //0.007
	Kd = 0.0;  
}

void cap(double &input,double cap_val) {
	if(input > cap_val)
		input = cap_val;
	if(input < -cap_val)
		input = -cap_val;  
}

void callback() {
	digitalWrite(13, !digitalRead(13));

	SpeedLeft = double(newLeft - currentposLeft) * 0.11 * 4 * 2 * 3.14/ (8192 * 2);
	SpeedRight = double(newRight - currentposRight) * 0.11 * 4 * 2 * 3.14 / (8192 * 2);
	currentposLeft = newLeft;
	currentposRight = newRight;

	flag++;
}

double OutputRight, OutputLeft;

void loop() {
	SetSpeedLeft = SetSpeedx - (0.57 * SetSpeedy * 2 / 3);
	SetSpeedRight = SetSpeedx + (0.57 * SetSpeedy * 2 / 3);

	long OldErrorRight, OldErrorLeft;
	newLeft = knobLeft.read();
	newRight = knobRight.read();
	a.A = newLeft;
	a.B = newRight;
	chatter.publish( &a );

	OldErrorLeft = ErrorLeft;
	OldErrorRight = ErrorRight;
	ErrorLeft = SetSpeedLeft - SpeedLeft;
	ErrorRight =  SetSpeedRight - SpeedRight;
	SumErrorLeft = SumErrorLeft + ErrorLeft;
	SumErrorRight = SumErrorRight + ErrorRight;
	if(ErrorLeft > 0.8 * SetSpeedLeft)
		ErrorLeft = 0;
	if(ErrorRight > 0.8 * SetSpeedRight)
		ErrorRight = 0;
	OutputLeft = scale * (SetSpeedLeft + Kpl * ErrorLeft + Kd * (ErrorLeft - OldErrorLeft) + Ki * SumErrorLeft);
	OutputRight = scale * (SetSpeedRight + Kpr * ErrorRight + Kd * (ErrorRight - OldErrorRight) + Ki * SumErrorRight);
	cap(OutputLeft,300);
	cap(OutputRight,300);

	ST.motor(2,-OutputLeft);
	ST.motor(1,OutputRight);

	nh.spinOnce();
}
