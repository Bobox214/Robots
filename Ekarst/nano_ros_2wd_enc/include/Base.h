#include <PID.h>

const int PWML  = 5;
const int DIR1L = 6;
const int DIR2L = 4;
const int PWMR  = 8;
const int DIR1R = 7;
const int DIR2R = 9;

const int ENCODER1L = 3;
const int ENCODER2L = 17;
const int ENCODER1R = 2;
const int ENCODER2R = 18;

volatile long countL = 0;
volatile long countR = 0;

void encoderCountL() {
    if (digitalRead(ENCODER2L))
        countL++;
    else
        countL--;
}
void encoderCountR() {
    if (digitalRead(ENCODER2R))
        countR++;
    else
        countR--;
}

class Base {
	public:
		Base()
		{	
            pinMode(PWML , OUTPUT);
            pinMode(DIR1L, OUTPUT);
            pinMode(DIR2L, OUTPUT);
            pinMode(PWML , OUTPUT);
            pinMode(DIR1L, OUTPUT);
            pinMode(DIR2L, OUTPUT);
            attachInterrupt(digitalPinToInterrupt(ENCODER1L),encoderCountL,RISING);
            attachInterrupt(digitalPinToInterrupt(ENCODER1R),encoderCountR,RISING);

            setMotorPwm(0,0);
            setPosition(0,0,0);
		}
		void setParameters(double baseWidth, double wheelRadius, int ticksPerRotation) {
			this->baseWidth        = baseWidth;
			this->wheelRadius      = wheelRadius;
			this->ticksPerRotation = ticksPerRotation;
		}
		void setPIDs(double Kp,double Ki, double Kd) {
			this->baseWidth        = baseWidth;
			this->wheelRadius      = wheelRadius;
			this->ticksPerRotation = ticksPerRotation;
			pidL.setCoefficients(kP,kI,kD);
			pidL.setOutputRange(-255,255,20);
			pidR.setCoefficients(kP,kI,kD);
			pidR.setOutputRange(-255,255,20);
		}
		void setPosition(double x, double y, double yaw) {
			this->_x   = x;
			this->_y   = y;
			this->_yaw = yaw;
		}
		void setDebug(bool debug) {
			this->debug = debug;
		}
		void loop() {
			double dt = (millis()-loopTime)/1000.0;
			long newTicksL = countL;
			long newTicksR = countR;
			double dl = (2*PI*wheelRadius*(newTicksL-ticksL))/ticksPerRotation;
			double dr = (2*PI*wheelRadius*(newTicksR-ticksR))/ticksPerRotation;
			double df = (dr+dl)/2;
			double dx = df*cos(_yaw);
			double dy = df*sin(_yaw);
			double dyaw = atan2(dr-dl,baseWidth);
			curVR = dr/dt;
			curVL = dl/dt;
			curW  = dyaw/dt;
			_x   += dx;
			_y   += dy;
			_yaw += dyaw;
			if (speedMode) {
				pwmL = pidL.update(curVL-goalVL,dt);
				pwmR = pidR.update(curVR-goalVR,dt);
				setMotorPwm(pwmL,pwmR);
			}
			ticksL    = newTicksL;
			ticksR    = newTicksR;
			loopTime  = millis();
		}
		void setMotorPwm(int pwmL,int pwmR) {
			pwmL = constrain(pwmL,-255,255);
			pwmR = constrain(pwmR,-255,255);
            if (pwmL==0) {
                digitalWrite(DIR1L,0);
                digitalWrite(DIR2L,0);
                digitalWrite(PWML,0);
            } else if (pwmL>0) {
                digitalWrite(DIR1L,1);
                digitalWrite(DIR2L,0);
                digitalWrite(PWML,pwmL);
            } else {
                digitalWrite(DIR1L,0);
                digitalWrite(DIR2L,1);
                digitalWrite(PWML,-pwmL);
            }
            if (pwmR==0) {
                digitalWrite(DIR1R,0);
                digitalWrite(DIR2R,0);
                digitalWrite(PWMR,0);
            } else if (pwmR>0) {
                digitalWrite(DIR1R,1);
                digitalWrite(DIR2R,0);
                digitalWrite(PWMR,pwmR);
            } else {
                digitalWrite(DIR1R,0);
                digitalWrite(DIR2R,1);
                digitalWrite(PWMR,-pwmR);
            }
		}
		void setSpeed(double v, double w) {
			speedMode = true;
			goalVR = v-w*baseWidth/2;
			goalVL = v+w*baseWidth/2;
		}
		void stop() {
            setMotorPwm(0,0);
			pidL.reset();
			pidR.reset();
			speedMode = false;
		}
			
		double x()   { return this->_x;   }
		double y()   { return this->_y;   }
		double yaw() { return this->_yaw; }
	private:
		double baseWidth,wheelRadius;
		int    ticksPerRotation;
		bool   debug;
		double _x,_y,_yaw;
		PID    pidL,pidR;
		bool   speedMode;
		double kP,kI,kD;
		double goalVR,goalVL;
		double curVR,curVL,curW;
		unsigned long loopTime;
		long    pwmL,pwmR;
		long    ticksL,ticksR;

};