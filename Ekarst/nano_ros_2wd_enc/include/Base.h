#include <ros.h>
#include <PID.h>

const int PWMR  = 5;
const int DIR1R = 6;
const int DIR2R = 4;
const int PWML  = 9;
const int DIR1L = 7;
const int DIR2L = 8;

const int ENCODER1R = 3;
const int ENCODER2R = 17;
const int ENCODER1L = 2;
const int ENCODER2L = 18;

volatile int32_t countL = 0;
volatile int32_t countR = 0;

char msg[128];

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
            pinMode(PWMR , OUTPUT);
            pinMode(DIR1R, OUTPUT);
            pinMode(DIR2R, OUTPUT);
            attachInterrupt(digitalPinToInterrupt(ENCODER1L),encoderCountL,RISING);
            attachInterrupt(digitalPinToInterrupt(ENCODER1R),encoderCountR,RISING);

            setMotorPwm(0,0);
            setPosition(0,0,0);
		}
		void setParameters(float baseWidth, float wheelRadius, int ticksPerRotation) {
			this->baseWidth        = baseWidth;
			this->wheelRadius      = wheelRadius;
			this->ticksPerRotation = ticksPerRotation;
		}
		void setPIDs(float Kp,float Ki, float Kd) {
			this->baseWidth        = baseWidth;
			this->wheelRadius      = wheelRadius;
			this->ticksPerRotation = ticksPerRotation;
			pidL.setCoefficients(Kp,Ki,Kd);
			pidL.setOutputRange(-255,255,50);
			pidR.setCoefficients(Kp,Ki,Kd);
			pidR.setOutputRange(-255,255,55);
		}
		void setPosition(float x, float y, float yaw) {
			this->_x   = x;
			this->_y   = y;
			this->_yaw = yaw;
		}
		void setDebug(bool debug) {
			this->debug = debug;
		}
		void loop() {
			float dt = (millis()-loopTime)/1000.0;
			int32_t newTicksL = countL;
			int32_t newTicksR = countR;
			float dl = (2*PI*wheelRadius*(newTicksL-ticksL))/ticksPerRotation;
			float dr = (2*PI*wheelRadius*(newTicksR-ticksR))/ticksPerRotation;
			float df = (dr+dl)/2;
			float dx = df*cos(_yaw);
			float dy = df*sin(_yaw);
			float dyaw = atan2(dr-dl,baseWidth);
			curVR = dr/dt;
			curVL = dl/dt;
			curW  = dyaw/dt;
			_x   += dx;
			_y   += dy;
			_yaw += dyaw;
			if (speedMode) {
				pwmL = (goalVL==0 ? 0 : pidL.update(goalVL-curVL,dt));
				pwmR = (goalVR==0 ? 0 : pidR.update(goalVR-curVR,dt));
                if (debug) {
                    sprintf(msg,"PWM R:%d,L:%d Ticks R:%ld,L:%ld",pwmR,pwmL,ticksR,ticksL);
                    nh->loginfo(msg);
                }
				setMotorPwm(pwmL,pwmR);
			}
			ticksL    = newTicksL;
			ticksR    = newTicksR;
			loopTime  = millis();
		}
		void setMotorPwm(int16_t pwmL,int16_t pwmR) {
            if (pwmL==0) {
                digitalWrite(DIR1L,0);
                digitalWrite(DIR2L,0);
                analogWrite(PWML,0);
            } else if (pwmL>0) {
                digitalWrite(DIR1L,1);
                digitalWrite(DIR2L,0);
                analogWrite(PWML,pwmL);
            } else {
                digitalWrite(DIR1L,0);
                digitalWrite(DIR2L,1);
                analogWrite(PWML,-pwmL);
            }
            if (pwmR==0) {
                digitalWrite(DIR1R,0);
                digitalWrite(DIR2R,0);
                analogWrite(PWMR,0);
            } else if (pwmR>0) {
                digitalWrite(DIR1R,1);
                digitalWrite(DIR2R,0);
                analogWrite(PWMR,pwmR);
            } else {
                digitalWrite(DIR1R,0);
                digitalWrite(DIR2R,1);
                analogWrite(PWMR,-pwmR);
            }
		}
		void setSpeed(float v, float w) {
			speedMode = true;
			goalVR = v+w*baseWidth/2;
			goalVL = v-w*baseWidth/2;
            sprintf(msg,"Speed R:%s,L:%s",String(goalVR).c_str(),String(goalVL).c_str());
            nh->loginfo(msg);
		}
		void stop() {
            setMotorPwm(0,0);
			pidL.reset();
			pidR.reset();
			speedMode = false;
		}

        void setNodeHandler(NodeHandle_t *nodeHandler) {
            nh = nodeHandler;
            pidL.setNodeHandler(nodeHandler);
            pidR.setNodeHandler(nodeHandler);
        }
			
		float x()   { return this->_x;   }
		float y()   { return this->_y;   }
		float yaw() { return this->_yaw; }
	private:
        NodeHandle_t *nh; 
		float baseWidth,wheelRadius;
		int    ticksPerRotation;
		bool   debug;
		float _x,_y,_yaw;
		PID    pidL,pidR;
		bool   speedMode;
		float goalVR,goalVL;
		float curVR,curVL,curW;
		unsigned long loopTime;
		int16_t pwmL,pwmR;
		int32_t ticksL,ticksR;

};
