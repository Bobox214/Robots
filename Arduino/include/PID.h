#include <ros.h>

class PID {
	public:
        
		PID() {
			reset();
		}
		void setCoefficients(float kP, float kI, float kD) {
			this->kP = kP;
			this->kI = kI;
			this->kD = kD;
		}
		void setOutputRange(float minOutput,float maxOutput,float neutralZone) {
			this->minOutput   = minOutput;
			this->maxOutput   = maxOutput;
			this->neutralZone = neutralZone;
		}
		float update(float error) {
			return update(error,error-lastError,1);
		}
		float update(float error,float dt) {
			return update(error,(error-lastError)/dt,dt);
		}
		float update(float error,float diffError,float dt) {

			output = kP*error+kI*cumError+kD*diffError;
			if (output<0)
				output -= neutralZone;
			else
				output += neutralZone;
				
			output = constrain(output,minOutput,maxOutput);
			if (output!=minOutput and output!=maxOutput) {
				cumError += error*dt;
			}
			lastError = error;
			return output;
		}
		void reset() {
			error     = 0;
			cumError  = 0;
			diffError = 0;
		}
        void setNodeHandler(NodeHandle_t *nodeHandler) {
            nh = nodeHandler;
        }
	private:
        NodeHandle_t *nh; 
		float neutralZone;
		float minOutput;
		float maxOutput;
		float kI, kP, kD;
		float error, lastError, diffError, cumError;
		float output;
};
