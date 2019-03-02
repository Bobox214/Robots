
class PID {
	public:
		PID() {
			reset();
		}
		void setCoefficients(double kP, double kI, double kD) {
			this->kP = kP;
			this->kI = kI;
			this->kD = kD;
		}
		void setOutputRange(double minOutput,double maxOutput,double neutralZone) {
			this->minOutput   = minOutput;
			this->maxOutput   = maxOutput;
			this->neutralZone = neutralZone;
		}
		double update(double error) {
			return update(error,error-lastError,1);
		}
		double update(double error,double dt) {
			return update(error,(error-lastError)/dt,dt);
		}
		double update(double error,double diffError,double dt) {
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
	private:
		double neutralZone;
		double minOutput;
		double maxOutput;
		double kI, kP, kD;
		double error, lastError, diffError, cumError;
		double output;
};
