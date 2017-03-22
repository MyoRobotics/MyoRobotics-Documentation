typedef struct
{
	uint32 tag;/*!<Tag to indicate data type when passing the union*/
	sint32 outputPosMax; /*!< maximum control output in the positive direction in counts, max 4000*/
	sint32 outputNegMax; /*!< maximum control output in the negative direction in counts, max -4000*/
	float32 spPosMax;/*<!Positive limit for the set point.*/
	float32 spNegMax;/*<!Negative limit for the set point.*/
	float32 timePeriod;/*!<Time period of each control iteration in microseconds.*/
	float32 radPerEncoderCount; /*!output shaft rotation (in rad) per encoder count */
	float32 polyPar[4]; /*! polynomial fit from displacement (d)  to tendon force (f) 
	                        f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ / */
	float32 torqueConstant; /*!motor torque constant in Nm/A */

	parameters_t params;

}control_Parameters_t;


typedef union
{
	pid_Parameters_t pidParameters;
}parameters_t;


typedef struct
{
	float32 integral;/*!<Integral of the error*/
	float32 pgain;/*!<Gain of the proportional component*/
	float32 igain;/*!<Gain of the integral component*/
	float32 dgain;/*!<Gain of the differential component*/
	float32 forwardGain; /*!<Gain of  the feed-forward term*/
	float32 deadBand;/*!<Optional deadband threshold for the control response*/
	float32 lastError;/*!<Error in previous time-step, used to calculate the differential component*/
	float32 IntegralPosMax; /*!<Integral positive component maximum*/
	float32 IntegralNegMax; /*!<Integral negative component maximum*/
}pid_Parameters_t;

