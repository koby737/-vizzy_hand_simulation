#include "pid.h" 
using namespace std;

pid::pid(float p, float i, float d, float _T, float _a){
	//warning: should check args
	ep = 0.0;
	yp = 0.0;
	ip = 0.0;
	dp = 0.0;

	kp = p;
	ki = i;
	kd = d;
	T = _T;
	a = _a;

	k1 = kp;
	k2 = kp*ki*T/2;

	if(kd < 0.000001)
		derivative = false;
	else{
		derivative = true;
		float den = kd + a*T;
		k3 = kd/den;
		k4 = kp*kd*a/den;
	}
}

float pid::calc(float ref, float y, float uff){

	float e = ref - y;
	e = deadzone(e, 0.00); 
	float p = k1*e;
	float i = ip + k2*(e + ep);
	float d = 0;

	if(derivative)
		d = k3*dp - k4*(y - yp);

	// anti-windup
	float ufb = p + i + d;
	float w = p + d;
	if(uff + ufb > umax)
    		i = umax - uff - w;
	else if(uff + ufb < umin)
		i = umin - uff - w;

	yp = y;
	ip = i;
	dp = d;
	ep = e;

	return p + i + d;
}

float pid::deadzone(float e, float err){
	if(e >= err)
		return e - err;
	else if(e <= -err)
		return e + err;
	else
		return 0;
}
