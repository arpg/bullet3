#ifndef BT_BEZIER_H
#define BT_BEZIER_H

/********************************************
 * 
 * Return value of Bezier curve defined by @pts 
 * (which is of size @degree+1) at parameterized 
 * value @tau.
 * 
 * Example:
 * For a cubic bezier (degree = 3), let pts 
 * resemble a  2D sin wave (so T is Vector2f): 
 * pts = { {0.0,     0.0},
 * 		   {pi/2,    1.0},
 * 		   {3*pi/2, -1.0},
 * 		   {2*pi,    0.0} }
 * We have
 * B(t) = (1-tau)^3*pts(0) + 
 * 		  3*tau*(1-tau)^2*pts(1) +
 * 		  3*tau^2*(1-tau)*pts(2) +
 * 		  tau^3*pts(3) 
 * and we would expect at a tau of 0.5,
 * calcBezier(pts, 3, 0.5) will return ~{pi, 0.0}.
 * 
 * ********************************************/

#include "btBulletDynamicsCommon.h"
namespace btBezier{

template <typename T>
T calcBezier(T* pts, int degree, float tau)
{
	btAssert(tau>=0 && tau<=1);
	btAssert(degree>=0);
	
	if (degree==0)
    {
        return pts[0];
    }
    else if (degree==1)
	{
		return (1-tau)*pts[0] + tau*pts[1];
	}
	else
	{
	    T b1, b2;
		b1 = calcBezier<T>(pts, degree-1, tau);
		b2 = calcBezier<T>(pts+1, degree-1, tau);
	    return ( (1-tau)*b1 + tau*b2 );
	}
}

} // namespace

#endif