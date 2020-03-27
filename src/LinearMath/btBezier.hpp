#ifndef BT_BEZIER_H
#define BT_BEZIER_H

namespace btBezier{

template <typename T>
T calcBezier(T* pts, float tau, int degree)
{
	assert(tau>=0 && tau<=1);
	assert(degree>=0);
	
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
		b1 = calcBezier<T>(pts,tau, degree-1);
		b2 = calcBezier<T>(pts+1,tau, degree-1);
	    return ( (1-tau)*b1 + tau*b2 );
	}
}

} // namespace

#endif