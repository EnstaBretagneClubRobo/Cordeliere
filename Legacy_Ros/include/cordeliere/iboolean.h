#ifndef __IBOOLEAN__
#define __IBOOLEAN__
#include <iostream>

// IBOOLEAN is an interval Boolean. Used for a trivalued logic
// if x=[0,0]=ifalse  : certainly false
// if x=[0,1]=iperhaps: don't know
// if x=[1,1]=itrue  : certainly true
// Otherwise x=empty
enum IBOOLEAN {itrue, ifalse, iperhaps, empty};

class iboolean
{
public:
	IBOOLEAN value;
public:
	iboolean ();
	iboolean (bool);
	iboolean (IBOOLEAN);
	iboolean (const iboolean&);
	friend iboolean operator&& (iboolean, iboolean);
        friend iboolean operator|| (iboolean, iboolean);
        friend iboolean operator!  (iboolean);
	friend bool		 operator== (iboolean, iboolean);
	friend bool		 operator!= (iboolean, iboolean);
        friend iboolean Not(iboolean x);
	friend iboolean Inter(iboolean,iboolean);
        friend iboolean Union(iboolean, iboolean);
        friend iboolean And(iboolean,iboolean);
        friend iboolean Or(iboolean,iboolean);
        friend iboolean leq(iboolean x, iboolean y);
        friend iboolean geq(iboolean x, iboolean y);
        friend iboolean Restrict(iboolean x, iboolean y);      // return x and !y
        friend iboolean Xor(iboolean x, iboolean y);



        friend std::ostream& operator<< (std::ostream& os, const iboolean&);

};

#endif
