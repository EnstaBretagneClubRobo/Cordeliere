    function  xdot2  = f2(x,u)   % state : x =(x,y,theta,v)
        xdot2=[x(4)*cos(x(3)); x(4)*sin(x(3)); u(1); u(2)];
    end