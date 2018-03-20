    function xdot=f(x,u)
        theta=x(3);
        xdot=[cos(theta);sin(theta);u];
    end