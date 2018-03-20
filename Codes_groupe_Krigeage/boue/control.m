function u = control(x,w,dw)
    A = [-x(4)*sin(x(3))   cos(x(3)) ;
          x(4)*cos(x(3))   sin(x(3)) ] ;
      
  
    
    y=[x(1);x(2)] ; dy = [x(4)*cos(x(3)) ; x(4)*sin(x(3))];
    ddy = 100*sign(w-y+dw-dy);
    u=A\ddy ;
end
