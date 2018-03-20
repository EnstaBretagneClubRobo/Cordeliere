
%-----------  Main  ------------
x_robot=[0;30;1;5]; 
dt_robot=0.02; 
L=2 ;
for t=0:dt_robot:40,
    clf; axis([0,150,-30,30]); axis square; hold on;
    s=0:0.01:50 ; 
    p=L*[s;30*cos(s)];
    w = L*[t;30*cos(t)];
    dw=L*[t;-30*sin(t)];
    draw_tank(x_robot,'red');  
    u=control(x_robot,w,dw);
    x_robot=x_robot+f2(x_robot,u)*dt;
    drawnow();
end


