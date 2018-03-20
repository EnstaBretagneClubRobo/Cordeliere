% Mezher Mohamad 
% Ouadrhiri Mohamed Amine

%% Simulation des boues qui souvent le courant maritime
clear all ; close all ; clc ; 
%%

init ;
dt = 0.02;
% on considere un z constant
z=5;

%position initial du robot
x_robot=[0;40;0;-5]; 
dt_robot=0.02; 
L=2 ;

q1hat = randi(9,2,1) ; q2hat = randi(9,2,1) ; q3hat = randi(9,2,1) ; q4hat = randi(9,2,1) ;
v1hat = randi(9,2,1) ; v2hat = randi(9,2,1) ; v3hat = randi(9,2,1) ; v4hat = randi(9,2,1) ; 

   
x1 = [10;-5;8;8]; x2 = [50;-10;8;8];x3=[100;0;8;8];x4=[80;-20;8;8];
y1=[0,0]; y2=randi(9,4,1);y3=randi(9,4,1);y4=randi(9,4,1);

for t = 0:dt:40,
    
    % position initial des cibles
    p1hat = [t+10;t-5];
    p2hat = [t+50;t-10];
    p3hat = [t+100;t];
    p4hat = [t+80;t-20];

    clf ; hold on ; axis([0 150 -40 40]);
    nq1 = x1(1:2) - q1hat ; 
    nq2 = x2(1:2) - q2hat ;
    nq3 = x3(1:2) - q3hat ;
    nq4 = x4(1:2) - q4hat ;
    
    w1 = v1hat - 2 *(x1(1:2) - p1hat) + nq1/(norm(nq1)^3);
    vbar1 = norm(w1) ; thetabar1 = atan2(w1(2),w1(1));

    
    w2 = v2hat - 2 *(x2(1:2) - p2hat) + nq2/(norm(nq2)^3);
    vbar2 = norm(w2) ; thetabar2 = atan2(w2(2),w2(1));    

    w3 = v3hat - 2 *(x3(1:2) - p3hat) + nq3/(norm(nq3)^3);
    vbar3 = norm(w3) ; thetabar3 = atan2(w3(2),w3(1));

    w4 = v4hat - 2 *(x4(1:2) - p4hat) + nq4/(norm(nq4)^3);
    vbar4 = norm(w4) ; thetabar4 = atan2(w4(2),w4(1));
    
    
    u1=[vbar1-x1(3); 10*atan(tan(0.5*(thetabar1-x1(4))))];
    u2=[vbar2-x2(3); 10*atan(tan(0.5*(thetabar2-x2(4))))];    
    u3=[vbar3-x3(3); 10*atan(tan(0.5*(thetabar3-x3(4))))];
    u4=[vbar4-x4(3); 10*atan(tan(0.5*(thetabar4-x4(4))))];
    
    % Save boue1 data in b1.cvs
    x1=x1+f(x1,u1)*dt;
    b1 = [x1(1) , x1(2) , t , z ];
    dlmwrite('b1.csv',b1, '-append')
    
    % Save boue2 data in b2.cvs
    x2=x2+f(x2,u2)*dt;
    b2 = [x2(1) , x2(2) , t , z ];
    dlmwrite('b2.csv',b2,'-append')


    % Save boue3 data in b3.cvs
    x3=x3+f(x3,u3)*dt;
    b3 = [x3(1) , x3(2) , t , z ];
    dlmwrite('b3.csv',b3,'-append')
    
    % Save boue4 data in b4.cvs
    x4=x4+f(x4,u4)*dt;
    b4 = [x4(1) , x4(2) , t , z ];
    dlmwrite('b4.csv',b4,'-append')

    draw_field(p1hat , q1hat , v1hat);
    draw_tank(x1([1,2,4]) , 'b' , 0.1);
    draw_tank(x2([1,2,4]) , 'b' , 0.1);
    draw_tank(x3([1,2,4]) , 'b' , 0.1);
    draw_tank(x4([1,2,4]) , 'b' , 0.1);
    drawnow();
    
    end

