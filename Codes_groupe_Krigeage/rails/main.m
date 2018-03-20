% Mezher Mohamad 
% Ouadrhiri Mohamed Amine
%%
clear all ; close all ; clc ; 
%%
init;

% Tracked line
a= [-100;-100];b=[-100;100];

 
%% Position initial du robot 
x=[-100;-100;4];  % x,y,theta

%%
dt=0.6;
j=0;
for t=0:.01:10,
    clf(); hold on; axis([-100 100 -100 100]); axis square; 
    phi=angle(b-a); 
    m = x(1:2); 
    e=det([b-a,m-a])/norm(b-a); % distance to the line
    thetabar=phi-atan(e);
    u=sawtooth(thetabar-x(3));
    x=x+f(x,u)*dt;
    draw_tank(x,'blue');
    %plot([a(1);b(1)],[a(2);b(2)],'red');
    grid on ;
    drawnow();
    
    if x(1)-b(1)< 0.5 && x(1)-b(1)> -0.5 && x(2)- b(2)<0.5 && x(2)-b(2)> -0.5
            if(j==0)
                b(1,:)=b(1,:)+10;
                a=m;
                j=j+1;
            elseif(j==1)
                b(2,:)=b(2,:)-200;
                a=m;
                j=2;
            elseif(j==2)
                 b(1,:)=b(1,:)+10;
                 a=m;
                j=3;
            else
                b(2,:)=b(2,:)+200;
                a=m;
                j=0;
            end
    end
end