function draw_arc(c,a,theta,color,linewidth) 
% c:center; a:initial point
s=0:0.01:abs(theta);
s=sign(theta)*s;
r=norm(a-c); alpha=angle(a-c);
w=c*ones(size(s))+r*[cos(alpha),-sin(alpha);sin(alpha),cos(alpha)]*[cos(s);sin(s)];
plot(w(1,:),w(2,:),color,'LineWidth',linewidth);
end

 
