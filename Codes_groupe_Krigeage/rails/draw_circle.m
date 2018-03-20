function draw_circle(c,r,color1,linewidth1)
if (exist('linewidth1')==0), linewidth1=1; end;
if (exist('color1')==0), color1='black'; end;
s=0:0.01:2*pi; 
 w=c*ones(size(s))+r*[cos(s);sin(s)];
 plot(w(1,:),w(2,:),color1,'LineWidth',linewidth1);
end

 
