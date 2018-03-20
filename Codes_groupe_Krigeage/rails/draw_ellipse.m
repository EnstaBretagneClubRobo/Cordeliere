function draw_ellipse(wbar,Gw,eta,color1,linewidth1)
if (exist('linewidth1')==0), linewidth1=1; end;
if (exist('color1')==0), color1='black'; end;
s=0:0.01:2*pi; 
 w=wbar*ones(size(s))+sqrtm(-2*log(1-eta)*Gw)*[cos(s);sin(s)];
 plot(w(1,:),w(2,:),color1,'LineWidth',linewidth1); 
end

 
