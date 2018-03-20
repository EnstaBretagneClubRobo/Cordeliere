function draw_arrow(x,y,theta,L,color)
   e=0.2;
   M1=L*[  0  1  1-e 1 1-e ; 
	     0  0   -e 0  e ];
   M =[  M1  ; 1 1 1 1 1];  
   R=[cos(theta),-sin(theta),x;sin(theta),cos(theta),y;0 0 1];    
   M =R*M;
   plot(M(1,:),M(2,:),color);       
end

