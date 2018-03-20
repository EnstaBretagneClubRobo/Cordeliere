function draw_field(phat,qhat,vhat)

    X = 0:1.5:150 ; Y = -60:1.5:60;
    v1hat = randi(9,2,1);
    [P1,P2] = meshgrid(X,Y);
    % uniforme tlo3
    VX1 = 0*P1 + v1hat(1) ;
    VY1 = 0*P2 + v1hat(2) ;
    quiver (X,Y,VX1,VY1); 
end