function R=eulermat(phi,theta,psi)
%          cf=cos(phi);         sf=sin(phi);
%          ct=cos(theta);       st=sin(theta);
%          cp=cos(psi);         sp=sin(psi);
%          R=[ ct*cp  -cf*sp+st*cp*sf    sp*sf+st*cp*cf ;
%              ct*sp   cp*cf+st*sp*sf   -cp*sf+st*cf*sp;
%              -st       ct*sf             ct*cf     ];
%     ou bien en utilisant la formule de Rodrigues     
     Apsi=psi*[0 -1 0;1 0 0;0 0 0];
     Atheta=theta*[0 0 1;0 0 0;-1 0 0];
     Aphi=phi*[0 0 0;0 0 -1;0 1 0];
     R=expm(Apsi)*expm(Atheta)*expm(Aphi);
end

