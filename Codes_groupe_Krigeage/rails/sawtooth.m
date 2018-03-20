function y = sawtooth(x,d)  % sawtooth funtion
     % dir=0: nearest,   dir=-1: to the right; dir=1: to the left; 
    if (exist('d')==0), d=0; end;
    y=d*pi+mod(x+pi-d*pi,2*pi)-pi;
    % when dir=0, equivalent to y=2*atan(tan(x/2));
end

