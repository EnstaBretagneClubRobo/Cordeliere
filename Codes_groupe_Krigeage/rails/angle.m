function theta = angle(u,v)  
  if (exist('v')==0), 
      theta=atan2(u(2),u(1));  % angle of u with respect to Ox
  else
      theta=sawtooth(angle(v)-angle(u)); %angle between u and v
  end;
end

