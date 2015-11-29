% This function convert a vector to a skew matrix
function wh = hat(w)
wh = [0 -w(3) w(2); 
      w(3) 0 -w(1); 
      -w(2) w(1) 0];
end