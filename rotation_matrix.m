syms theta phi psi real

Rx = [1    0          0; 
      0  cos(phi)  -sin(phi); 
      0  sin(phi)   cos(phi)];  % roll
  
Ry = [cos(theta) 0   sin(theta); 
        0        1        0;
     -sin(theta) 0   cos(theta)];  % pitch
  
Rz = [cos(psi) -sin(psi)   0;
      sin(psi)  cos(psi)   0;
        0          0       1];  % yaw

%R = (Rx*Ry*Rz)';

R = Rz'*Ry'*Rx';  % gives the same result as the commented version