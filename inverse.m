clear variables;
syms x1 x2 x3 x4 x5 x6
syms r1 r2 r3 x y z

rel=[0   0   99  x1;
    pi/2 30   0   x2+pi/2;
    0    120 0   x3;
    pi/2 25 0   x4+pi/2;
    pi/2  0  140  x5;
    -pi/2 0 0   0;
    0 0 25  x6];


i = 1;

RotX = [ 1 0 0;
    0 cos(x1) -sin(x1);
    0 sin(x1) cos(x1)];

RotZ = [ cos(x2) -sin(x2) 0;
         sin(x2) cos(x2) 0;
         0 0 1];
 
 BaseTool = RotZ*RotX;
 P = [x y z]';
 BaseTool = [BaseTool P];
 L = [0 0 0 1];
 BaseTool = [BaseTool; L]
 
 R0_1 = [ cos(rel(i,4)) -sin(rel(i,4)) 0 rel(i,2);
        sin(rel(i,4))*cos(rel(i,1)) cos(rel(i,4))*cos(rel(i,1)) -sin(rel(i,1)) -sin(rel(i,1))*rel(i,3);
        sin(rel(i,4))*sin(rel(i,1)) cos(rel(i,4))*sin(rel(i,1)) cos(rel(i,1)) cos(rel(i,1))*rel(i,3);
        0                           0                            0             1]
     
i = 1
DirectR1_6 = [cos(rel(i,4)) -sin(rel(i,4)) 0 rel(i,2);
        sin(rel(i,4))*cos(rel(i,1)) cos(rel(i,4))*cos(rel(i,1)) -sin(rel(i,1)) -sin(rel(i,1))*rel(i,3);
        sin(rel(i,4))*sin(rel(i,1)) cos(rel(i,4))*sin(rel(i,1)) cos(rel(i,1)) cos(rel(i,1))*rel(i,3);
        0                           0                            0             1]

R1_6 = mldivide(R0_1, BaseTool)  % = inv(R0_1) * R0_6

theta = cosh(DirectR1_6(3,3))



