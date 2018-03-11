clear all;
syms x1 x2 x3 x4 x5 x6
rel=[0   0   99  x1;
    pi/2 3   0   x2+pi/2;
    0    120 0   x3;
    pi/2 2.5 0   x4+pi/2;
    pi/2  0  14  x5;
    -pi/2 0 2.5  x6]

names={'t10','t21','t32','t43','t54','t65'}
for i=1:6
    names{i}=[ cos(rel(i,4)) -sin(rel(i,4)) 0 rel(i,2);
        sin(rel(i,4))*cos(rel(i,1)) cos(rel(i,4))*cos(rel(i,1)) -sin(rel(i,1)) -sin(rel(i,1))*rel(i,3);
        sin(rel(i,4))*sin(rel(i,1)) cos(rel(i,4))*sin(rel(i,1)) cos(rel(i,1)) cos(rel(i,1))*rel(i,3);
        0                           0                            0             1];
end

t60=names{1}*names{2}*names{3}*names{4}*names{5}*names{6}