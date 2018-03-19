clear all;
syms x1 x2 x3 x4 x5 x6
rel=[0   0   99  x1;
    pi/2 30   0   x2+pi/2;
    0    120 0   x3;
    pi/2 25 0   x4+pi/2;
    pi/2  0  140  x5;
    -pi/2 0 0   0;
    0 0 25  x6]


for i=1:7
    transf(:,:,i)=[ cos(rel(i,4)) -sin(rel(i,4)) 0 rel(i,2);
        sin(rel(i,4))*cos(rel(i,1)) cos(rel(i,4))*cos(rel(i,1)) -sin(rel(i,1)) -sin(rel(i,1))*rel(i,3);
        sin(rel(i,4))*sin(rel(i,1)) cos(rel(i,4))*sin(rel(i,1)) cos(rel(i,1)) cos(rel(i,1))*rel(i,3);
        0                           0                            0             1];
end

R06=transf(:,:,1)*transf(:,:,2)*transf(:,:,3)*transf(:,:,4)*transf(:,:,5)*transf(:,:,6)*transf(:,:,7)   %0R6

rotz=[cos(x1) -sin(x1) 0;
    sin(x1) cos(x1) 0;
    0     0      1]

roty=[cos(x2+x3+pi/2) 0 sin(x2+x3+pi/2);     %%nao e em torno de y mas sim de -y da frame 0 
    0     1       0;                        
    -sin(x2+x3+pi/2)   0 cos(x2+x3+pi/2)]

rotx=[1  0   0;
    0 cos(x4+x6+pi/2) -sin(x4+x6+pi/2);
    0 sin(x4+x6+pi/2) cos(x4+x6+pi/2)];

rotz2=[cos(x5) -sin(x5) 0;
    sin(x5) cos(x5) 0;
    0   0   1];

rot=rotz*roty*rotx;
rot2=rotz*roty*rotz2;

%%inverse
%end_point=[px,py+2.5,pz,1]';
%new_end=names{1}*names{2}*names{3}*names{4}*names{5}*end_point


matrix=R06;
m1=subs(matrix,x1,0);
m2=subs(m1,x2,pi/2);   
m3=subs(m2,x3,pi/3);
m4=subs(m3,x4,0);
m5=subs(m4,x5,pi);   
m6=subs(m5,x6,0);

rot1=[cos(0) -sin(0) 0;
    sin(0) cos(0) 0;
    0     0      1]

rot2=[cos(-pi/2-pi/3+pi/2) 0 sin(-pi/2-pi/3+pi/2);
    0     1       0;
    -sin(-pi/2-pi/3+pi/2)   0 cos(-pi/2-pi/3+pi/2)]

rot3=[1  0   0;
    0 cos(pi) -sin(pi);       
    0 sin(pi) cos(pi)];


end_point=matrix(:,4)
rotinv=matrix(1:3,1:3)

new_end=end_point-25*rotinv(:,3)';   %%posso assumir isto?
x=new_end(1);
y=new_end(2);
z=new_end(3);
%calculo para x1 
x1_1=atan2(y,x);
x1_2=pi+x1_1;

%calculo para x2=beta-alpha e x3=pi-ang_inte
%nao esquecer altura base
hipotenusa=sqrt(x^2+y^2)
D=(120^2+140^2-hipotenusa)/(2*120*140)
ang_inte1=acos(D)
ang_inte2=-acos(D)
x3_1=pi-ang_inte1;
x3_2=pi-ang_inte2;

beta=atan2(z-99,sqrt(hipotenusa-(z-99)^2))  %base
x2_1=beta-atan2(sin(x3_1)*140,120+cos(x3_1)*140)
x2_2=beta-atan2(sin(x3_2)*140,120+cos(x3_2)*140)

%%verificaçoes??
R03=transf(1:3,1:3,1)*transf(1:3,1:3,2)*transf(1:3,1:3,3);
subs(R03,x1,x1_1);
subs(R03,x2,x2_1);   %%%como escolher opçao???
subs(R03,x3,x3_1);

R36=rotinv*inv(R03);
%use euler con
