%function [x1,x2,x3,x4,x5,x6] = robotica()
clear all;
syms x1 x2 x3 x4 x5 x6
    %alpha a d beta
rel=[0   0   99  x1;
    pi/2  30   0   x2+pi/2;
    0   120 0   x3;
    0   25  0  0;               %confirmar
    pi/2 0 140   x4;    
    -pi/2  0  0  x5;    %-pi/2
    pi/2 0 0  x6;        %pi/2
    0 0 25  0]


for i=1:8
    transf(:,:,i)=[ cos(rel(i,4)) -sin(rel(i,4)) 0 rel(i,2);
        sin(rel(i,4))*cos(rel(i,1)) cos(rel(i,4))*cos(rel(i,1)) -sin(rel(i,1)) -sin(rel(i,1))*rel(i,3);
        sin(rel(i,4))*sin(rel(i,1)) cos(rel(i,4))*sin(rel(i,1)) cos(rel(i,1)) cos(rel(i,1))*rel(i,3);
        0                           0                            0             1];
end

R06=transf(:,:,1)*transf(:,:,2)*transf(:,:,3)*transf(:,:,4)*transf(:,:,5)*transf(:,:,6)*transf(:,:,7)*transf(:,:,8)   %0R6

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


m6=R06;
x1=0;
x2=pi/4;
x3=-pi/3;
x4=0;
x5=pi/3;
x6=pi/3;
m6=subs(m6)

rot1=[cos(x1) -sin(x1) 0;
    sin(x1) cos(x1) 0;
    0     0      1]

rot2=[cos(-pi/2-pi/3+pi/2) 0 sin(-pi/2-pi/3+pi/2);
    0     1       0;
    -sin(-pi/2-pi/3+pi/2)   0 cos(-pi/2-pi/3+pi/2)]

rot3=[1  0   0;
    0 cos(x4+x6+pi) -sin(x4+x6+pi);       
    0 sin(x4+x6+pi) cos(x4+x6+pi)];

end_point=m6(1:3,4)
rotinv=m6(1:3,1:3)

%recuar end_point
R05=transf(:,:,1)*transf(:,:,2)*transf(:,:,3)*transf(:,:,4)*transf(:,:,5)*transf(:,:,6)*transf(:,:,7);
x1=0;
x2=pi/4;
x3=-pi/3;
x4=0;
x5=pi/3;
x6=pi/3;
R05=subs(R05)

new_end=end_point-R05(1:3,1:3)*[0,0,25]';   %%posso assumir isto?
x=double(new_end(1));
y=double(new_end(2));
z=double(new_end(3));


%calculo para x1 
x1_1=atan2(y,x);
x1_2=pi+x1_1;

hipo=sqrt(25^2+140^2);

%calculo para x2=beta-alpha e x3=pi-aaung_inte
%nao esquecer altura base
hipotenusa=sqrt((x-30)^2+y^2+(z-99)^2);
D=((120)^2+(hipo)^2-hipotenusa^2)/(2*(120)*(hipo)); %%falta considerar ate a ponta
%x3_1=pi-ang_inte1;
%x3_2=pi-ang_inte2;
aux=acos(D);
alpha=pi-aux; 
if alpha<pi/2   %1 e 2quadrante
    x3_1=pi/2-alpha-sin(25/hipo);   %%1 q
   x3_11=pi/2-alpha+sin(25/hipo);  %%2q
    x3_2=pi-x3_1;
    x3_22=pi-x3_11;
else
    x3_1=-(alpha-pi/2+sin(25/hipo));  %%4q
    x3_11=-(alpha-pi/2-sin(25/hipo));  %3q
    x3_2=pi+(x3_1);
    x3_22=pi+(x3_11);
end
%x3_3=pi/2-alpha+sin(25/hipo);
%x3_4=pi/2+alpha-sin(25/hipo);
   
alpha2=pi+aux;


beta=atan2(z-99,sqrt(hipotenusa^2-(z-99)^2));  
aux=atan2(sin(alpha)*hipo,120+cos(alpha)*hipo);   
x2_1=-(pi/2-(beta+aux));
x2_2=-(pi/2-(beta-aux));      %%confirmar sinais
x2_11=-x2_1;   %2q
x2_22=-x2_2;    %2q

R03=transf(1:3,1:3,1)*transf(1:3,1:3,2)*transf(1:3,1:3,3);
[x1,x2,x3,x4,x5,x6]=calculate(x1_1,x2_1,x3_1,R03,rotinv);
[x11,x21,x31,x41,x51,x61]=calculate(x1_1,x2_2,x3_2,R03,rotinv)
[x12,x22,x32,x42,x52,x62]=calculate(x1_2,x2_1,x3_1,R03,rotinv);
[x13,x23,x33,x43,x53,x63]=calculate(x1_2,x2_2,x3_2,R03,rotinv);


%function [x1,x2,x3,x4,x5,x6] = calculate(x1_1,x2_1,x3_1,R03,rotinv)
    %%verificaçoes??
    x1=x1_1;
    x2=x2_1;      %%falta considerar todas as hipoteses
    x3=x3_1;
    R03=double(subs(R03));

    R36=inv(R03)*rotinv;
    %use euler para ter formula da r36

%    r36=transf(1:3,1:3,4)*transf(1:3,1:3,5)*transf(1:3,1:3,6)*transf(1:3,1:3,7)
    %%-cos(x5)=R36(2,3)   cos(x4)*sin(x5)=R36(1,3)
    %%-sin(x5)*sin(x6)=R36(2,2)
    x5=double(acos(-R36(2,3)))
        
   
    x4=double(acos(R36(1,3)/sin(x5)))
    x6=double(acos(R36(2,1)/sin(x5)))  %%x4 e x6 rever complementar
    
%end
