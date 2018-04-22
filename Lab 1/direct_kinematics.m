function [x,y,z,angz0,angy,angz1] = direct_kinematics(x1,x2,x3,x4,x5,x6)

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

if R06(3,3)<1
    if R06(3,3)>-1
       angy=acos(R06(3,3))
       angz0=atan2(R06(2,3),R06(1,3))
       angz1=atan2(R06(3,2),-R06(3,1))
    else   %cosbeta=-1
        angy=pi
       angz0=-atan2(R06(2,1),R06(2,2))
       angz1=0
    end
else  %cosbeta=1
    angy=0
    angz0=atan2(R06(2,1),R06(2,2))
    angz1=0
end

end_point=R06(1:3,4);
x=double(end_point(1));
y=double(end_point(2));
z=double(end_point(3));
        