function [result]= inverse_kinematics(x,y,z,angz0,angy,angz1)

syms x1 x2 x3 x4 x5 x6;
rel=[0   0   99  x1;
    pi/2  30   0   x2+pi/2;
    0   120 0   x3;
    0   25  0  0;               %confirmar
    pi/2 0 140   x4;    
    -pi/2  0  0  x5;    %-pi/2
    pi/2 0 0  x6;        %pi/2
    0 0 25  0];


for i=1:8
    transf(:,:,i)=[ cos(rel(i,4)) -sin(rel(i,4)) 0 rel(i,2);
        sin(rel(i,4))*cos(rel(i,1)) cos(rel(i,4))*cos(rel(i,1)) -sin(rel(i,1)) -sin(rel(i,1))*rel(i,3);
        sin(rel(i,4))*sin(rel(i,1)) cos(rel(i,4))*sin(rel(i,1)) cos(rel(i,1)) cos(rel(i,1))*rel(i,3);
        0                           0                            0             1];
end

R06=transf(:,:,1)*transf(:,:,2)*transf(:,:,3)*transf(:,:,4)*transf(:,:,5)*transf(:,:,6)*transf(:,:,7)*transf(:,:,8);   %0R6


rotz0=[cos(angz0) -sin(angz0) 0;
    sin(angz0) cos(angz0) 0;
    0     0      1];

roty=[cos(angy) 0 sin(angy);     %%nao e em torno de y mas sim de -y da frame 0 
    0     1       0;                        
    -sin(angy)   0 cos(angy)];

rotz1=[cos(angz1) -sin(angz1) 0;
    sin(angz1) cos(angz1) 0;
    0     0      1];

rotinv=rotz0*roty*rotz1;
end_point=[x,y,z];

new_end=end_point'-rotinv*[0,0,25]';
nx=double(new_end(1));
ny=double(new_end(2));
nz=double(new_end(3));

%calculo para x1 
x1_1=atan2(ny,nx);
x1_2=pi+x1_1;

hipo=sqrt(25^2+140^2);

opc=[]
for i=1:2
    if i==1
        x1=x1_1;
        hipotenusa=sqrt((abs(nx)-30*abs(cos(x1)))^2+(abs(ny)-30*abs(sin(x1)))^2+(abs(nz)-99)^2);    %%erro x-30 qundo x1 muda
    else 
        x1=x1_2;
        hipotenusa=sqrt((abs(nx)+30*abs(cos(x1)))^2+(abs(ny)+30*abs(sin(x1)))^2+(abs(nz)-99)^2); 
    end
    D=((120)^2+(hipo)^2-hipotenusa^2)/(2*(120)*(hipo)); %%falta considerar ate a ponta
    if D>1 || D<-1
        continue
    end

    aux=acos(D);
    alpha=pi-aux; 
    if alpha<pi/2   %1 e 2quadrante
        x3_1=pi/2-alpha-sin(25/hipo);   %%1 q
       x3_11=pi/2+alpha-sin(25/hipo);  %%2q
    else
        x3_1=-(alpha-pi/2+sin(25/hipo));  %%4q
        x3_11=-(alpha+pi/2-sin(25/hipo));  %3q
        x3_2=pi+(x3_1);
        x3_22=pi+(x3_11);
    end

    alpha2=pi+aux;

    beta=atan2(abs(nz)-99,sqrt(hipotenusa^2-(abs(nz)-99)^2));  
    aux=atan2(sin(alpha)*hipo,120+cos(alpha)*hipo);   
    x2_1=pi/2-(beta+aux);   %este dois n fazem sentido com x1_2
    x2_2=pi/2-(beta-aux);      %%confirmar sinais
    x2_11=-x2_1;   %1q     
    x2_22=-x2_2;    %1q
    
    opc=[opc; x1 x2_11 x3_1;
        x1 x2_22 x3_11;
        x1 x2_2 x3_1;
        x1 x2_1 x3_11]
    
end


result=[]
s=size(opc)
bla=[]
flag=true;
for i=1:s(1)
     M02=transf(:,:,1)*transf(:,:,2);
    x1=opc(i,1)
    x2=opc(i,2) 
    M02=double(subs(M02));
    T02=M02(1:3,4)    %considerar desnivel
    
    M03=M02*transf(:,:,3);
    x3=opc(i,3)
    M03=double(subs(M03));
    R03=M03(1:3,1:3);
    T03=M03(1:3,4)
    
    M04=M03*transf(:,:,4)*transf(:,:,5);
    x4=0;    %posso considerar isto pq x4 so define rotacao
    M04=double(subs(M04));
    T04=M04(1:3,4)
    
    if flag
        v=T03-T02;
        yy=(T03(2)-T04(2))/v(2)
        zz=(T03(3)-yy*v(3))
        xx=(T03(1)-yy*v(1))
        if (xx-30<T04(1) && T04(1)<xx+30) && (zz-30<T04(3) && T04(3)<zz+30)
            flag=false;
            msgbox('elbow singularity')
        end
        if -0.5<T04(1) && T04(1)<0.5 && -0.5<T04(2) && T04(2)<0.5
            flag=false;
            msgbox('shoulder singularity')
        end
    end
    R36=double(R03.'*rotinv);
    
    x5=atan2(sqrt(R36(1,3)^2+R36(3,3)^2),-R36(2,3));
    if and(x5<0.01, x5>-0.01)
        x6=0;
        x4=double(atan2(R36(1,2),R36(1,1)));
        
    else
        x4=atan2(R36(3,3),R36(1,3));
        x6=atan2(-R36(2,2),R36(2,1));
    end
    bla=[bla; x1 x2 x3 x4 x5 x6];
    Rfinal=transf(:,:,1)*transf(:,:,2)*transf(:,:,3)*transf(:,:,4)*transf(:,:,5)*transf(:,:,6)*transf(:,:,7)*transf(:,:,8);
    Rfinal=double(subs(Rfinal));
       
    if Rfinal(1,4)<x+1 && Rfinal(1,4)>x-1 && Rfinal(2,4)<y+1 && Rfinal(2,4)>y-1 && Rfinal(3,4)<z+1 && Rfinal(3,4)>z-1 
        result=[result; x1 x2 x3 x4 x5 x6];
    end
    if flag
        if (x5>pi/2-0.01 && x5<pi/2+0.01) || (x5>-pi/2-0.01 && x5<-pi/2+0.01)
            flag=false;
            msgbox('wrist singularity')
        end
    end
end    