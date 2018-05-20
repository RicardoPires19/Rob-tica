%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);
a=pioneer_read_sonars();
b=pioneer_read_odometry();
num_voltas=1;
last_odom=0;
value=11000;
pos=1;
flag=true;
while a(4)>200   %~(a(1)<800 && a(8)>4000)
    fprintf('begin\n')
    b=pioneer_read_odometry();
    points=[];
    iter=1;
    time=[];
    t=0;
    
    %%n esquecer mudar
    a=pioneer_read_sonars();
    diag=280+a(1)+a(8);
    cor=1660;
    ang=acos(cor/diag);
    ang=real(ang);
    if ang~=0
        dist=a(1)*cos(ang);
    else
        dist=a(1);
     %v=0
    end
    fprintf('fora ciclo diagonal é %f com angulo %f var dist esta a %f\n \n',diag,ang,dist)
    pause(0.2)
    tStart=tic;
    while true
        if toc(tStart)/2>=iter
             t=toc(tStart);
             a=pioneer_read_sonars();
             if ~isempty(points)
                 if a(1)-points(length(points))*1000<200
                    points=[points;a(1)*10^-3];    %%esta em metros certo?
                    time=[time;t];
                    fprintf('nao porta')
                 end
             else
                 points=[points;a(1)*10^-3];    %%esta em metros certo?
                 time=[time;t];
             end
             iter=iter+1;
             pause(0.2);
             
             %mudarrrrrrrrrrrrrrrrrrrr
             if a(8)~=5000    %ultimo corredor
                diag=280+a(1)+a(8);
                
                fprintf('valores medidos %d %d \n',a(1),a(8))
                ang=acos(cor/diag);
                ang=real(ang);
                if ang~=0
                    dist=a(1)*cos(ang);
                else
                    dist=a(1);
                end
             end
             b=pioneer_read_odometry();
             fprintf('odometry %d %d \n', b(1), b(2) )
             
             if length(points)>3
%                  if abs(b(pos))>9500+abs(last_odom)
%                      fprintf('ultima correcao \n')
%                      break;
%                  end
                 if dist>500 && (points(length(points))>points(length(points)-1)) 
                     fprintf('fora de limites \n')
                     break;
                 end
                 if dist<330 && (points(length(points))<points(length(points)-1)) 
                     fprintf('fora de limites \n')
                     break;
                 end
             end
%              if length(time)>10
%                  break;
%              end
             
             while abs(b(pos))>value
                 a=pioneer_read_sonars();
                 b=pioneer_read_odometry();
                 %fprintf('4: %d 5: %d 8: %d \n', a(4), a(5), a(8))
                 %fprintf('odometry %d %d \n', b(1), b(2) )
                 while abs(c-0.8727)>0.05
                     if a(2)<4000 && a(1)<4000
                        d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                        c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                     else
                         break;
                     end
                 end
                 fprintf('entrouuuuuuuuuuu')
                 if a(4)<1500 || a(5)<3300 
                    fprintf('curvaaaaaa')
                     pioneer_set_controls(sp, 100, -7);
                     pause(11.5)
                     pioneer_set_controls(sp, 100, 0);
                     points=[];
                     time=[];
                     t=0;
                     b=pioneer_read_odometry();
                     if num_voltas==1
                        last_odom=b(2);
                        value=11500+abs(last_odom);
                        pos=2;       
                     end
                     if num_voltas==2
                        last_odom=b(1);
                        value=11500+abs(last_odom);
                        pos=1;
                     end
                     if num_voltas==3
                        last_odom=b(2);
                        value=11500+abs(last_odom);
                        pos=2;
                     end
                     
                     if num_voltas==4
                        fprintf('ultima volta')
                     end
                     num_voltas=num_voltas+1;
                 flag=false;
                 corrigido=true;
                 end
                 if ~flag 
                    break;
                 end
             end
             
             
             fprintf('valor dist %f e angulo %f na iteracao %d\n',dist,ang, iter)
         end
       
        %if length(points)>2 && (points(length(points))- points(length(points)-1))>0.15
        %    fprintf('break\n');
        %    break;
        %end
    end
    fprintf('off')
    if flag
        if ~isempty(time)
            [f, n] = polyfit(time, points, 1);
            %angle = atan( f(1) + f(2) );
            %angle = rad2deg(angle);
            %v=angle/time(length(time));
            %if( f(1) >= 0)
            angle = atan( (f(1) + f(2)) );
            %else
    %             x = - ( f(2)/f(1) );
    %             y = f(2);
    %             h = sqrt( x^2 + y^2 );
    %             angle = acos( x / h );
    %         end
            angle = rad2deg(angle);
            v=angle/time(length(time));
            if ( f(1) > 0)
                v=v;
            else 
                v=-v;  %dps tirar redundante
            end
            t=time(length(time));

            %tStart=tic;
            while dist>500 || dist<330 
                b=pioneer_read_odometry();
                if abs(b(pos))>value
                    break;
                end
                fprintf('dist %d \n', dist)
                pioneer_set_controls(sp,100,round(v));
                %fprintf('valor velocidade %f aplicada durante %f tempo\n',v,t)
                if t<20
                    pause(t/4)   %%pode necessitar de ajuste t esta em sec
                else
                    pause(5)
                end
                fprintf('fim correcao para ficar nos limites %f %f\n',ang, dist)
                %if toc(tStart)>0.07
%                 pioneer_set_controls(sp,100,-round(v));
%                 pause(t/8)
%                 pioneer_set_controls(sp,100,0);
                a=pioneer_read_sonars();
                diag=280+a(1)+a(8);
                ang=acos(cor/diag);
                ang=real(ang);
                dist=a(1)*cos(ang);
                cont=0
                stop=tic;
                c=0;
                while abs(c-0.8727)>0.05 || cont==0 || toc(stop)<t/4
                    fprintf('olaaaa %d \n',cont)
                    cont=1;
                    pioneer_set_controls(sp,100,-round(v));
                    pause(0.1)
                    a=pioneer_read_sonars();
                    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
%                         if c>1
%                             pioneer_set_controls(sp,100,round(v));
%                         end
                        %stop=tic;
%                         if angg-ang<0.7
%                             ang=[ang;angg];
%                         end
                    if toc(stop)>t/5
                        fprintf('demasiado tempo a endireitar') 
                        break;
                    end
                    
                    fprintf('fim endireitar angulo %f \n', c)
                end
                if abs(b(pos))>value
                    break;
                end
                a=pioneer_read_sonars();
                diag=280+a(1)+a(8);
                ang=acos(cor/diag);
                ang=real(ang);
                dist=a(1)*cos(ang);
            end
            pioneer_set_controls(sp,100,0);
        end
    else

        while abs(c-0.8727)>0.05
            a=pioneer_read_sonars();
            if a(2)<4000 && a(1)<4000
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            end
            pioneer_set_controls(sp,0,round(8));
            fprintf('fim endireitar dps da curvaaaaaaaa\n')
        end
        pioneer_set_controls(sp,100,0);
        flag=true;
    end
end
a(7)
fprintf('saiu')
pioneer_set_controls(sp,0,0)