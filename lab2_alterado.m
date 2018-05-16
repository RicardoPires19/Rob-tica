%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);
a=pioneer_read_sonars();
b=pioneer_read_odometry();
num_voltas=1;
last_odom=0;
while a(4)>200   %~(a(1)<800 && a(8)>4000)
    b=pioneer_read_odometry();
    pos=1;
    points=[];
    iter=1;
    time=[];
    t=0;
    
    %%n esquecer mudar
    a=pioneer_read_sonars();
    diag=280+a(1)+a(8);
    cor=1650;
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
        b=pioneer_read_odometry();
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
             pause(0.4);
             
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
             if ((dist>500 || dist<330) && length(points)>1) || (abs(b(pos))>9500+last_odom && length(points)>3)
                 break;
             end
             
             while abs(b(pos))>10000+last_odom
                 a=pioneer_read_sonars();
                 fprintf('4: %d 5: %d 8: %d', a(4), a(5), a(8))
                 if a(4)<1400 || a(5)<1400 || a(8)>1300
                    fprintf('curvaaaaaa')
                     pioneer_set_controls(sp, 100, -5);
                     pause(15.5)
                     pioneer_set_controls(sp, 100, 0);
                     points=[];
                     time=[];
                     t=0;
                     b=pioneer_read_odometry();
                     if num_voltas==1 || num_voltas==3
                        last_odom=abs(b(2));
                        num_voltas=num_voltas+1;
                        pos=2;
                        if num_voltas==3
                            last_odom=-last_odom;
                        end
                     else
                        last_odom=abs(b(1));
                        num_voltas=num_voltas+1;
                        pos=1;
                        if num_voltas==4
                            last_odom=-last_odom;
                        end
                     end
                 end
                 break;
             end
             
             
             fprintf('valor dist %f e angulo %f na iteracao %d\n',dist,ang, iter)
         end
       
        %if length(points)>2 && (points(length(points))- points(length(points)-1))>0.15
        %    fprintf('break\n');
        %    break;
        %end
    end
    fprintf('off')
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
        pioneer_set_controls(sp,100,round(v));
        fprintf('valor velocidade %f aplicada durante %f tempo\n',v,t)
        pause(t/4)   %%pode necessitar de ajuste t esta em sec
        fprintf('fim correcao\n')
        %if toc(tStart)>0.07
        pioneer_set_controls(sp,100,-round(v));
        pause(t/8)
%             fprintf('sidfniasou');
        pioneer_set_controls(sp,100,0);
%         end
    end
end
a(7)
fprintf('saiu')
pioneer_set_controls(sp,0,0)