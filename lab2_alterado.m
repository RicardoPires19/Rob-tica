%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);
a=pioneer_read_sonars();

while ~(a(1)<800 && a(8)>4000)
    tic;
    tStart=tic;
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
    fprintf('fora ciclo diagonal é %f',diag,'com angulo %f',ang, 'var dist esta a %f\n',dist)
    pause(0.2)
    while dist>330 && dist<600
        if toc(tStart)/4>=iter
             t=toc(tStart);
             a=pioneer_read_sonars();
             if length(points)>0
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
                ang=acos(cor/diag);
                ang=real(ang);
                if ang~=0
                    dist=a(1)*cos(ang);
                else
                    dist=a(1);
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
    if length(time)>0
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
        pause(5)   %%pode necessitar de ajuste t esta em sec
        fprintf('fim correcao\n')
        %if toc(tStart)>0.07
        pioneer_set_controls(sp,100,-round(v));
        pause(2.5)
%             fprintf('sidfniasou');
        pioneer_set_controls(sp,100,0);
%         end
    end
end
a(7)
fprintf('saiu')
pioneer_set_controls(sp,0,0)