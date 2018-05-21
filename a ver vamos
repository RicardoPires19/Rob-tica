%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);
a=pioneer_read_sonars();
b=pioneer_read_odometry();
num_voltas=1;
last_odom=0;
value=11500;
pos=1;
flag=true;
while a(4)>200   %~(a(1)<800 && a(8)>4000)
    fprintf('begin\n')
    b=pioneer_read_odometry();
    points=[];
    iter=1;
    time=[];
    t=0;
    
    a=pioneer_read_sonars();
    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
    ang=pi-c;
    dist=sin(ang)*a(1);
    fprintf('nova distancia %d \n',dist)
    
    %%n esquecer mudar
    %     a=pioneer_read_sonars();
    %     diag=280+a(1)+a(8);
    %     cor=1660;
    %     ang=acos(cor/diag);
    %     ang=real(ang);
    %     if ang~=0
    %         dist=a(1)*cos(ang);
    %     else
    %         dist=a(1);
    %      %v=0
    %     end
    %     fprintf('distancia actual %d \n',dist)
    fprintf('fora ciclo com angulo %f var dist esta a %f \n', ang, dist)
    pause(0.2)
    tStart=tic;
    while true
        if toc(tStart)/2>=iter
            t=toc(tStart);
            a=pioneer_read_sonars();
            if ~isempty(points)
                if abs( mean(points - a(1)^10*-3) ) < 200
                    points=[points;a(1)*10^-3];    %%esta em metros certo?
                    time=[time;t];
                    fprintf('nao porta\n')
                end
            else
                points=[points;a(1)*10^-3];    %%esta em metros certo? nope, milimetros
                time=[time;t];
            end
            iter=iter+1;
            pause(0.2);
            
            %mudarrrrrrrrrrrrrrrrrrrr
            %              if a(8)~=5000    %ultimo corredor
            %                 diag=280+a(1)+a(8);
            %
            %                 fprintf('valores medidos %d %d \n',a(1),a(8))
            %                 ang=acos(cor/diag);
            %                 ang=real(ang);
            %                 if ang~=0
            %                     dist=a(1)*cos(ang);
            %                 else
            %                     dist=a(1);
            %                 end
            %              end
            %              fprintf('implementada distancia %d \n',dist)
            a=pioneer_read_sonars();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            fprintf('nova distancia %d , points.length %d \n',dist, length(points))
            b=pioneer_read_odometry();
            fprintf('odometry %d %d \n', b(1), b(2) )
            
            if length(points)>3
                if abs(b(pos))>value
                    fprintf('ultima correcao \n')
                    break;
                end
                if dist>480 && (points(length(points))>points(length(points)-1))
                    fprintf('fora de limites para cima\n')
                    break;
                end
                if dist<290 && (points(length(points))<points(length(points)-1))
                    fprintf('fora de limites para baixo\n')
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
                %                  while abs(c-0.8727)>0.05
                %                      if a(2)<4000 && a(1)<4000
                %                         d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                %                         c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                %                      else
                %                          break;
                %                      end
                %                  end
                fprintf('entrouuuuuuuuuuu\n')
                if abs(b(pos))>value+500
                    if a(4)<1500 || a(5)<3300 || a(8) > 3000
                        fprintf('curvaaaaaa\n')
                        pioneer_set_controls(sp, 100, -7);
                        pause(11.5)
                        pioneer_set_controls(sp, 100, 0);
                        points=[];
                        time=[];
                        t=0;
                        b=pioneer_read_odometry();
                        if num_voltas==1
                            last_odom=b(2);
                            value=11500+abs(last_odom)+500;
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
                    
                end
                
            end
            if ~flag
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
            fprintf('dist antes ciclo %f \n', dist)
            while dist>450 || dist<330
                b=pioneer_read_odometry();
                %                 if abs(b(pos))>value
                %                     break;
                %                 end
                fprintf('dist %d \n', dist)
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                angle=pi-c;
                dist=sin(angle)*a(1);
                pioneer_set_controls(sp,100,ceil(v));
                %fprintf('valor velocidade %f aplicada durante %f tempo\n',v,t)
                %                 if t<20
                %                     pause(t/2)   %%pode necessitar de ajuste t esta em sec
                %                 else
                %                     pause(5)
                %                 end
                pause(3)
                fprintf('fim correcao para ficar nos limites %f %f\n',ang, dist)
                %if toc(tStart)>0.07
                %                 pioneer_set_controls(sp,100,-ceil(v));
                %                 pause(t/8)
                %                 pioneer_set_controls(sp,100,0);
                %                 a=pioneer_read_sonars();
                %                 diag=280+a(1)+a(8);
                %                 ang=acos(cor/diag);
                %                 ang=real(ang);
                %                 dist=a(1)*cos(ang);
                
                cont=0
                stop=tic;
                c=0;
                while abs(c-0.8727)>0.1 || cont==0 || toc(stop)<3  % || toc(stop)<4
                    cont=1;
%                     pioneer_set_controls(sp,100,-ceil(v));
%                     if a(1)>4500 || a(2)>4500
%                         fprintf('sensores na merda')
%                         pioneer_set_controls(sp,100,-abs(ceil(v)));
%                         while dist>450
%                             fprintf('continua a andar dist = %3.2f \n', dist)
%                             a=pioneer_read_sonars();
%                             d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
%                             c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
%                             angle=pi-c;
%                             dist=sin(angle)*a(1);
%                         end
%                         break;
%                             
%                     end
                    pause(0.1)
                    %                         if c>1
                    %                             pioneer_set_controls(sp,100,ceil(v));
                    %                         end
                    %stop=tic;
                    %                         if angg-ang<0.7
                    %                             ang=[ang;angg];
                    %                         end
                    a=pioneer_read_sonars();
                    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                    angle=pi-c;
                    dist=sin(angle)*a(1);
                    if dist>450 || dist<300
                        pioneer_set_controls(sp,100,-ceil(v));
                        pause(4)
                        pioneer_set_controls(sp,100,ceil(v));
                        pause(2)
                        stop=tic;
                    end
                    if toc(stop)>3
                        fprintf('demasiado tempo a endireitar')
                        break;
                    end
                    b=pioneer_read_odometry()
                    if abs(b(pos))>value-1000
                        break;
                    end

                    fprintf('fim endireitar angulo %f \n', c)
                end
                b=pioneer_read_odometry()
                if abs(b(pos))>value-1000
                    break;
                end
                %                 if abs(b(pos))>value
                %                     break;
                %                 end
                %                 a=pioneer_read_sonars();
                %                 diag=280+a(1)+a(8);
                %                 ang=acos(cor/diag);
                %                 ang=real(ang);
                %                 dist=a(1)*cos(ang);
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                angle=pi-c;
                dist=sin(angle)*a(1);
            end
            pioneer_set_controls(sp,100,0);
        end
    else
        a=pioneer_read_sonars();
        d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
        c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
        angle=pi-c;
        dist=sin(angle)*a(1);
        if a(1)>4500 || a(2)>4500
             while dist>400
                pioneer_set_controls(sp,100,round(5));
                pause(2)
                pioneer_set_controls(sp,100,round(-5));
                pause(1)
                pioneer_set_controls(sp,100,0);
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                angle=pi-c;
                dist=sin(angle)*a(1);
             end
             while abs(c-0.8727)>0.1
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
             end 
             fprintf('fim endireitar dps da curvaaaaaaaa\n')
             pioneer_set_controls(sp,100,0);
            flag=true;
        end
    end
end
a(7)
fprintf('saiu')
pioneer_set_controls(sp,0,0)
