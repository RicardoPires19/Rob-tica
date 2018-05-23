%sp=serial_port_start('COM4');
%pioneer_init(sp);

ROBOT_VELOCITY = 200;

pioneer_set_controls(sp,ROBOT_VELOCITY,0);
a=pioneer_read_sonars();
b=pioneer_read_odometry();
num_voltas=1;
last_odom=0;
value=11500;
pos=1;
points=[];
volta_correcao=true;
tira_medidas=true;
ROBOT_VELOCITY_WHILE_FIXING = 200;
while a(4)>200
    fprintf('begin\n')
    b=pioneer_read_odometry();
    points=[];
    iter=1;
    time=[];
    t=0;
    
    a=pioneer_read_sonars();
    pause(0.1);
    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
    ang=pi-c;
    dist=sin(ang)*a(1);
    fprintf('nova distancia %d \n',dist)
    
    fprintf('fora ciclo com angulo %f var dist esta a %f \n', ang, dist)
    tStart=tic;
    while true
        if tira_medidas
            t=toc(tStart);
            a=pioneer_read_sonars();
            if length(points)>15
                if abs(mean(points)-a(1)*10^-3)<600*10^-3
                    points=[points;a(1)*10^-3];    %%esta em metros certo?
                    time=[time;t];
                end
            else
                points=[points;a(1)*10^-3];    %%esta em metros certo?
                time=[time;t];
            end
            iter=iter+1;
            pause(0.2)
            a=pioneer_read_sonars();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            b=pioneer_read_odometry();
            
            if length(points)>15
                if abs(b(pos))>value
                    fprintf('ultima correcao \n')
                    break;
                end
                if dist>450 && (points(length(points))>points(length(points)-1))
                    fprintf('fora de limites \n')
                    break;
                end
                if dist<280 && (points(length(points))<points(length(points)-1))
                    fprintf('fora de limites \n')
                    break;
                end
            end
        end
            while abs(b(pos))>value
                a=pioneer_read_sonars();
                b=pioneer_read_odometry();
                
                if abs(b(pos))>value+500
                    if a(4)<1400 || a(5)<1600
                        fprintf('CURVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n')
                        doCurve(sp)
%                         points=[];
%                         time=[];
%                         t=0;
                        b=pioneer_read_odometry();
                        if num_voltas==1
                            last_odom=b(2);
                            value=12000+abs(last_odom);
                            pos=2;
                        end
                        if num_voltas==2
                            last_odom=b(1);
                            value=12000-abs(last_odom);
                            pos=1;
                        end
                        if num_voltas==3
                            last_odom=b(2);
                            value=10000+abs(last_odom);
                            pos=2;
                        end
                        
                        if num_voltas==4
                            fprintf('Ultima volta')
                            last_odom=b(1);
                            value=11000+abs(last_odom);
                            pos=1;
                        end
                        num_voltas=num_voltas+1;
                    end
                    tira_medidas=true;
                    a=pioneer_read_sonars();
                    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                    while abs(c-0.8727)>0.02
                        fprintf('inicio endireitar dentro dos limites\n')
                        a=pioneer_read_sonars();
                        d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                        c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                        pause(0.2)
                        pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,2);
                        b=pioneer_read_odometry();
                        if abs(b(pos))>value+200
                            break;
                        end
                    end
                end
            end
            fprintf('valor dist %f e angulo %f na iteracao %d\n',dist,ang, iter)
    end
    fprintf('off')
    if ~isempty(time)
        [f, n] = polyfit(time, points, 1);
        angle = atan( (f(1) + f(2)) );
        angle = rad2deg(angle);
        v=angle/time(length(time));
        if ( f(1) > 0)
            v=1;
        else
            v=-1;  %dps tirar redundante
        end
        t=time(length(time));

        while volta_correcao
            fprintf('dist antes ciclo %f \n', dist)
            tempo=tic;
%             a=pioneer_read_sonars();
%             b=pioneer_read_odometry();
            while dist<400 || dist>430
                volta_correcao=false;
                b=pioneer_read_odometry();
                pause(0.2)
                if abs(b(pos))>value-500
                    tira_medidas=false;
                    break;
                end
                fprintf('dist %d \n', dist)
                a=pioneer_read_sonars();
                pause(0.2)
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                angle=pi-c;
                dist=sin(angle)*a(1);
                if dist<400
                    if dist<300
                        pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,-abs(v));
                    else
                        pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,-abs(v));    
                    end
                else
                    if dist>500
                        pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,abs(v));
                    else
                        pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,ceil(abs(v)));
                    end
                end
                fprintf('FIM DA CORREÇÃO: [ANGLE] %f  [DISTANCE] %f\n',ang, dist)
                stop=tic;
            end
            max=toc(tempo);
            a=pioneer_read_sonars();
            b=pioneer_read_odometry();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            tempo=tic;
            fprintf('SENSORS\n [A(1)] %f\n [A(2)] %f\n [C] %f\n', a(1),a(2),c)
            while (a(1)>4500 && a(2)>4500) || abs(c-0.8727)>0.02
                if v<0
                    pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING, abs(v));                        
                else
                    pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING, -abs(v));
                end
                fprintf('distancia %d endireitar para centro \n ', dist)
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                angle=pi-c;
                dist=sin(angle)*a(1);

                if toc(tempo)>max
                    fprintf('demasiado tempo a endireitar')
                    break;
                end
                b=pioneer_read_odometry();
                if abs(b(pos))>value-1000 && abs(c-0.8727)>0.1
                    fprintf('curva durante o endireitar')
                    tira_medidas=false;
                    break;
                    
                end

                fprintf('fim endireitar angulo %f \n', c)
                b=pioneer_read_odometry();
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                angle=pi-c;
                dist=sin(angle)*a(1);
                if dist<250
                    fprintf('endireitar demasiado proximo parede esquerda')
                    pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -6);
                    pause(0.25);
                    volta_correcao=true;
                    break;
                elseif dist>480
                    fprintf('endireitar demasiado proximo parede direita')
                    pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 6);
                    pause(0.25);
                    volta_correcao=true;
                    break;
                end
            end
        end
       pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,0);
       volta_correcao=true;
    end
%     else
%         a=pioneer_read_sonars();
%         pause(0.1)
%         d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
%         c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
%         angle=pi-c;
%         dist=sin(angle)*a(1);
%         fprintf('dist %f dps da curva \n',dist);
%         while dist>500
%             pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,-1);
%             pause(0.2)
%             a=pioneer_read_sonars();
%             d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
%             c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
%             angle=pi-c;
%             dist=sin(angle)*a(1);
%             fprintf('distancia ao endireitar dps da curva %f\n', dist)
%         end
%          while abs(c-0.8727)>0.02
%             fprintf('inicio endireitar dentro dos limites\n')
%             a=pioneer_read_sonars();
%             d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
%             c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
%             pause(0.2)
%             pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,1);
%          end
%          fprintf('fim endireitar dps da curvaaaaaaaa\n')
%          pioneer_set_controls(sp,ROBOT_VELOCITY,0);
%         flag=true;
%     end
end
a(7)
fprintf('saiu')
pioneer_set_controls(sp,0,0)