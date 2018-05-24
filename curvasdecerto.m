%sp=serial_port_start('COM4');
%pioneer_init(sp);

% VELOCIDADE ANGULAR POSITIVA = ESQUERDA
% VELOCIDADE ANGULAR NEGATIVA = DIREITA

ROBOT_VELOCITY = 250;

pioneer_set_controls(sp,ROBOT_VELOCITY,0);
a=pioneer_read_sonars();
b=pioneer_read_odometry();
num_voltas=1;
last_odom=0;
% corredor 1
%value = 9500;
% corredor 2
value = 11000;
% corredor 3
% value = 10500;
% corredor 4
% value = ;
pos=1;
points=[];
volta_correcao=true;
tira_medidas=true;
ROBOT_VELOCITY_WHILE_FIXING = 250;
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
    while true
%         fprintf('ciclo true\n')
        b=pioneer_read_odometry();
        while abs(b(pos))<value-500
            a=pioneer_read_sonars();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            b=pioneer_read_odometry();
            
            if dist < 300 || dist > 450
                break;
            end
            fprintf('valor dist %f e angulo %f na iteracao %d\n',dist,ang, iter)
%             elseif length(points)>15
%                 if abs(mean(points)-a(1)*10^-3)<600*10^-3
%                     points=[points;a(1)*10^-3];    %%esta em metros certo?
%                     time=[time;t];
%                 end
%             else
%                 points=[points;a(1)*10^-3];    %%esta em metros certo?
%                 time=[time;t];
            iter=iter+1;
            pause(0.2)
           
            b=pioneer_read_odometry();
            
%             if length(points)>15
                
%                 if dist>450 && (points(length(points))>points(length(points)-1))
%                     fprintf('fora de limites \n')
%                     break;
%                 end
%                 if dist<310 && (points(length(points))<points(length(points)-1))
%                     fprintf('fora de limites \n')
%                     break;
%                 end
%             end
        end
        
        while abs(b(pos))>value-500
            a=pioneer_read_sonars();
            b=pioneer_read_odometry();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            while abs(c-0.8727)>0.02
                fprintf('inicio endireitar dentro dos limites %f\n',c)
                a=pioneer_read_sonars();
                d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                pause(0.2)
                if c-0.8727<-0.02
                    fprintf('endireitar para a esquerda\n')
                    pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -2);
                elseif c-0.8727>0.02
                    fprintf('endireitar para a DIREITA\n')
                     pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 2);
                end
                b=pioneer_read_odometry();
                if abs(b(pos))>value+700
                    break;
                end
            end
            if abs(b(pos))>value+500
                if a(4)<1400 || a(5)<1600
                    fprintf('CURVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n')
                    doCurve(sp, c)

                    b=pioneer_read_odometry();
                    if num_voltas==1
                        last_odom=b(2);
                        value=9500+abs(last_odom);
                        pos=2;
                    elseif num_voltas==2
                        last_odom=b(1);
                        value=11000-abs(last_odom);
                        pos=1;
                    elseif num_voltas==3
                        last_odom=b(2);
                        value=9500+abs(last_odom);
                        pos=2;
                    elseif num_voltas==4
                        fprintf('Ultima volta')
                        last_odom=b(1);
                        value=11000+abs(last_odom);
                        pos=1;
                    end
                    num_voltas=num_voltas+1;
                    tira_medidas=true;
                    a=pioneer_read_sonars();
                    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                    while abs(c-0.8727)>0.02
                        fprintf('inicio endireitar dentro dos limites %f\n',c)
                        a=pioneer_read_sonars();
                        d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                        c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                        pause(0.2)
                        if c-0.8727<-0.02
                            fprintf('endireitar para a esquerda\n')
                            pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -2);
                        elseif c-0.8727>0.02
                            fprintf('endireitar para a direita\n')
                            pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 2);
                        end
                        b=pioneer_read_odometry();
                    end
                end
            end
        end
        if dist < 300 || dist > 450
            fprintf('iniciar correcao')
            break;
        end
        
    end
%     fprintf('off')
%     if ~isempty(time)
%         [f, n] = polyfit(time, points, 1);
        %         angle = atan( (f(1) + f(2)) );
        %         angle = rad2deg(angle);
        %         v=angle/time(length(time));
    if (dist > 400)
        v=1;
    else
        v=-1;  %dps tirar redundante
    end


    while volta_correcao
        fprintf('dist antes ciclo %f \n', dist)
        tempo=tic;
        %             a=pioneer_read_sonars();
        %             b=pioneer_read_odometry();
        while dist<380 || dist>400
            volta_correcao=false;
            b=pioneer_read_odometry();
            pause(0.2)
            if abs(b(pos))>value-500
                fprintf('corrigir proximo da curva\n')
%                 tira_medidas=false;
                break;
            end
            fprintf('dist %d \n', dist)
            a=pioneer_read_sonars();
            pause(0.2)
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            if dist<380
                v=-abs(v);
                pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,-abs(v));
            else
                v=abs(v);
                pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,abs(v));
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
            if abs(b(pos))>value-500 
                fprintf('curva durante o endireitar')
%                 tira_medidas=false;
                break;

            end

            fprintf('fim endireitar angulo %3.3f \n', c)
            b=pioneer_read_odometry();
            a=pioneer_read_sonars();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            if dist<300
                fprintf('endireitar demasiado proximo parede esquerda')
                pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -6);
                pause(0.35);
                volta_correcao=true;
                break;
            elseif dist>480
                fprintf('endireitar demasiado proximo parede direita')
                pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 6);
                pause(0.35);
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
% end
% a(7)
% fprintf('saiu')
% pioneer_set_controls(sp,0,0)