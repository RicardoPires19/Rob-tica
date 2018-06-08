%sp=serial_port_start('COM4');
%pioneer_init(sp);

% VELOCIDADE ANGULAR POSITIVA = ESQUERDA
% VELOCIDADE ANGULAR NEGATIVA = DIREITA

ROBOT_VELOCITY = 250;

pioneer_set_controls(sp,ROBOT_VELOCITY,0);
b=pioneer_read_odometry();
num_voltas=1;
last_odom=0;
RIGHT_LIMIT = 400;
RIGHT_LIMIT_OUTER = 480;
RIGHT_LIMIT_MIDDLE = 450;
LEFT_LIMIT = 380;
LEFT_LIMIT_OUTER = 300;
LEFT_LIMIT_EXTREME = 150;

value = 11000;

TEMPO_DE_CORREDOR = 46;
pos=1;
points=[];
volta_correcao=true;
tira_medidas=true;
ROBOT_VELOCITY_WHILE_FIXING = 250;

pioneer_set_controls(sp, 250, 0);
pause(4)
pioneer_set_controls(sp, 250, 16);
pause(5.35)
pioneer_set_controls(sp, 250, 0);
pause(14)

a=pioneer_read_sonars();
pause(0.1);
dist = a(1);

LEFT_LIMIT = dist - 10;
RIGHT_LIMIT = dist + 10;
LEFT_LIMIT_OUTER = LEFT_LIMIT - 80;
RIGHT_LIMIT_OUTER = RIGHT_LIMIT + 80;
RIGHT_LIMIT_MIDDLE = RIGHT_LIMIT + 50;

fprintf('a(1) = %f | LEFT = %f | RIGHT = %f\n', a(1), LEFT_LIMIT, RIGHT_LIMIT)

tic;
while true
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
%         while abs(b(pos))<value-500
        if toc < TEMPO_DE_CORREDOR - 2
            a=pioneer_read_sonars();
            d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
            c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
            angle=pi-c;
            dist=sin(angle)*a(1);
            b=pioneer_read_odometry();
            
            if dist < LEFT_LIMIT_OUTER || dist > RIGHT_LIMIT_MIDDLE
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
        if num_voltas>=4 && toc > TEMPO_DE_CORREDOR-2
            break;
        end
%         while abs(b(pos))>value-500
        while toc > TEMPO_DE_CORREDOR - 2
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
                a=pioneer_read_sonars();
                fprintf('sensors frente %f %f\n',a(4),a(5));
                if a(4)>1400 || a(5)>1400
                    break;
                end
            end
            
            if toc > TEMPO_DE_CORREDOR + 1
                if a(4)<1500 || a(5)<1600
                    fprintf('CURVAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n')
                    fprintf('toc = %f', toc)
                    doCurve(sp, c, num_voltas)
                    tic;
                    num_voltas=num_voltas+1;
                    b=pioneer_read_odometry();
                    if num_voltas==2
                        fprintf('Antigos limites restablecidos\n')
                        RIGHT_LIMIT = 410;
                        RIGHT_LIMIT_OUTER = 480;
                        RIGHT_LIMIT_MIDDLE = 430;
                        LEFT_LIMIT = 390;
                        LEFT_LIMIT_OUTER = 370;
                    elseif num_voltas==3
                        RIGHT_LIMIT = 450;
                        RIGHT_LIMIT_MIDDLE = 470;
                        RIGHT_LIMIT_OUTER = 700;
                        fprintf('Num_voltas = 3\n')
                    elseif num_voltas>=-3
                        fprintf('Ultima volta!\n')
                    end
                    fprintf('NUM_voltas = %d\n', num_voltas)
                    tira_medidas=true;
                    a=pioneer_read_sonars();
                    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                    while abs(c-0.8727)>0.05
                        fprintf('inicio endireitar dentro dos limites %f\n',c)
                        a=pioneer_read_sonars();
                        d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
                        c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
                        pause(0.2)
                        if c-0.8727<-0.05
                            fprintf('endireitar para a esquerda\n')
                            pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -2);
                        elseif c-0.8727>0.05
                            fprintf('endireitar para a direita\n')
                            pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 2);
                        end
                        if a(1) > LEFT_LIMIT_OUTER
                            fprintf('finito de endireitar dps da curva')
                            break;
                        end
                        b=pioneer_read_odometry();
                    end
                end
            end
        end
        
        if dist < LEFT_LIMIT_OUTER || dist > RIGHT_LIMIT_MIDDLE
            fprintf('iniciar correcao')
            break;
        end
        
    end
    if num_voltas==4 && toc > TEMPO_DE_CORREDOR-2
        break;
    end
%     fprintf('off')
%     if ~isempty(time)
%         [f, n] = polyfit(time, points, 1);
        %         angle = atan( (f(1) + f(2)) );
        %         angle = rad2deg(angle);
        %         v=angle/time(length(time));
    if (dist > RIGHT_LIMIT)
        v=1;
    else
        v=-1;  %dps tirar redundante
    end


    while volta_correcao
        fprintf('dist antes ciclo %f \n', dist)
        tempo=tic;
        %             a=pioneer_read_sonars();
        %             b=pioneer_read_odometry();
        while dist < LEFT_LIMIT || dist > RIGHT_LIMIT
            volta_correcao=false;
            b=pioneer_read_odometry();
            pause(0.2)
%             if abs(b(pos))>value-500
            if toc > TEMPO_DE_CORREDOR - 2
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
            if dist < RIGHT_LIMIT
                v=-abs(v);
                pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,v);
            else
                v=abs(v);
                pioneer_set_controls(sp,ROBOT_VELOCITY_WHILE_FIXING,v);
            end
            if dist < LEFT_LIMIT_EXTREME
                v=-abs(v);
                pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -6);
                pause(0.35);
                pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 0);
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
%             if abs(b(pos))>value-500 
            if toc > TEMPO_DE_CORREDOR - 2
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
            if dist < LEFT_LIMIT_OUTER
                fprintf('endireitar demasiado proximo parede esquerda')
                pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, -6);
                pause(0.35);
                volta_correcao=true;
                break;
            elseif dist > RIGHT_LIMIT_OUTER
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

fprintf('SAIUUUUUUUUUUUUUUUUUUU\n')
a=pioneer_read_sonars();
v=-5;
d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
last_measure=50000
count=0
while abs(c-0.8727)>0.02
    fprintf('inicio endireitar dentro dos limites %f\n',c)
    a=pioneer_read_sonars();
    d=a(1)^2+a(2)^2-2*a(1)*a(2)*cos(0.7);
    c=(a(2)^2+a(1)^2-d)/(2*sqrt(d)*a(1));
    pause(0.2)
    fprintf('SENSORES %f %f \n',a(1), a(2))
    if a(1)>4500 && a(2)>4500
        fprintf('endireitar para a esquerda\n')
        pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 2);
    else
        if c-0.8727<-0.02
            fprintf('endireitar para a direta\n')
            pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING,- 2);
        elseif c-0.8727>0.02
            fprintf('endireitar para a esquerda\n')
             pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING, 2);
        end
        if a(4)<1400 && a(5)<1600
            break;
        end
    end
    a=pioneer_read_sonars();
    fprintf('%f %f \n',a(4),a(5))
    if toc > TEMPO_DE_CORREDOR + 2.5
        if a(4)<1600 || a(5)<1600 
            break;
        end
    end

end

pioneer_set_controls(sp, ROBOT_VELOCITY_WHILE_FIXING,0);
fprintf('quase %f',toc)
while true
    if toc > TEMPO_DE_CORREDOR + 2.5
        fprintf('entrouuuuuuuuuuuuuuuuuuu')
       a = pioneer_read_sonars();
        t = 0;
        a(1)

        if(a(1) < 250)
            w = 12; t=0.25;
        % elseif(a(1) > 750)
        %     w = -14;
        else
            w = 16;
        end

        if c-0.8727<-0.2
             t = -0.4;
        elseif c-0.8727<-0.05
             t = -0.2;
        elseif c-0.8727>0.2
             t = 0.4;
        elseif c-0.8727>0.05
             t = 0.2;
        end
        fprintf('c = %f | velocidade = %f a(1) = %f\n', c, w, a(1))
        fprintf('t = %f\n', t);

        pioneer_set_controls(sp, 250, w);

        pause(6+t)

        pioneer_set_controls(sp, 250, 0);

        pause(8+t)

        pioneer_set_controls(sp, 250, -14);

        pause(6)

        pioneer_set_controls(sp, 250, 0);

        pause(2)
        break;
    end
    
end
pioneer_set_controls(sp, 0, 0);
pioneer_close(sp);