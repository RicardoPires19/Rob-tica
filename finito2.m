function [] = finito2(sp, c)

a = pioneer_read_sonars();
t = 0;
a(1)

if(a(1) < 250)
    w = 12; t=0.25;
% elseif(a(1) > 750)
%     w = -14;
else
    w = 15;
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
end
