function [] = doCurve(sp, c)

a = pioneer_read_sonars();
t = 0;
a(1)

if(a(1) < 250)
    w = -12; t=0.25;
% elseif(a(1) > 750)
%     w = -14;
else
    w = -14;
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
fprintf('c = %f | conta = %f a(1) = %f\n', c, c-0.8727, a(1))
fprintf('w = %f\n', w);

pioneer_set_controls(sp, 250, w);
pause(6.25+t)
% tic=start
% 
% while true
%     a=pioneer_read_sonars();
%     fprintf('sensores na curva %f %f \n',a(7),a(8) );
% 
% end
pioneer_set_controls(sp, 250, 0);

pause(0.5)

end