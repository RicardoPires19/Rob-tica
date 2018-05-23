function [] = doCurve(sp, c)

a = pioneer_read_sonars();
t = 0;
a(1)
if(a(1) < 250)
    w = -11; t=2.5;
elseif(a(1) > 600)
    w = -13; t = 0.5;
else
    w = -14;
end

if abs(c-0.8727)>0.15
    w = w - 2;
if abs(c-0.8727)>0.05
    w = w - 1;
end    

pioneer_set_controls(sp, 250, w);

pause(6+t)

pioneer_set_controls(sp, 250, 0);

pause(0.5)

end