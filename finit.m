a = pioneer_read_sonars();
t = 0;

if(a(1) < 250)
    w = 13;
elseif(a(1) > 600)
    w = 13; t = 1;
else
    w = 14;
end
    
pioneer_set_controls(sp, 250, w);

pause(6)

pioneer_set_controls(sp, 250, 0);

pause(8+t)

pioneer_set_controls(sp, 250, -14);

pause(6)

pioneer_set_controls(sp, 250, 0);

pause(2)

pioneer_set_controls(sp, 0, 0);