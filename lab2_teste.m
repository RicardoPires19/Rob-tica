%sp=serial_port_start('COM4');
%pioneer_init(sp);

cor = 1650;
iter = 1;
a = pioneer_read_sonars();
pioneer_set_controls(sp,100,0);


while true
    [tStart, points, time, t, diag, ang] = init_cycle();
    pause(0.2);
    
    if(a(4) > 600 || a(5) > 600)
        [iter, points, time] = move_forward(a, tStart, ang, iter, points, time, sp);
    elseif(a(1) > 1000)
        rotate_left(sp);
    elseif(a(8) > 1000)
        rotate_right(sp);
    else
        pioneer_set_controls(sp,0,0);
    end
        
end