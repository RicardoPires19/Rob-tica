%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);
a=pioneer_read_sonars();
while 1
    tic;
    tStart=tic;
    points=[];
    iter=1;
    time=[];
    t=0;
    a(1)
    diag=280+a(1)+a(8)
    cor=1650
    ang=acos(diag/cor)
    while a(1)*cos(ang)>300 && a(1)*cos(ang)<846
        if toc(tStart)/7>=iter
             t=toc(tStart)
             a=pioneer_read_sonars();
             a(1)
             points=[points;a(1)*10^-3];
             time=[time;t]
             iter=iter+1;
             pause(0.2)
        end
       
        if length(points)>2 && (points(length(points))- points(length(points)-1))>0.15
            fprintf('break');
            break;
        end
    end
    time
    if length(time)>0
        [f, ~] = polyfit(time, points, 1);
        angle = atan( f(1) + f(2) );
        angle = rad2deg(angle);
        v=angle/time(length(time));
    end
    tStart=tic;
    pioneer_set_controls(sp,100,-round(v));

    if toc(tStart)>0.1
        pioneer_set_controls(sp,100,0);
        fprintf('sidfniasou');
    end
end
