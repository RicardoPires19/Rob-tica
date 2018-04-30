%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);
a=pioneer_read_sonars();
while a(4)>500 && a(5)>500
    tic;
    a=pioneer_read_sonars();
    tStart=tic;
    points=[];
    iter=1;
    time=[];
    t=0;
    a(1)
    diag=280+a(1)+a(8);
    cor=1650;
    ang=acos(cor/diag);
    a(1)*cos(ang)
     a(1)*cos(ang)
     pause(0.2)
    while a(1)*cos(ang)>300 && a(1)*cos(ang)<700
        if toc(tStart)/7>=iter
             t=toc(tStart);
             a=pioneer_read_sonars();
             a(1);
             points=[points;a(1)*10^-3];
             time=[time;t];
             iter=iter+1;
             pause(0.2);
        end
       
        if length(points)>2 && (points(length(points))- points(length(points)-1))>0.15
            fprintf('break');
            break;
        end
    end
    time
    if length(time)>0
        [f, ~] = polyfit(time, points, 1);
        %angle = atan( f(1) + f(2) );
        %angle = rad2deg(angle);
        %v=angle/time(length(time));
        if( f(1) >= 0)
            angle = atan( (f(1) + f(2)) );
        else
            x = - ( f(2)/f(1) );
            y = f(2);
            h = sqrt( x^2 + y^2 );
            angle = acos( x / h );
        end

    end
    tStart=tic;
    pioneer_set_controls(sp,100,-round(v));

    if toc(tStart)>0.07
        pioneer_set_controls(sp,100,0);
        fprintf('sidfniasou');
    end
end
pioneer_set_controls(sp,0,0)