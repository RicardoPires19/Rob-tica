%sp=serial_port_start('COM4');
%pioneer_init(sp);

pioneer_set_controls(sp,100,0);

tic;
tStart=tic;
points=[];
iter=1;
time=[];
t=0;
while t<70
    if toc(tStart)/4>=iter
         t=toc(tStart)
         a=pioneer_read_sonars();
         points=[points;a(1)*10^-3];
         time=[time;t]
         iter=iter+1;

    end
    if (points(length(points))- points(length(points)-1))>0.15
        break;
    end
end
[f, ~] = polyfit(time, points, 1);
angle = atan( f(1) + f(2) );
angle = rad2deg(angle);
v=angle/time(length(time));

tStart=tic;
pioneer_set_controls(sp,100,-round(v));

if toc(tStart)>1
    pioneer_set_controls(sp,100,0);
end

tic;
tStart=tic;
points=[];
iter=1;
time=[];
t=0;
while t<70
    if toc(tStart)/4>=iter
         t=toc(tStart)
         a=pioneer_read_sonars();
         points=[points;a(1)*10^-3];
         time=[time;t]
         iter=iter+1;

    end
    if length(points)>2
        if (points(length(points))- points(length(points)-1))>0.15
            break;
        end
    end
end
[f, ~] = polyfit(time, points, 1);
angle = atan( f(1) + f(2) );
angle = rad2deg(angle);
v=angle/time(length(time));
