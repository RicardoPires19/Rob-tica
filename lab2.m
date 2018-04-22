%sp=serial_port_start('COM4');
%pioneer_init(sp);
%pioneer_set_controls(sp,100,0.00001);
pioneer_set_controls(sp,100,0);

tic;
tStart=tic;
points=[];
iter=1;
time=[];
while t<10
    if toc(tStart)/2.5>=iter
        t=toc(tStart)
        a=pioneer_rad_sonars();
        points=[points;a(8)];
        time=[time;t]
        iter=iter+1;
    end
end
fuc=polyval(time,points,1);
w=polyder(p)
