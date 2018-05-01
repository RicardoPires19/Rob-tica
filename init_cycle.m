function [tStart, points, iter, time, t, diag, ang] = init_cycle()

tic;
a=pioneer_read_sonars();
tStart=tic;
points=[];
iter=1;
time=[];
t=0;
a(1)
diag=280+a(1)+a(8);
   
ang=acos(cor/diag);
a(1)*cos(ang)

end