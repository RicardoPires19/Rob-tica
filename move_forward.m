function [iter, points, time] = move_forward(a, tStart, ang, iter, points, time, sp)
    
% acho que aqui devia de ser  a(8)*cos(90 - ang)<700
    if(a(1)*cos(ang)>300 && a(1)*cos(ang)<700)
        if toc(tStart)/7>=iter
             t=toc(tStart);
             a=pioneer_read_sonars();
             a(1);
             points=[points;a(1)*10^-3];
             time=[time;t];
             iter=iter+1;
             pause(0.2);
        end
        return
    end
    
    %if length(points)>2 && (points(length(points))- points(length(points)-1))>0.15
    %    fprintf('break');
    %    break;
    %end
    iter = 1;
    time
    if ~isempty(time)
        angle = calculate_angle();
    end
    
    v = angle/time(length(time));

    pioneer_set_controls(sp, 100, -round(v));
    pause(v * angle)
    pioneer_set_controls(sp, 100, 0);
end