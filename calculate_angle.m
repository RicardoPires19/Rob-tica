function [angle] = calculate_angle(SampleX, SampleY)

[f, ~] = polyfit(SampleX, SampleY, 1);

if( f(1) >= 0)
    angle = atan( (f(1) + f(2)) );
else
    x = - ( f(2)/f(1) );
    y = f(2);
    h = sqrt( x^2 + y^2 );
    angle = acos( x / h );
end


angle = rad2deg(angle);

end