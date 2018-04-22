function [angle] = calculate_angle(SampleX, SampleY)

[f, ~] = polyfit(SampleX, SampleY, 1);

angle = atan( f(1) + f(2) );

angle = rad2deg(angle);

end