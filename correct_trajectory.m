function [angle1, angle2, time] = correct_trajectory(t, y, d, v)
% x - pontos do sonar
% time - intervalo de tempo entre cada ponto de x
% d - última distância entre o robot e a parede

angle2 = -round(calculate_angle(t, y));

angle1 = 2 * -angle2;

end