function [x,y] = Circulo(r)
theta = linspace(0,2*pi,200);
x = r*cos(theta);
y = r*sin(theta);
end