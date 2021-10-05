function [ x,y ] = plotCar(Car)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
l=Car.length/2;
w=Car.width/2;
phi=Car.State(3);
p1=[Car.State(1)+sqrt(l^2+w^2)*cos(atan(w/l)+phi) Car.State(2)+sqrt(l^2+w^2)*sin(atan(w/l)+phi)];
p2=[Car.State(1)+sqrt(l^2+w^2)*cos(pi-atan(w/l)+phi) Car.State(2)+sqrt(l^2+w^2)*sin(pi-atan(w/l)+phi)];
p3=[Car.State(1)+sqrt(l^2+w^2)*cos(pi+atan(w/l)+phi) Car.State(2)+sqrt(l^2+w^2)*sin(pi+atan(w/l)+phi)];
p4=[Car.State(1)+sqrt(l^2+w^2)*cos(2*pi-atan(w/l)+phi) Car.State(2)+sqrt(l^2+w^2)*sin(2*pi-atan(w/l)+phi)];
rect=[p1;p2;p3;p4;p1];
x=rect(:,1);
y=rect(:,2);
end

