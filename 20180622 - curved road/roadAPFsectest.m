y = 7:-0.1:-3.5;
eta = 0.9;
k=1;
z =  eta./(y-7).^2 +...
     eta./(y+3.5).^2+...
     k*exp(-(y-3.5).^2./(2*eta^2))+...
     k*exp(-(y).^2./(2*eta^2));
 plot(y,z)
 hold on 
