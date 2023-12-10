clc;
clear;

n=3:20;
A = 0.25*n.*cot(pi./n);
P = n;
k = P./sqrt(A);
plot(n,k,'Linewidth',2)
yline(2*pi/sqrt(pi), 'k--')
xlabel('n: No. of sides of polygon')
ylabel('k parameter')
