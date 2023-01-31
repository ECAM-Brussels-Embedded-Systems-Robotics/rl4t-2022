clear all
close all
Angle1 = -70:10:90;
Angle2 = -70:10:90;
[distance] = fopen('mapping.txt','r');
[A,counta] =fscanf(distance,'%f');
[angle] = fopen('angle.txt','r');
[B,countb] =fscanf(angle,'%f');
g = countb/2;
for ang=1:g 
    Angle1 = [Angle1,Angle2];
end
n=1;
while n<countb 
    B(n,1)=B(n,1)*90/1.5+90;
    B(n+1,1)=B(n+1,1)*3.4;
    n=n+2;
end

% for t=0:g
%     X = cosd(B(t,0)).*A(g*17:(g+1)*16);
%     Y = sind(B(t,0)).*A(g*17:(g+1)*16);
%     hold on;
%     plot(X,Y,'*');
% end
