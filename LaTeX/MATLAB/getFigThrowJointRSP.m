close all
clear all

%% set time step
T = 0.01;      

%% open figure
open('throwTraj.fig');        

%% get handle
h = gcf;

%% get data and axis objects
axesObjs = get(h, 'Children');
dataObjs = get(axesObjs, 'Children'); 

%% get data object types
objTypes = get(dataObjs, 'Type');

%% attempt 2

x = get(get(gca,'Children'),'XData');

y = get(get(gca,'Children'),'YData');
figure

%% trajectory
xTraj   =   x{3};
xmin    =   min(xTraj);
xmax    =   max(xTraj);
gD      =   3.8;            % goal degree
yTraj   =   -y{3};
zmin    =   -0.218;
zmax    =   abs(xmin-xmax)*tand(gD)+zmin;
zTraj   =   zmin:abs(zmin-zmax)/(length(xTraj)-1):zmax*ones(length(xTraj),1);

%% reachaible area
n       =   6000;    % number of points to use
X       =   x{4};
Y       =   -y{4};
X       =   X(1:n);
Y       =   Y(1:n);
Z       =   rand(n,1)*0.04+-0.23;
plot3(X,Y,Z,'.','LineWidth',0.1)
hold on
plot3(xTraj,yTraj,zTraj,'ro-','LineWidth',5)
axis([-0.4 0.4 -0.4 0.4  -0.27 -0.15])
grid on

labelSize   =   14;
titleSize   =   16;

xlabel('X Position (meters)','FontSize', labelSize)
ylabel('Y Position (meters)','FontSize', labelSize)
zlabel('Z Position (meters)','FontSize', labelSize)
title({'Sparse Reachable Map Cross Section for Right Arm'; 'with setup phase and velocity trajectory L_d'},'FontSize',titleSize)



%% setup traj
nn      =   21;        %% index to stop at
Xs      =   x{1};
xs      =   Xs(1:nn);
Ys      =   -y{1};
ys      =   Ys(1:nn);
zstart  =   -0.2;
zs      =   zstart:-(abs(zstart-zTraj(1)))/(nn-1):zTraj(1);

plot3(xs,ys,zs,'g','LineWidth',4);
legend('Sparse Reachable Map','Velosity Phase L_d', 'Setup Phase' ,'FontSize', labelSize)



% % % % % %% Actuial full trajectory;
% % % % % ns = 21;
% % % % % ne = 155;
% % % % % ra = ns:floor((ne-ns)/(length(zTraj)-1)):ne
% % % % % % xact        =   [xs+rand(1,length(xs))*0.0001, Xs(nnn:2:((nnn+length(zTraj)*2))-1)];
% % % % % % yact        =   [ys+rand(1,length(xs))*0.0001, Ys(nnn:2:((nnn+length(zTraj)*2))-1)];
% % % % % xact        =   [xs+rand(1,length(xs))*0.0001, Xs(ra)];
% % % % % yact        =   [ys+rand(1,length(xs))*0.0001, Ys(ra)];
% % % % % zact        =   [zs, zTraj];

%% Actuial full trajectory;
ns = 21;
ne = 155;
ra = ns:ne;
% xact        =   [xs+rand(1,length(xs))*0.0001, Xs(nnn:2:((nnn+length(zTraj)*2))-1)];
% yact        =   [ys+rand(1,length(xs))*0.0001, Ys(nnn:2:((nnn+length(zTraj)*2))-1)];



xact        =   [xs+rand(1,length(xs))*0.0001, Xs(ra)];
yact        =   [ys+rand(1,length(xs))*0.0001, Ys(ra)];
zact        =   [zs, ones(1,length(Ys(ra)))*zs(length(zs))];

xa  =   xact;
ya  =   yact;
za  =   zact;
%% desired full trajectory
ts          =   1;
te          =   length(xTraj);
rra         =   0:(length(ra)-1);
rax         =   (xTraj(te)-xTraj(ts))*rra/length(rra);
ray         =   (yTraj(te)-yTraj(ts))*rra/length(rra);
raz         =   (zTraj(te)-zTraj(ts))*rra/length(rra);
xf          =   xTraj(te)/xTraj(ts)*rax + xTraj(ts);
yf          =   yTraj(te)/yTraj(ts)*ray + yTraj(ts);
zf          =   zTraj(te)/xTraj(ts)*raz + zTraj(ts);
xdes        =   [xs, xf];
ydes        =   [ys, yf];
zdes        =   [zs, zf];
% xdes        =   [xs, xf];
% ydes        =   [ys, yf];
% zdes        =   [zs, zf];

size(xdes)
size(ydes)
size(zdes)

size(xact)
size(yact)
size(zact)

%% index to subtract
ls  =   1;
lm  =   length(xs);
le  =   length(xdes);

xd  =   xdes;
xa  =   xact;
yd  =   ydes;
ya  =   yact;
zd  =   zdes;
za  =   zact;

ex1 = xd(ls:lm)-xa(ls:lm);
xf  =   xa;
yf  =   ya;
zf  =   zf;

for ( i = lm:le )
    d   = 0.03;
    xe  = abs(abs(xd(i))-abs(xa));
    ix  = find(xe == min(xe));
    ye  = abs(abs(yd(i))-abs(ya));
    iy  = find(ye == min (ye));
    
    %xit  = min(ix);
    %it  = find(xa > (xd(i)-d) & xa < (xd(i)+d));
    %ii  = min(it);
    xf(i) = xa(min(ix));
    yf(i) = ya(min(iy));
    %zf(i) = za(ii);
end

figure
n = length(xa);
plot(xa(1:n),ya(1:n),'o')
hold on
%plot(xd(1:21),yd(1:21),'r+')
%plot(xTraj,yTraj,'r+')

xxt = [];
yyt = []
ii = 1;
tdiv = 2;
for ( i = 1:9 )
    for (j = 1:tdiv)
        xxt(ii) = xTraj(i)+(xTraj(i+1)-xTraj(i))*(j-1)/tdiv;
        ii = ii+1;  
    end
        
%     xxt(ii) = xTraj(i);
%     ii = ii+1;
%     xxt(ii) = xTraj(i)+(xTraj(i+1)-xTraj(i))/6;
%     ii = ii+1;
%     xxt(ii) = xTraj(i)+(xTraj(i+1)-xTraj(i))*2/6;
%     ii = ii+1;
%     xxt(ii) = xTraj(i)+(xTraj(i+1)-xTraj(i))*3/6;
%     ii = ii+1;
%     xxt(ii) = xTraj(i)+(xTraj(i+1)-xTraj(i))*4/6;
%     ii = ii+1;
%     xxt(ii) = xTraj(i)+(xTraj(i+1)-xTraj(i))*5/6;
%     ii = ii+1;
end
ii = 1;
for ( i = 1:9 )
    for (j = 1:tdiv)
        yyt(ii) = yTraj(i)+(yTraj(i+1)-yTraj(i))*(j-1)/tdiv;
        ii = ii+1;
    end
%     yyt(ii) = yTraj(i);
%     ii = ii+1;
%     yyt(ii) = yTraj(i)+(yTraj(i+1)-yTraj(i))/6;
%     ii = ii+1;
%     yyt(ii) = yTraj(i)+(yTraj(i+1)-yTraj(i))*2/6;
%     ii = ii+1;
%     yyt(ii) = yTraj(i)+(yTraj(i+1)-yTraj(i))*3/6;
%     ii = ii+1;
%     yyt(ii) = yTraj(i)+(yTraj(i+1)-yTraj(i))*4/6;
%     ii = ii+1;
%     yyt(ii) = yTraj(i)+(yTraj(i+1)-yTraj(i))*5/6;
%     ii = ii+1;
end

%% these are the good desired trajectories
xxd = [xd(1:21), xxt];
yyd = [yd(1:21), yyt];

figure
plot(xxd, yyd, 'r+')
hold on
xf = [];
yf = [];
% use xa and ya
for( i = 1:length(xxd))
   ex = abs(xxd(i)-xa);
   ix = find(ex == min(ex));
   ix = min(ix);
   xf(i) = xa(ix);
   yf(i) = ya(ix);
end
plot(xf,yf,'o')

e = sqrt((xxd-xf).^2+(yyd-yf).^2)
figure
plot(e)

a1 = 0.17914;
a2 = 0.18159;

rspA = [];
rebA = [];
rspD = [];
rebD = [];



for( i = 1:length(xxd))
   di = [ 1, 2 ];
   dL = [a1, a2]
   x = xxd(i);
   y = yyd(i);
   [d, di, dflag] = huboSpEb2dofIK(x,y,di, dL);
   rspD(i) = d(1);
   rebD(i) = d(2);
   
   x = xf(i);
   y = yf(i);
   [d, di, dflag] = huboSpEb2dofIK(x,y,di, dL);
   rspA(i) = d(1);
   rebA(i) = d(2);
   
end
labelSize   =   14;
titleSize   =   16
te = 0.31; %% end time in seconds
T   = te/length(rspA);
t   = 0:T:(te-T); 
figure
subplot(3,1,1)
plot(t,rspA)
hold on
plot(t,rspD,'r')
ylabel('Position (rad)','FontSize', labelSize)
legend('Commanded', 'Measured','FontSize', labelSize)
title({'Right Shoulder Pitch Commanded and Measured', 'Motion Profile'},'FontSize',titleSize)
subplot(3,1,2)
dD = diff(rspD)/T;
plot(t(2:length(t)),dD)
hold on
dA = diff(rspA)/T;
dA(1) = dA(2);
plot(t(2:length(t)),dA,'r')
ylabel('Velocity (rad/sec)','FontSize', labelSize)
subplot(3,1,3)
ddD = diff(dD)/T;
plot(t(3:length(t)),ddD)
hold on
ddA = diff(dA)/T;
plot(t(3:length(t)),ddA,'r')
ylabel('Acceleration (rad/sec^2)','FontSize', labelSize)
xlabel('Time (sec)','FontSize', labelSize)


%% for position space
%  xD = xxd(22:length(xxd));
%  yD = yyd(22:length(xxd));
%  xA = xf(22:length(xxd));
%  yA = yf(22:length(xxd));

ra = [22:length(xxd)];
%ra = 1:length(xxd);
 xD = xxd(ra);
 yD = yyd(ra);
 xA = xf(ra);
 yA = yf(ra);
 
 T2 = T*0.6306;
 T2 = T*4.314/5*1.02;
 sD = (abs(diff(xD))+abs(diff(yD)))/T2;
 sA = (abs(diff(xA))+abs(diff(yA)))/T2;

 
labelSize   =   14;
titleSize   =   16
te = 0.31; %% end time in seconds
ts = 0.2;
T   = (te-ts)/length(sD);
t   = ts:T:(te-T); 
figure
subplot(2,1,1)
plot(t,sD)
hold on
plot(t,sA,'r')
axis([ ts, te -4, 12])
ylabel('Velocity (m/s)','FontSize', labelSize)
legend('Commanded', 'Measured','FontSize', labelSize)
title({'Velocity and Acceleration of End Effector During', 'Motion Profile'},'FontSize',titleSize)
subplot(2,1,2)
dD = diff(sD)/T2;
plot(t(2:length(t)),dD)
axis([ts,te, -1200, 1200])
hold on
dA = diff(sA)/T2;
dA(1) = dA(2);
plot(t(2:length(t)),dA,'r')
ylabel('Acceleration (m/s^2)','FontSize', labelSize)
xlabel('Time (sec)','FontSize', labelSize)






% figure
% plot(t,rebA)
% hold on
% plot(t,rebD,'r')


% % % % % 
% % % % % figure
% % % % % n = length(xxd);
% % % % % plot(xxd,yyd,'r+')
% % % % % hold on
% % % % % m = length(xa);
% % % % % plot(xa(n:m),ya(n:m),'o')
% % % % % title('test')
% % % % % 
% % % % % xaf = [];
% % % % % xaf = [xxd(1:21)+rand(1,21)*0.001 ];
% % % % % yaf = [yyd(1:21)+rand(1,21)*0.001 ];
% % % % % 
% % % % % figure
% % % % % plot(xxd,yyd,'o')
% % % % % hold on
% % % % % plot(xaf,yaf,'r+')
% % % % % ss = 21;
% % % % % ee = 56;
% % % % % plot(xa(ss:ee), ya(ss:ee), 'g+')
% % % % % title('test2')

%e = sqrt((xdes-xact).^2 + (ydes-yact).^2 + (zdes-zact).^2);
%e2 = xdes - xact;

%e = sqrt((xdes-xf).^2 + (ydes-yf).^2 + (zdes-zf).^2);

%e = ydes-yf;
%figure
%plot(e)
%figure 
%plot(e2)




xyMag   =   sqrt(diff(xTraj).^2+diff(yTraj).^2);

xDist   = atand(diff(yTraj)/diff(xTraj));


% % % 
% % % 
% % % xGood   = x{1};
% % % yGood   = -y{1};
% % % 
% % % xAll    = x{2};
% % % yAll    = -y{2};
% % % 
% % % ss      =   4.8;        % dot size
% % % %plot(xAll,yAll,'r.', 'MarkerSize',ss);
% % % %hold on
% % % plot(xGood,yGood,'.', 'MarkerSize',ss)
% % % axis([-0.4 0.4 -0.4 0.4])
% % % 
% % % labelSize   =   14;
% % % xlabel('X Position (Meters)','FontSize', labelSize)
% % % ylabel('Z Position (Meters)','FontSize', labelSize)
% % % 
% % % titleSize   =   16;
% % % 
% % % legend('Reachable Points')
% % % %title('Sparse Reachable Region for Right Arm (Y=-0.21m to -0.22m)','FontSize',titleSize)
% % % title('Sparse Reachable Map Cross Section for Right Arm (Y=-0.21m to -0.22m)','FontSize',titleSize)
% disp('resize then press ENTER')
% pause();
% 
% %savefig('fig4p8.pdf','pdf')
% tname = 'throwTrajRSPacc.pdf';
% saveas(gca,tname);
% system(['pdfcrop ',tname,' ',tname]);

