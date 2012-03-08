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

ss = 4.3;
plot3(X,-Y,Z,'.','LineWidth',0.1,'MarkerSize',ss)
hold on
plot3(xTraj,-yTraj,zTraj,'ro-','LineWidth',5)
axis([-0.4 0.4 -0.4 0.4  -0.27 -0.15])
grid on

labelSize   =   14;
titleSize   =   16;

xlabel({'Z Position (meters)'},'FontSize', labelSize)
ylabel({'X Position (meters)'},'FontSize', labelSize)
zlabel({'Y Position (meters)'},'FontSize', labelSize)
title({'Sparse Reachable Map Cross Section for Right Arm'; 'with setup phase and velocity trajectory L_d'},'FontSize',titleSize)


%% setup traj
nn      =   21;        %% index to stop at
xs      =   x{1};
xs      =   xs(1:nn);
ys      =   -y{1};
ys      =   ys(1:nn);
zstart  =   -0.2;
zs      =   zstart:-(abs(zstart-zTraj(1)))/(nn-1):zTraj(1);

plot3(xs,-ys,zs,'g','LineWidth',4);


xyMag   =   sqrt(diff(xTraj).^2+diff(yTraj).^2);

xDist   = atand(diff(yTraj)/diff(xTraj));

legend('Sparse Reachable Map','Velocity Phase L_d', 'Setup Phase' ,'FontSize', labelSize)
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
disp('resize then press ENTER')
pause();

%savefig('fig4p8.pdf','pdf')
tname = 'throwTraj3D.pdf';
saveas(gca,tname);
system(['pdfcrop ',tname,' ',tname]);

