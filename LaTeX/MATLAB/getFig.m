close all
clear all


%% open figure
open('Right Arm Reachable Points 2DOF.fig');        

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


xGood   = x{1};
yGood   = -y{1};

xAll    = x{2};
yAll    = -y{2};

ss      =   4.8;        % dot size
%plot(xAll,yAll,'r.', 'MarkerSize',ss);
%hold on
plot(xGood,yGood,'.', 'MarkerSize',ss)
axis([-0.4 0.4 -0.4 0.4])

labelSize   =   14;
xlabel('X Position (Meters)','FontSize', labelSize)
ylabel('Z Position (Meters)','FontSize', labelSize)

titleSize   =   16;

legend('Reachable Points')
%title('Sparse Reachable Region for Right Arm (Y=-0.21m to -0.22m)','FontSize',titleSize)
title('Sparse Reachable Map Cross Section for Right Arm (Y=-0.21m to -0.22m)','FontSize',titleSize)
disp('resize then press ENTER')
pause();

%savefig('fig4p8.pdf','pdf')
tname = 'reachable2DofR4p8.pdf';
saveas(gca,tname);
system(['pdfcrop ',tname,' ',tname]);

