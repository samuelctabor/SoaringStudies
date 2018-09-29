close all;

thermalState = [2.79,-29.0,8.58,-20.3];

x=readGdbArray('actionPath_x.txt');
y=readGdbArray('actionPath_y.txt');
th=readGdbArray('actionPath_theta.txt');

x(all(x'==0),:)=[];
y(all(y'==0),:)=[];
th(all(th'==0),:)=[];

x(:,all(x==0))=[];
y(:,all(y==0))=[];
th(:,all(th==0))=[];

th=th(:,1:size(x,2));

action = linspace(45,15,5);

leg=arrayfun(@num2str,action,'UniformOutput',false);

figure,plot(x',y');

hold on;
plot(thermalState(3),thermalState(4),'ro')
legend(leg)
axis equal;

figure,plot(th')
legend(leg)


poly.A = -0.030993;
poly.B =  0.447319;
poly.C = -2.302930;


lift = thermalState(1) + exp(-((x-thermalState(3)).^2 + (y-thermalState(4)).^2)./thermalState(2)^2);
v0=9;
sink = (poly.A*v0^2 + poly.B * v0+poly.C)./cosd(th);

allCost = sink + lift;

t=1:size(lift,2);
figure;
subplot(2,2,1); plot(lift');
subplot(2,2,2); plot(sink');
subplot(2,2,3); plot(allCost');

cumCost = cumsum(allCost,2)
div = repmat(1:size(allCost,2),size(allCost,1),1);

subplot(2,2,4); plot(cumCost'./div');

legend(leg)

sum(allCost,2)
