%      Copyright 2020 Maram Khatib
%  
%      Licensed under the Apache License, Version 2.0 (the "License");
%      you may not use this file except in compliance with the License.
%      You may obtain a copy of the License at
%      http://www.apache.org/licenses/LICENSE-2.0
%  
%      Unless required by applicable law or agreed to in writing, software
%      distributed under the License is distributed on an "AS IS" BASIS,
%      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%      See the License for the specific language governing permissions and
%      limitations under the License.

clc
close all

controlPointsMinDistancfile=distanceControlPointsMin;
controlpointsMinfile=controlPointMin;
eeMinDistancefile=distanceEEMin;
positiondesiredfile=e0;
eefile=e0*0;
orien=e1;
qfile=QV;
qdotfile=dQV;
qddotfile=ddQV;

controlPointsMinDistancfile(controlPointsMinDistancfile == .11)=-1;
eeMinDistancefile(eeMinDistancefile == .11)=-1;
close all
dt=0.002;
t= 0 : dt :size(eefile,1)*dt-dt;

close all
figure
grid on 
xlabel('time [s]','FontSize',15)
ylabel('$D_{min}(\mathbf{p}_{c_{ee}})$ [m]','Interpreter','latex','FontSize',15)
ylim([0.0 0.1])
xlim([0.0 size(eefile,1)*dt-dt])
hold on
plot(t',eeMinDistancefile(:,1),'b.','LineWidth',2);
set(gca,'FontSize',15);
set(gcf,'Position',[355 420 560 200])
hold on 

figure
subplot(2,1,2)
plot(t',controlPointsMinDistancfile(:,1),'b.','LineWidth',1);
grid on 
ylabel('$D_{minimal}$ [m]','Interpreter','latex','FontSize',15)
xlabel('time [s]','FontSize',5)
ylim([0.0 0.15])
xlim([0.0 size(eefile,1)*dt-dt])
set(gca,'FontSize',15);
hold on

subplot(2,1,1)
plot(t',controlpointsMinfile(:,1),'g.','LineWidth',1);
grid on 
ylim([0 8])
xlim([0.0 size(eefile,1)*dt-dt])
ylabel('active\ C.P.','Interpreter','latex','FontSize',5)
yticks([0 2 4 6 8])
set(gca,'FontSize',15);
hold on


figure
grid on 
xlabel('time [s]','FontSize',15)
ylabel('error [m]','Interpreter','latex','FontSize',15)
hold on
plot(t',positiondesiredfile(:,1)-eefile(:,1),'r','LineWidth',2);
plot(t',positiondesiredfile(:,2)-eefile(:,2),'b','LineWidth',2);
plot(t',positiondesiredfile(:,3)-eefile(:,3),'g','LineWidth',2);
xlim([0.0 38])
set(gca,'FontSize',15);
legend('x','y','z');

figure
grid on 
xlabel('time [s]', 'FontSize', 15)
ylabel('error $[\cos\alpha_d - \cos\alpha]$','Interpreter','latex','FontSize', 15)
xlim([0.0 38])
plot(t',orien,'b','LineWidth',2);
hold on

index=find(orien<0);
orien2=orien;
orien2(index)=[];
t2=t;
t2(index)=[];
plot(t2',orien2,'r.','LineWidth',2);

set(gca,'FontSize',15);

dt=0.002;
t= 0 : dt :size(eefile,1)*dt-dt;
figure 
grid on 
xlabel('time [s]', 'FontSize', 15)
ylabel('joint positions $[rad]$','Interpreter','latex', 'FontSize', 15)
hold on
plot(t',qfile(:,1),'r','LineWidth',2);
plot(t',qfile(:,2),'k','LineWidth',2);
plot(t',qfile(:,3),'g','LineWidth',2);
plot(t',qfile(:,4),'b','LineWidth',2);
plot(t',qfile(:,5),'m','LineWidth',2);
plot(t',qfile(:,6),'c','LineWidth',2);
plot(t',qfile(:,7), 'color' ,[1 .5 0],'LineWidth',2);
set(gca,'FontSize',15);
legend('${q}_1$','${q}_2$','${q}_3$','${q}_4$','${q}_5$','${q}_6$','${q}_7$','Orientation','horizontal');
set(legend,'Interpreter','latex');
xlim([0.0 38])
set(legend,'FontSize',14);
hold on

dt=0.002;
t= 0 : dt :size(eefile,1)*dt-dt;
figure 
grid on 
xlabel('time [s]', 'FontSize', 15)
ylabel('joint velocities [rad/s]','Interpreter','latex', 'FontSize', 15)
hold on
plot(t',qdotfile(:,1),'r','LineWidth',2);
plot(t',qdotfile(:,2),'k','LineWidth',2);
plot(t',qdotfile(:,3),'g','LineWidth',2);
plot(t',qdotfile(:,4),'b','LineWidth',2);
plot(t',qdotfile(:,5),'m','LineWidth',2);
plot(t',qdotfile(:,6),'c','LineWidth',2);
plot(t',qdotfile(:,7),'color' ,[1 .5 0],'LineWidth',2);
set(gca,'FontSize',15);
legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','$\dot{q}_7$','Orientation','horizontal');
set(legend,'Interpreter','latex');
xlim([0.0 38])
set(legend,'FontSize',14);
hold on

dt=0.002;
set(gcf,'Position',[355 420 560 200])
set(gcf,'Position',[355 420 560 200])
set(gcf,'Position',[355 420 560 200])
t= 0 : dt :size(eefile,1)*dt-dt;
figure 
grid on 
xlabel('time [s]', 'FontSize', 15)
ylabel('joint accelerations $[rad/s^2]$','Interpreter','latex', 'FontSize', 15)
hold on
plot(t',qddotfile(:,1),'r','LineWidth',2);
plot(t',qddotfile(:,2),'k','LineWidth',2);
plot(t',qddotfile(:,3),'g','LineWidth',2);
plot(t',qddotfile(:,4),'b','LineWidth',2);
plot(t',qddotfile(:,5),'m','LineWidth',2);
plot(t',qddotfile(:,6),'c','LineWidth',2);
plot(t',qddotfile(:,7),'color' ,[1 .5 0],'LineWidth',2);
set(gca,'FontSize',15);
legend('$\ddot{q}_1$','$\ddot{q}_2$','$\ddot{q}_3$','$\ddot{q}_4$','$\ddot{q}_5$','$\ddot{q}_6$','$\ddot{q}_7$','Orientation','horizontal');
set(legend,'Interpreter','latex');
xlim([0.0 38])
set(legend,'FontSize',14);
hold on
