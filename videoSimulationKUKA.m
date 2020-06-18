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


set(0, 'defaultTextInterpreter', 'none'); 
hfig=figure;
set(gcf,'Position',[500 0 500 500])

q=QV;
samples=size(q,1);

T=0.001;
tt=0:T:samples*T-T;

ee0=T0d;
error_ee=e0;
error_eeori=e1;

subplot(3,1,1:2)

p=zeros(63,3);
k=1;
for ts=0:0.1:2*pi+0.1
    [p(k,:),~,~] = Task_kuka(ts);
    k=k+1;
end
plot3(p(:,1),p(:,2),p(:,3),'g','LineWidth',1);
hold on

kuka=get_KukaPoints(q(1,:));

kukax=kuka(1,:);
kukay=kuka(2,:);
kukaz=kuka(3,:);
hPlotRobot1=plot3(kukax,kukay,kukaz,'o-k','LineWidth',3);
set(hPlotRobot1,'XDataSource','kukax')
set(hPlotRobot1,'YDataSource','kukay')
set(hPlotRobot1,'ZDataSource','kukaz')
hold on

ee_X=ee0(1,1);
ee_Y=ee0(1,2);
ee_Z=ee0(1,3);
hPlotRH=plot3(ee_X,ee_Y,ee_Z,'r','LineWidth',2);
set(hPlotRH,'XDataSource','ee_X')
set(hPlotRH,'YDataSource','ee_Y')
set(hPlotRH,'ZDataSource','ee_Z')
hold on

%draw the cone for pointing task
Ze=Zd;
Ze=Ze/norm(Ze);
Pe=Pe1(1,:);
H = .078; %// height
R1 = H*tan(deg2rad(alpha_d)); %// radius
N = 100; %// number of points to define the circu reference
r1=Pe;
r2=Pe+Ze*H;
[x,y,z]=cylinder2P([0,R1],N,r1,r2);
drawCone=mesh(x, y, z);
set(drawCone,'XDataSource','x');
set(drawCone,'YDataSource','y');
set(drawCone,'ZDataSource','z');
hold on 

for k=1:1:size(p_obs,1)
    plot3(p_obs(k,1),p_obs(k,2),p_obs(k,3),'og','LineWidth',5);
end
hold on 

robot.set_q_and_qdot(q(1,:),[]);
ControlPoints=robot.compute_control_points_position;
ControlPointx=ControlPoints(1,:);
ControlPointy=ControlPoints(2,:);
ControlPointz=ControlPoints(3,:);
drawControlPoints= plot3(ControlPointx,ControlPointy,ControlPointz,'.r','MarkerSize',25);
set(drawControlPoints,'XDataSource','ControlPointx')
set(drawControlPoints,'YDataSource','ControlPointy')
set(drawControlPoints,'ZDataSource','ControlPointz')
hold on 


if controlPointMin(1,1)~=-1
    ReControlpointmin_X=ControlPoints(1,controlPointMin(1,1))+dpControlpointmin(1,1);
    ReControlpointmin_Y=ControlPoints(2,controlPointMin(1,1))+dpControlpointmin(1,2);
    ReControlpointmin_Z=ControlPoints(3,controlPointMin(1,1))+dpControlpointmin(1,3);
    hPlotRe=plot3(ReControlpointmin_X,ReControlpointmin_Y,ReControlpointmin_Z,'.b','MarkerSize',25);
    set(hPlotRe,'XDataSource','ReControlpointmin_X')
    set(hPlotRe,'YDataSource','ReControlpointmin_Y')
    set(hPlotRe,'ZDataSource','ReControlpointmin_Z')
    
    cp=ControlPoints(:,controlPointMin(1,1));
    distanceLinex=[p_obs(obscontrolPointMin(1,1),1);cp(1)];
    distanceLiney=[p_obs(obscontrolPointMin(1,1),2);cp(2)];
    distanceLinez=[p_obs(obscontrolPointMin(1,1),3);cp(3)];
    hPlotDisLine=plot3(distanceLinex,distanceLiney,distanceLinez,'-g','LineWidth',1);
    set(hPlotDisLine,'XDataSource','distanceLinex')
    set(hPlotDisLine,'YDataSource','distanceLiney')
    set(hPlotDisLine,'ZDataSource','distanceLinez')

    repulsiveLinex=[ReControlpointmin_X;cp(1)];
    repulsiveLiney=[ReControlpointmin_Y;cp(2)];
    repulsiveLinez=[ReControlpointmin_Z;cp(3)];
    hPlotReLine=plot3(repulsiveLinex,repulsiveLiney,repulsiveLinez,'-b','LineWidth',1);
    set(hPlotReLine,'XDataSource','repulsiveLinex')
    set(hPlotReLine,'YDataSource','repulsiveLiney')
    set(hPlotReLine,'ZDataSource','repulsiveLinez')
            
else
    ReControlpointmin_X=0;
    ReControlpointmin_Y=0;
    ReControlpointmin_Z=0;
    hPlotRe=plot3(ReControlpointmin_X,ReControlpointmin_Y,ReControlpointmin_Z,'.b','MarkerSize',25);
    set(hPlotRe,'XDataSource','ReControlpointmin_X')
    set(hPlotRe,'YDataSource','ReControlpointmin_Y')
    set(hPlotRe,'ZDataSource','ReControlpointmin_Z')
    
    
    distanceLinex=[0;0];
    distanceLiney=[0;0];
    distanceLinez=[0;0];
    hPlotDisLine=plot3(distanceLinex,distanceLiney,distanceLinez,'-g','LineWidth',1);
    set(hPlotDisLine,'XDataSource','distanceLinex')
    set(hPlotDisLine,'YDataSource','distanceLiney')
    set(hPlotDisLine,'ZDataSource','distanceLinez')

    repulsiveLinex=[0;0];
    repulsiveLiney=[0;0];
    repulsiveLinez=[0;0];
    hPlotReLine=plot3(repulsiveLinex,repulsiveLiney,repulsiveLinez,'-b','LineWidth',1);
    set(hPlotReLine,'XDataSource','repulsiveLinex')
    set(hPlotReLine,'YDataSource','repulsiveLiney')
    set(hPlotReLine,'ZDataSource','repulsiveLinez')
   
end


if controlPointMin2(1,1)~=0
    ReControlpointmin2_X=ControlPoints(1,controlPointMin2(1,1))+dpControlpointmin2(1,1);
    ReControlpointmin2_Y=ControlPoints(2,controlPointMin2(1,1))+dpControlpointmin2(1,2);
    ReControlpointmin2_Z=ControlPoints(3,controlPointMin2(1,1))+dpControlpointmin2(1,3);
    hPlotRe2=plot3(ReControlpointmin2_X,ReControlpointmin2_Y,ReControlpointmin2_Z,'.b','MarkerSize',25);
    set(hPlotRe2,'XDataSource','ReControlpointmin2_X')
    set(hPlotRe2,'YDataSource','ReControlpointmin2_Y')
    set(hPlotRe2,'ZDataSource','ReControlpointmin2_Z')
    
    cp2=ControlPoints(:,controlPointMin2(1,1));
    distanceLinex2=[p_obs(obscontrolPointMin2(1,1),1);cp2(1)];
    distanceLiney2=[p_obs(obscontrolPointMin2(1,1),2);cp2(2)];
    distanceLinez2=[p_obs(obscontrolPointMin2(1,1),3);cp2(3)];
    hPlotDisLine2=plot3(distanceLinex2,distanceLiney2,distanceLinez2,'-m','LineWidth',1);
    set(hPlotDisLine2,'XDataSource','distanceLinex2')
    set(hPlotDisLine2,'YDataSource','distanceLiney2')
    set(hPlotDisLine2,'ZDataSource','distanceLinez2')

    repulsiveLinex2=[ReControlpointmin2_X;cp2(1)];
    repulsiveLiney2=[ReControlpointmin2_Y;cp2(2)];
    repulsiveLinez2=[ReControlpointmin2_Z;cp2(3)];
    hPlotReLine2=plot3(repulsiveLinex2,repulsiveLiney2,repulsiveLinez2,'-b','LineWidth',1);
    set(hPlotReLine2,'XDataSource','repulsiveLinex2')
    set(hPlotReLine2,'YDataSource','repulsiveLiney2')
    set(hPlotReLine2,'ZDataSource','repulsiveLinez2')
            
else
    ReControlpointmin2_X=0;
    ReControlpointmin2_Y=0;
    ReControlpointmin2_Z=0;
    hPlotRe2=plot3(ReControlpointmin2_X,ReControlpointmin2_Y,ReControlpointmin2_Z,'.k','MarkerSize',25);
    set(hPlotRe2,'XDataSource','ReControlpointmin2_X')
    set(hPlotRe2,'YDataSource','ReControlpointmin2_Y')
    set(hPlotRe2,'ZDataSource','ReControlpointmin2_Z')
    
    
    distanceLinex2=[0;0];
    distanceLiney2=[0;0];
    distanceLinez2=[0;0];
    hPlotDisLine2=plot3(distanceLinex2,distanceLiney2,distanceLinez2,'-m','LineWidth',1);
    set(hPlotDisLine2,'XDataSource','distanceLinex2')
    set(hPlotDisLine2,'YDataSource','distanceLiney2')
    set(hPlotDisLine2,'ZDataSource','distanceLinez2')

    repulsiveLinex2=[0;0];
    repulsiveLiney2=[0;0];
    repulsiveLinez2=[0;0];
    hPlotReLine2=plot3(repulsiveLinex2,repulsiveLiney2,repulsiveLinez2,'-b','LineWidth',1);
    set(hPlotReLine2,'XDataSource','repulsiveLinex2')
    set(hPlotReLine2,'YDataSource','repulsiveLiney2')
    set(hPlotReLine2,'ZDataSource','repulsiveLinez2')
   
end
hold on



grid on
xlabel('X','FontSize',10);
ylabel('Y','FontSize',10);
zlabel('Z','FontSize',10);

az=128;
el=7;
view([az,el]);
axis manual
zlim([-.5,.5])
ylim([0,1])
xlim([-0.6,0.4])
axis square 

subplot(3,1,3)
TT=tt(1);
E_ee=error_ee(1);
E_eeori=error_eeori(1);
hplotEP=plot(2*TT,E_ee,'r','LineWidth',1.2);
hold on
set(hplotEP,'XDataSource','TT');
set(hplotEP,'YDataSource','E_ee')
hplotER=plot(2*TT,E_eeori,'b','LineWidth',1.2);
set(hplotER,'XDataSource','TT');
set(hplotER,'YDataSource','E_eeori')

grid on
xlabel('time [s]')
s=sprintf('task errors');
ylabel(s)
legend('pos','ori')
set(gca,'LineWidth',1.2,'FontSize',10)
ylim([-.05,0.05]);
xlim([0,2*Ttot]);

%vidObj = VideoWriter('acc alph5 ineq.avi'); %to save video
%open(vidObj);%to save video

f=1;
for i=1:samples
     
    if (mod(i,40)==1)  %15 for video, other 40
        
        kuka=get_KukaPoints(q(i,:));
        kukax=kuka(1,:);
        kukay=kuka(2,:);
        kukaz=kuka(3,:);
        refreshdata(hPlotRobot1)
        
        ee_X=ee0(1:i,1);
        ee_Y=ee0(1:i,2);
        ee_Z=ee0(1:i,3);
        refreshdata(hPlotRH)
        
        %draw the cone for pointing task
        Ze=Zd;
        Ze=Ze/norm(Ze);
        Pe=Pe1(i,:);
        H = .078; 
        R2 = H*tan(deg2rad(alpha_d)); 
        N = 100; 
        r1=Pe;
        r2=Pe+Ze*H;
        [x,y,z]=cylinder2P([0,R2],N,r1,r2);
        refreshdata(drawCone)

        robot.set_q_and_qdot(q(i,:),[]);
        ControlPoints=robot.compute_control_points_position;
        ControlPointx=ControlPoints(1,:);
        ControlPointy=ControlPoints(2,:);
        ControlPointz=ControlPoints(3,:);
        refreshdata(drawControlPoints)
        
        if controlPointMin(i,1)~=-1
            ReControlpointmin_X=ControlPoints(1,controlPointMin(i,1))+.0001*velocityControlPointMin(i,1)*dpControlpointmin(i,1);
            ReControlpointmin_Y=ControlPoints(2,controlPointMin(i,1))+.0001*velocityControlPointMin(i,1)*dpControlpointmin(i,2);
            ReControlpointmin_Z=ControlPoints(3,controlPointMin(i,1))+.0001*velocityControlPointMin(i,1)*dpControlpointmin(i,3);
            refreshdata(hPlotRe);
            
            cp=ControlPoints(:,controlPointMin(i,1));
      
            distanceLinex=[p_obs(obscontrolPointMin(i,1),1);cp(1)];
            distanceLiney=[p_obs(obscontrolPointMin(i,1),2);cp(2)];
            distanceLinez=[p_obs(obscontrolPointMin(i,1),3);cp(3)];
            refreshdata(hPlotDisLine);
            
            repulsiveLinex=[ReControlpointmin_X;cp(1)];
            repulsiveLiney=[ReControlpointmin_Y;cp(2)];
            repulsiveLinez=[ReControlpointmin_Z;cp(3)];
            refreshdata(hPlotReLine);
            
        else 
            ReControlpointmin_X=0;
            ReControlpointmin_Y=0;
            ReControlpointmin_Z=0;
            refreshdata(hPlotRe);
            
            
            distanceLinex=[0;0];
            distanceLiney=[0;0];
            distanceLinez=[0;0];
            refreshdata(hPlotDisLine);
            
            repulsiveLinex=[0;0];
            repulsiveLiney=[0;0];
            repulsiveLinez=[0;0];
            refreshdata(hPlotReLine);
        end
        
         if controlPointMin2(i,1)~=0
            ReControlpointmin2_X=ControlPoints(1,controlPointMin2(i,1))+velocityControlPointMin2(i,1)*dpControlpointmin2(i,1);
            ReControlpointmin2_Y=ControlPoints(2,controlPointMin2(i,1))+velocityControlPointMin2(i,1)*dpControlpointmin2(i,2);
            ReControlpointmin2_Z=ControlPoints(3,controlPointMin2(i,1))+velocityControlPointMin2(i,1)*dpControlpointmin2(i,3);
            refreshdata(hPlotRe2);
            
            cp2=ControlPoints(:,controlPointMin2(i,1));
            
            distanceLinex2=[p_obs(obscontrolPointMin2(i,1),1);cp2(1)];
            distanceLiney2=[p_obs(obscontrolPointMin2(i,1),2);cp2(2)];
            distanceLinez2=[p_obs(obscontrolPointMin2(i,1),3);cp2(3)];
            refreshdata(hPlotDisLine2);
            
            repulsiveLinex2=[ReControlpointmin2_X;cp2(1)];
            repulsiveLiney2=[ReControlpointmin2_Y;cp2(2)];
            repulsiveLinez2=[ReControlpointmin2_Z;cp2(3)];
            refreshdata(hPlotReLine2);
            
        else 
            ReControlpointmin2_X=0;
            ReControlpointmin2_Y=0;
            ReControlpointmin2_Z=0;
            refreshdata(hPlotRe2);
            
            
            distanceLinex2=[0;0];
            distanceLiney2=[0;0];
            distanceLinez2=[0;0];
            refreshdata(hPlotDisLine2);
            
            repulsiveLinex2=[0;0];
            repulsiveLiney2=[0;0];
            repulsiveLinez2=[0;0];
            refreshdata(hPlotReLine2);
        end

        subplot(3,1,1:2)
       
        TT=2*tt(1:i);
        
        E_ee=error_ee(1:i);
        E_eeori=error_eeori(1:i);
        refreshdata(hplotEP)        
        refreshdata(hplotER)        

        M(f)=getframe(hfig);
        %writeVideo(vidObj,M(f));%to save video
       
        f=f+1;
    end

end
%close(vidObj);%to save video

disp('VIDEO PERFORMED') 

