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


function [Kuka]=get_KukaPoints(q)

    global robot
   
    Kuka=zeros(3,4);
    
    ph=[0;0;0;1];
    A0=robot.transMat(q(1),0,1,0,robot.d1);
    A1=A0*robot.transMat(q(2),0,-1,0,0);
    A2=A1*robot.transMat(q(3),0,-1,0,robot.d3);
    A3=A2*robot.transMat(q(4),0,1,0,0);
    A4=A3*robot.transMat(q(5),0,1,0,robot.d5);
    A5=A4*robot.transMat(q(6),0,-1,0,0);
    A6=A5*robot.transMat(0,1,0,0,robot.d7);
    
    p0=A0*ph;
    p0=p0(1:3);
    Kuka(:,1)=p0';%the position of joint 2
    
    p2=A2*ph;
    p2=p2(1:3);
    Kuka(:,2)=p2';%the position of joint 4 
         
    p4=A4*ph;
    p4=p4(1:3);
    Kuka(:,3)=p4';%the position of joint 6
      
    p6=A6*ph;
    p6=p6(1:3);
    Kuka(:,4)=p6';%the position of end-effector
    
end