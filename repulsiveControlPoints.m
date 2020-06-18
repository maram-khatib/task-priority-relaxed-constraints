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

function [distanceControlPoints,obsControlPoints,dpControlPoints,velocityControlPoints,nnControlPoints] = repulsiveControlPoints(controlPoints,p_obs)
    
   %intialization
    velocityControlPoints=zeros(1,8);% repulsive density
    distanceControlPoints=ones(1,8);
    distanceControlPoints=distanceControlPoints*.16;%from closest obstacle
    obsControlPoints=zeros(1,8);% the closest obstacle
    dpControlPoints=zeros(3,8);% repulsive vector
    nnControlPoints=zeros(3,8);% normalized direction
    
    for i=1:1:8
        for j=1:1:size(p_obs,1)
            distance= norm(controlPoints(:,i) - p_obs(j,:)');
            direction=(controlPoints(:,i) - p_obs(j,:)')/distance;

            survaliance1=0.15;% far danger threshold
            v_max1=.15;
            rho1=survaliance1;
            gamma1=2;% more high more sharp
            const1=3;%more high more slow to start

            if(distance<survaliance1)
               
               velocity= v_max1 / (1+ exp(distance*(const1/rho1)*gamma1-gamma1));
                     
                if(velocity>velocityControlPoints(1,i))
                    distanceControlPoints(1,i)=norm(controlPoints(:,i) - p_obs(j,:)');
                    obsControlPoints(1,i)= j;
                    velocityControlPoints(1,i)=velocity;
                    dpControlPoints(:,i)= direction*velocityControlPoints(1,i);
                    nnControlPoints(:,i)=(direction/norm(direction))';% corresponding to the closest object
                end
            end
        end
        
    end
end