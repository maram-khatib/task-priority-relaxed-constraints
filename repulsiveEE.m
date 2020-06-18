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

function [dp,distancemin,velocity,nn] = repulsiveEE(ee0,p_obs)
    
    %intialization
    velocity=0.0;% repulsive density
    dp=zeros(3,1);% repulsive vector
    nn=zeros(3,1);% normalized direction
    distancemin=0.11;% from closest obstacle, the initial should be > danger threshold
    
    for j=1:1:size(p_obs,1)
        distance= norm(ee0-p_obs(j,:)');
        direction= (ee0-p_obs(j,:)')/distance;

        survaliance=0.1;%danger threshold
        v_max=5;
        rho=survaliance;
        gamma=10;% more high more sharp
        const=1.5;%shape parameter: more high more slow to start

        if(distance<survaliance)
            
            v= v_max/ (1+ exp(distance*(const/rho)*gamma-gamma));
            
            if(v>velocity)  
                velocity=v;
                dp= direction*velocity;
                nn=(direction/norm(direction))';% corresponding to the closest object
                distancemin=distance;
                
            end
        end
    end
end