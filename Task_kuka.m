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

function [p,dp,ddp] = Task_kuka(lamda)
    
    cx=0;rx=0.2;cy=0.6;ry=0.1;h=0.2;%ellipse parameters
    
    px=cx+rx*cos(lamda);
    py=cy+ry*sin(lamda);
    pz=h*cos(lamda);
    
    dpx=-rx*sin(lamda);
    dpy=ry*cos(lamda);
    dpz=-h*sin(lamda);
    
    ddpx=-rx*cos(lamda);
    ddpy=-ry*sin(lamda);
    ddpz=-h*cos(lamda);
    
    p=[px;py;pz];
    dp=[dpx;dpy;dpz];
    ddp=[ddpx;ddpy;ddpz];

end