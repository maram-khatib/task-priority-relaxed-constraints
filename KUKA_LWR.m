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


classdef KUKA_LWR<handle
    
    properties
        d1; % distance from base to second joint
        d3; % distance from second joint to fourth joint
        d5; % distance from fourth joint to sixth joint
        d7; % distance from sixth joint to end-effector
        q; % 7x1 vector with the current joint configuration
        qdot; % 7x1 vector with the current joint velocity 
        
        PrevJacobianPos; % 3x7 matrix
        PrevJacobianOri; % 1x7 matrix
        PrevJacobianControlPoints; % 3x7x8 matrix related to the 8 control points
    
    end
    
    methods
        function obj=KUKA_LWR(d1,d3,d5,d7,q,qdot,zd)
            
            obj.d1=d1;
            obj.d3=d3;
            obj.d5=d5;
            obj.d7=d7;
            obj.q=q;
            obj.qdot=qdot;
            obj.PrevJacobianPos = obj.computeJacobianPos;
            [~,ze]=obj.computeEnd_effectorOrientation;
            [~,obj.PrevJacobianOri] = obj.computeJacobianOri(zd,ze);
            
            cpJacobian = obj.compute_control_points_jacobian;
            obj.PrevJacobianControlPoints =cpJacobian;
            
        end
        
        function obj=set_q_and_qdot(obj,q,qdot)
            obj.q=q;
            obj.qdot=qdot;
        end
        
        function obj=set_update(obj,zd)
            obj.PrevJacobianPos = obj.computeJacobianPos;
            [~,ze]=obj.computeEnd_effectorOrientation;
            [~,obj.PrevJacobianOri] = obj.computeJacobianOri(zd,ze);
            
            cpJacobian = obj.compute_control_points_jacobian;
            obj.PrevJacobianControlPoints =cpJacobian;
        end
           
        function J=computeJacobianPos(obj)
       
            %for jacobian 3*7
            l2=obj.d3;
            l3=obj.d5;
            l4=obj.d7;
            
            s1=sin(obj.q(1));
            s2=sin(obj.q(2));
            s3=sin(obj.q(3));
            s4=sin(obj.q(4));
            s5=sin(obj.q(5));
            s6=sin(obj.q(6));
            c1=cos(obj.q(1));
            c2=cos(obj.q(2));
            c3=cos(obj.q(3));
            c4=cos(obj.q(4));
            c5=cos(obj.q(5));
            c6=cos(obj.q(6));
           
        %position jacobian 3*7  
        J =[    l2*s1*s2 - l3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - l4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))),-l4*c1*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) - l3*c1*(c2*c4 + c3*s2*s4) - l2*c1*c2,l4*c1*c2*c3*s5*s6 - l3*c1*c2*s3*s4 - l4*c3*c6*s1*s4 - l4*s1*s3*s5*s6 - l4*c1*c2*c6*s3*s4 - l3*c3*s1*s4 + l4*c3*c4*c5*s1*s6 + l4*c1*c2*c4*c5*s3*s6,(c1*c3 - c2*s1*s3)*(l4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + l3*(c2*c4 + c3*s2*s4)) + s2*s3*(l4*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + l3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)),l4*s6*(c3*c5*s1 + c1*c2*c5*s3 + c1*s2*s4*s5 - c4*s1*s3*s5 + c1*c2*c3*c4*s5),l4*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + l4*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)),0;
                -l4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - l3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - l2*c1*s2, -l4*s1*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) - l3*s1*(c2*c4 + c3*s2*s4) - l2*c2*s1,l3*c1*c3*s4 + l4*c1*c3*c6*s4 - l3*c2*s1*s3*s4 + l4*c1*s3*s5*s6 - l4*c1*c3*c4*c5*s6 - l4*c2*c6*s1*s3*s4 + l4*c2*c3*s1*s5*s6 + l4*c2*c4*c5*s1*s3*s6,(c3*s1 + c1*c2*s3)*(l4*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + l3*(c2*c4 + c3*s2*s4)) + s2*s3*(l4*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + l3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)),l4*s6*(c2*c5*s1*s3 - c1*c3*c5 + c1*c4*s3*s5 + s1*s2*s4*s5 + c2*c3*c4*s1*s5),l4*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + l4*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)),0;
                 0,l3*c2*c3*s4 - l3*c4*s2 - l2*s2 - l4*c4*c6*s2 + l4*c2*c3*c6*s4 + l4*c2*s3*s5*s6 - l4*c5*s2*s4*s6 - l4*c2*c3*c4*c5*s6,-s2*(l3*s3*s4 + l4*c6*s3*s4 - l4*c3*s5*s6 - l4*c4*c5*s3*s6), l3*c3*c4*s2 - l3*c2*s4 - l4*c2*c6*s4 + l4*c3*c4*c6*s2 + l4*c2*c4*c5*s6 + l4*c3*c5*s2*s4*s6,l4*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5), l4*c2*c5*c6*s4 - l4*c2*c4*s6 - l4*c3*s2*s4*s6 + l4*c6*s2*s3*s5 - l4*c3*c4*c5*c6*s2,0];
             
        
        end
        
       function [J,Jori]=computeJacobianOri(obj,Zd,Ze)
           
            s1=sin(obj.q(1));
            s2=sin(obj.q(2));
            s3=sin(obj.q(3));
            s4=sin(obj.q(4));
            s5=sin(obj.q(5));
            s6=sin(obj.q(6));
            c1=cos(obj.q(1));
            c2=cos(obj.q(2));
            c3=cos(obj.q(3));
            c4=cos(obj.q(4));
            c5=cos(obj.q(5));
            c6=cos(obj.q(6));
            
            %jacobian ori 3*7    
            J =[ 0,s1,-c1*s2,-c3*s1 - c1*c2*s3,-s4* (s1*s3 - c1*c2*c3) - c1*c4*s2,c5* (c3*s1 + c1*c2*s3) - s5* (c4* (s1*s3 - c1*c2*c3) - c1*s2*s4),s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2);
                 0,-c1,-s1*s2,c1*c3 - c2*s1*s3,s4* (c1*s3 + c2*c3*s1) - c4*s1*s2,s5* (c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3),c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
                 1,0,c2,-s2*s3,c2*c4 + c3*s2*s4,c5*s2*s3 - s5*(c2*s4 - c3*c4*s2),s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)];    
                
            v =[ 0, -Ze(3), Ze(2); Ze(3), 0, -Ze(1); -Ze(2), Ze(1), 0];
	
            Jori=Zd*v'*J;
       end
        
        function Jdot=computeJacobianDot(obj)
            %%%%Compute the time derivative of the Jacobian%%%%%%%%%%%%%%%
            l2=obj.d3;
            l4=obj.d5;
            l7=obj.d7;
            qdot1=obj.qdot(1);
            qdot2=obj.qdot(2);
            qdot3=obj.qdot(3);
            qdot4=obj.qdot(4);
            qdot5=obj.qdot(5);
            qdot6=obj.qdot(6);
            s1=sin(obj.q(1));
            s2=sin(obj.q(2));
            s3=sin(obj.q(3));
            s4=sin(obj.q(4));
            s5=sin(obj.q(5));
            s6=sin(obj.q(6));
            c1=cos(obj.q(1));
            c2=cos(obj.q(2));
            c3=cos(obj.q(3));
            c4=cos(obj.q(4));
            c5=cos(obj.q(5));
            c6=cos(obj.q(6));
            Jdot=[qdot1*(l4*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)+l7*(c6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-s6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)))+l2*c1*s2)-qdot4*(l7*(c6*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+c5*s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2))+l4*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4))-qdot3*(l7*(s6*(s5*(c1*s3+c2*c3*s1)-c4*c5*(c1*c3-c2*s1*s3))+c6*s4*(c1*c3-c2*s1*s3))+l4*s4*(c1*c3-c2*s1*s3))+qdot2*(l7*(s1*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+c6*s1*(c2*c4+c3*s2*s4))+l4*s1*(c2*c4+c3*s2*s4)+l2*c2*s1)+l7*qdot6*(s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)))-l7*qdot5*s6*(s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-c5*(c1*c3-c2*s1*s3)),qdot4*(l4*c1*(c2*s4-c3*c4*s2)+l7*c1*(c6*(c2*s4-c3*c4*s2)-c5*s6*(c2*c4+c3*s2*s4)))+qdot2*(l4*c1*(c4*s2-c2*c3*s4)+l2*c1*s2+l7*c1*(s6*(c5*(s2*s4+c2*c3*c4)-c2*s3*s5)+c6*(c4*s2-c2*c3*s4)))+qdot1*(l4*s1*(c2*c4+c3*s2*s4)+l2*c2*s1+l7*s1*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4)))-qdot3*(l7*c1*(s6*(c3*s2*s5+c4*c5*s2*s3)-c6*s2*s3*s4)-l4*c1*s2*s3*s4)-l7*qdot6*c1*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))+l7*qdot5*c1*s6*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3),l7*qdot6*(c3*s1*s4*s6-c6*s1*s3*s5+c1*c2*c3*c6*s5+c3*c4*c5*c6*s1+c1*c2*s3*s4*s6+c1*c2*c4*c5*c6*s3)-qdot3*(l4*c1*c2*c3*s4-l4*s1*s3*s4-l7*c6*s1*s3*s4+l7*c3*s1*s5*s6+l7*c1*c2*c3*c6*s4+l7*c1*c2*s3*s5*s6+l7*c4*c5*s1*s3*s6-l7*c1*c2*c3*c4*c5*s6)-qdot4*(c3*s1+c1*c2*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot1*(l4*c1*c3*s4+l7*c1*c3*c6*s4-l4*c2*s1*s3*s4+l7*c1*s3*s5*s6-l7*c1*c3*c4*c5*s6-l7*c2*c6*s1*s3*s4+l7*c2*c3*s1*s5*s6+l7*c2*c4*c5*s1*s3*s6)+qdot2*c1*s2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)-l7*qdot5*s6*(c5*s1*s3-c1*c2*c3*c5+c3*c4*s1*s5+c1*c2*c4*s3*s5),qdot4*(l4*c1*c4*s2+l4*s1*s3*s4-l4*c1*c2*c3*s4+l7*c1*c4*c6*s2+l7*c6*s1*s3*s4-l7*c1*c2*c3*c6*s4+l7*c1*c5*s2*s4*s6-l7*c4*c5*s1*s3*s6+l7*c1*c2*c3*c4*c5*s6)-qdot1*(l4*c1*c4*s3+l4*s1*s2*s4+l4*c2*c3*c4*s1+l7*c1*c4*c6*s3+l7*c6*s1*s2*s4+l7*c2*c3*c4*c6*s1-l7*c4*c5*s1*s2*s6+l7*c1*c5*s3*s4*s6+l7*c2*c3*c5*s1*s4*s6)-qdot3*(c3*s1+c1*c2*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-l7*qdot6*(c1*s2*s4*s6-c4*s1*s3*s6+c1*c2*c3*c4*s6+c1*c4*c5*c6*s2+c5*c6*s1*s3*s4-c1*c2*c3*c5*c6*s4)-qdot2*c1*(l4*c3*c4*s2-l4*c2*s4-l7*c2*c6*s4+l7*c3*c4*c6*s2+l7*c2*c4*c5*s6+l7*c3*c5*s2*s4*s6)+l7*qdot5*s5*s6*(s1*s3*s4+c1*c4*s2-c1*c2*c3*s4),l7*qdot6*c6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)-l7*qdot5*s6*(c3*s1*s5+c1*c2*s3*s5-c1*c5*s2*s4+c4*c5*s1*s3-c1*c2*c3*c4*c5)-l7*qdot1*s6*(c2*c5*s1*s3-c1*c3*c5+c1*c4*s3*s5+s1*s2*s4*s5+c2*c3*c4*s1*s5)-l7*qdot3*s6*(c5*s1*s3-c1*c2*c3*c5+c3*c4*s1*s5+c1*c2*c4*s3*s5)+l7*qdot4*s5*s6*(s1*s3*s4+c1*c4*s2-c1*c2*c3*s4)-l7*qdot2*c1*s6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5),qdot6*(l7*(s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-c5*(c1*c3-c2*s1*s3))*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))-l7*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3)*(s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3))))-qdot1*(l7*(s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-c5*(c3*s1+c1*c2*s3))*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4))+l7*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3)*(c6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-s6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3))))+qdot3*(l7*c3*s1*s4*s6-l7*c6*s1*s3*s5+l7*c1*c2*c3*c6*s5+l7*c3*c4*c5*c6*s1+l7*c1*c2*s3*s4*s6+l7*c1*c2*c4*c5*c6*s3)-qdot4*(l7*c1*s2*s4*s6-l7*c4*s1*s3*s6+l7*c1*c2*c3*c4*s6+l7*c1*c4*c5*c6*s2+l7*c5*c6*s1*s3*s4-l7*c1*c2*c3*c5*c6*s4)+l7*qdot5*c6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)+l7*qdot2*c1*(c2*c4*s6-c2*c5*c6*s4+c3*s2*s4*s6-c6*s2*s3*s5+c3*c4*c5*c6*s2),0;
                  l7*qdot6*(s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)+c6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)))-qdot2*(l7*(c1*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+c1*c6*(c2*c4+c3*s2*s4))+l4*c1*(c2*c4+c3*s2*s4)+l2*c1*c2)-qdot1*(l4*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+l7*(c6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)-s6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)))-l2*s1*s2)-qdot3*(l7*(s6*(s5*(s1*s3-c1*c2*c3)-c4*c5*(c3*s1+c1*c2*s3))+c6*s4*(c3*s1+c1*c2*s3))+l4*s4*(c3*s1+c1*c2*s3))-qdot4*(l7*(c6*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+c5*s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2))+l4*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4))-l7*qdot5*s6*(s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-c5*(c3*s1+c1*c2*s3)),qdot4*(l4*s1*(c2*s4-c3*c4*s2)+l7*s1*(c6*(c2*s4-c3*c4*s2)-c5*s6*(c2*c4+c3*s2*s4)))-qdot1*(l4*c1*(c2*c4+c3*s2*s4)+l2*c1*c2+l7*c1*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4)))+qdot2*(l4*s1*(c4*s2-c2*c3*s4)+l2*s1*s2+l7*s1*(s6*(c5*(s2*s4+c2*c3*c4)-c2*s3*s5)+c6*(c4*s2-c2*c3*s4)))-qdot3*(l7*s1*(s6*(c3*s2*s5+c4*c5*s2*s3)-c6*s2*s3*s4)-l4*s1*s2*s3*s4)-l7*qdot6*s1*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))+l7*qdot5*s1*s6*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3),qdot4*(c1*c3-c2*s1*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot3*(l4*c1*s3*s4+l4*c2*c3*s1*s4+l7*c1*c6*s3*s4-l7*c1*c3*s5*s6+l7*c2*c3*c6*s1*s4-l7*c1*c4*c5*s3*s6+l7*c2*s1*s3*s5*s6-l7*c2*c3*c4*c5*s1*s6)-qdot1*(l4*c3*s1*s4+l4*c1*c2*s3*s4+l7*c3*c6*s1*s4+l7*s1*s3*s5*s6+l7*c1*c2*c6*s3*s4-l7*c1*c2*c3*s5*s6-l7*c3*c4*c5*s1*s6-l7*c1*c2*c4*c5*s3*s6)+l7*qdot6*(c1*c6*s3*s5-c1*c3*s4*s6-c1*c3*c4*c5*c6+c2*c3*c6*s1*s5+c2*s1*s3*s4*s6+c2*c4*c5*c6*s1*s3)+qdot2*s1*s2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)+l7*qdot5*s6*(c1*c5*s3+c2*c3*c5*s1+c1*c3*c4*s5-c2*c4*s1*s3*s5),qdot1*(l4*c1*s2*s4-l4*c4*s1*s3+l4*c1*c2*c3*c4+l7*c1*c6*s2*s4-l7*c4*c6*s1*s3-l7*c1*c4*c5*s2*s6-l7*c5*s1*s3*s4*s6+l7*c1*c2*c3*c4*c6+l7*c1*c2*c3*c5*s4*s6)+qdot4*(l4*c4*s1*s2-l4*c1*s3*s4-l4*c2*c3*s1*s4+l7*c4*c6*s1*s2-l7*c1*c6*s3*s4-l7*c2*c3*c6*s1*s4+l7*c1*c4*c5*s3*s6+l7*c5*s1*s2*s4*s6+l7*c2*c3*c4*c5*s1*s6)+qdot3*(c1*c3-c2*s1*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot2*s1*(l4*c3*c4*s2-l4*c2*s4-l7*c2*c6*s4+l7*c3*c4*c6*s2+l7*c2*c4*c5*s6+l7*c3*c5*s2*s4*s6)-l7*qdot6*(c1*c4*s3*s6+s1*s2*s4*s6+c2*c3*c4*s1*s6+c4*c5*c6*s1*s2-c1*c5*c6*s3*s4-c2*c3*c5*c6*s1*s4)-l7*qdot5*s5*s6*(c1*s3*s4-c4*s1*s2+c2*c3*s1*s4),l7*qdot1*s6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)+l7*qdot5*s6*(c1*c3*s5+c1*c4*c5*s3-c2*s1*s3*s5+c5*s1*s2*s4+c2*c3*c4*c5*s1)+l7*qdot6*c6*(c2*c5*s1*s3-c1*c3*c5+c1*c4*s3*s5+s1*s2*s4*s5+c2*c3*c4*s1*s5)+l7*qdot3*s6*(c1*c5*s3+c2*c3*c5*s1+c1*c3*c4*s5-c2*c4*s1*s3*s5)-l7*qdot4*s5*s6*(c1*s3*s4-c4*s1*s2+c2*c3*s1*s4)-l7*qdot2*s1*s6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5),l7*(qdot1*c1*c4*s2*s6-qdot5*c1*c3*c5*c6+qdot2*c2*c4*s1*s6+qdot1*c3*c6*s1*s5-qdot3*c1*c3*s4*s6+qdot3*c1*c6*s3*s5-qdot4*c1*c4*s3*s6+qdot6*c4*c6*s1*s2-qdot6*c1*c6*s3*s4+qdot6*c1*c3*s5*s6+qdot1*s1*s3*s4*s6-qdot4*s1*s2*s4*s6-qdot1*c1*c2*c3*s4*s6+qdot1*c1*c2*c6*s3*s5-qdot1*c1*c5*c6*s2*s4+qdot1*c4*c5*c6*s1*s3-qdot2*c2*c5*c6*s1*s4+qdot3*c2*c3*c6*s1*s5-qdot4*c2*c3*c4*s1*s6-qdot4*c4*c5*c6*s1*s2+qdot5*c2*c5*c6*s1*s3-qdot6*c2*c3*c6*s1*s4+qdot4*c1*c5*c6*s3*s4+qdot5*c1*c4*c6*s3*s5+qdot6*c1*c4*c5*s3*s6+qdot2*c3*s1*s2*s4*s6-qdot2*c6*s1*s2*s3*s5+qdot3*c2*s1*s3*s4*s6+qdot5*c6*s1*s2*s4*s5-qdot6*c2*s1*s3*s5*s6+qdot6*c5*s1*s2*s4*s6-qdot3*c1*c3*c4*c5*c6-qdot1*c1*c2*c3*c4*c5*c6+qdot2*c3*c4*c5*c6*s1*s2+qdot3*c2*c4*c5*c6*s1*s3+qdot4*c2*c3*c5*c6*s1*s4+qdot5*c2*c3*c4*c6*s1*s5+qdot6*c2*c3*c4*c5*s1*s6),0;
                  0,qdot4*(l4*s2*s4+l4*c2*c3*c4+l7*c6*s2*s4+l7*c2*c3*c4*c6-l7*c4*c5*s2*s6+l7*c2*c3*c5*s4*s6)-qdot2*(l2*c2+l4*c2*c4+l7*c2*c4*c6+l4*c3*s2*s4+l7*c3*c6*s2*s4+l7*c2*c5*s4*s6+l7*s2*s3*s5*s6-l7*c3*c4*c5*s2*s6)-l7*qdot6*(c2*c3*s4*s6-c4*s2*s6-c2*c6*s3*s5+c5*c6*s2*s4+c2*c3*c4*c5*c6)-qdot3*c2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)+l7*qdot5*s6*(s2*s4*s5+c2*c5*s3+c2*c3*c4*s5),l7*qdot6*s2*(s3*s4*s6+c3*c6*s5+c4*c5*c6*s3)-qdot2*c2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)-qdot3*s2*(l4*c3*s4+l7*c3*c6*s4+l7*s3*s5*s6-l7*c3*c4*c5*s6)-qdot4*s2*s3*(l4*c4+l7*c4*c6+l7*c5*s4*s6)+l7*qdot5*s2*s6*(c3*c5-c4*s3*s5),qdot2*(l4*s2*s4+l4*c2*c3*c4+l7*c6*s2*s4+l7*c2*c3*c4*c6-l7*c4*c5*s2*s6+l7*c2*c3*c5*s4*s6)-qdot4*(l4*c2*c4+l7*c2*c4*c6+l4*c3*s2*s4+l7*c3*c6*s2*s4+l7*c2*c5*s4*s6-l7*c3*c4*c5*s2*s6)+l7*qdot6*(c2*s4*s6+c2*c4*c5*c6-c3*c4*s2*s6+c3*c5*c6*s2*s4)-qdot3*s2*s3*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-l7*qdot5*s5*s6*(c2*c4+c3*s2*s4),l7*qdot2*s6*(s2*s4*s5+c2*c5*s3+c2*c3*c4*s5)-l7*qdot5*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+l7*qdot6*c6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5)-l7*qdot4*s5*s6*(c2*c4+c3*s2*s4)+l7*qdot3*s2*s6*(c3*c5-c4*s3*s5),l7*qdot4*(c2*s4*s6+c2*c4*c5*c6-c3*c4*s2*s6+c3*c5*c6*s2*s4)-l7*qdot6*(c2*c4*c6+c3*c6*s2*s4+c2*c5*s4*s6+s2*s3*s5*s6-c3*c4*c5*s2*s6)-l7*qdot2*(c2*c3*s4*s6-c4*s2*s6-c2*c6*s3*s5+c5*c6*s2*s4+c2*c3*c4*c5*c6)+l7*qdot5*c6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5)+l7*qdot3*s2*(s3*s4*s6+c3*c6*s5+c4*c5*c6*s3),0];
        end
        
        function p=computeEnd_effectorPosition(obj)
            p0=[0;0;0;1];
            A=obj.transMat(obj.q(1),0,1,0,obj.d1);
            A=A*obj.transMat(obj.q(2),0,-1,0,0);
            A=A*obj.transMat(obj.q(3),0,-1,0,obj.d3);
            A=A*obj.transMat(obj.q(4),0,1,0,0);
            A=A*obj.transMat(obj.q(5),0,1,0,obj.d5);
            A=A*obj.transMat(obj.q(6),0,-1,0,0);
            A=A*obj.transMat(0,1,0,0,obj.d7);
            p=A*p0;
            p=p(1:3);
        end
        
        function controlPoints=compute_control_points_position(obj)
            controlPoints=zeros(3,8);
            ARM_CONTROL_POINTS=4;
            FOREARM_CONTROL_POINTS=4;
            LINK2_LENGHT=obj.d3;
            LINK3_LENGHT=obj.d5;
            
            p0=[0;0;0;1];
            %W1
            A=obj.transMat(obj.q(1),0,1,0,obj.d1);
            kineWToFrame0=A;
            %M2
            A=A*obj.transMat(obj.q(2),0,-1,0,0);
            %M3
            A=A*obj.transMat(obj.q(3),0,-1,0,obj.d3);
            kineWToFrame1=A;
            %M4
            A=A*obj.transMat(obj.q(4),0,1,0,0);
            %M5
            A=A*obj.transMat(obj.q(5),0,1,0,obj.d5);
            kineWToFrame2=A;
            %M6
            A=A*obj.transMat(obj.q(6),0,-1,0,0);
            %M7
            A=A*obj.transMat(0,1,0,0,obj.d7);
            kineWToFrame3=A;
     
            P=[0;0;0;1];
            for i=1:1:ARM_CONTROL_POINTS
                l=(LINK2_LENGHT-LINK2_LENGHT/2)*(i-1)/ARM_CONTROL_POINTS;
                P(2)=l;
                Ph=kineWToFrame1*P;
                controlPoints(1,i)=Ph(1)/Ph(4);
                controlPoints(2,i)=Ph(2)/Ph(4);
                controlPoints(3,i)=Ph(3)/Ph(4);
            end
            for i=1:1:FOREARM_CONTROL_POINTS
                l=LINK3_LENGHT*(i-1)/FOREARM_CONTROL_POINTS;
                P(2)=-l;
                Ph=kineWToFrame2*P;
                controlPoints(1,i+ARM_CONTROL_POINTS)=Ph(1)/Ph(4);
                controlPoints(2,i+ARM_CONTROL_POINTS)=Ph(2)/Ph(4);
                controlPoints(3,i+ARM_CONTROL_POINTS)=Ph(3)/Ph(4);
            end
        end
        
        function controlPointsJacobian = compute_control_points_jacobian(obj)
            ARM_CONTROL_POINTS=4;
            FOREARM_CONTROL_POINTS=4;
            LINK2_LENGHT=obj.d3;
            LINK3_LENGHT=obj.d5;
            
            controlPointsJacobian=zeros(3,7,8);
            l2=obj.d3;
            l3=obj.d5;
            l4=obj.d7;

            s1=sin(obj.q(1));
            s2=sin(obj.q(2));
            s3=sin(obj.q(3));
            s4=sin(obj.q(4));
            s5=sin(obj.q(5));
            s6=sin(obj.q(6));
            c1=cos(obj.q(1));
            c2=cos(obj.q(2));
            c3=cos(obj.q(3));
            c4=cos(obj.q(4));
            c5=cos(obj.q(5));
            c6=cos(obj.q(6));
            %Mat M2,M3,M4,M5,M6,M7,
            for i=1:1:ARM_CONTROL_POINTS
                l=(LINK2_LENGHT-LINK2_LENGHT/2)*(i-1)/ARM_CONTROL_POINTS;
                controlPointsJacobian(1,1,i)=l*s1*s2;
                controlPointsJacobian(2,1,i)=-l*c1*c2;
                controlPointsJacobian(3,1,i)=0;
               
                controlPointsJacobian(1,2,i)=-l*c1*c2;
                controlPointsJacobian(2,2,i)=-l*s1*c2;
                controlPointsJacobian(3,2,i)=-l*s2*c1*c1 - l*s2*s1*s1;

                controlPointsJacobian(1,3,i)= 0;
                controlPointsJacobian(2,3,i)= 0;
                controlPointsJacobian(3,3,i)= 0;
                
                controlPointsJacobian(1,4,i)= 0;
                controlPointsJacobian(2,4,i)= 0;
                controlPointsJacobian(3,4,i)= 0;
                
                controlPointsJacobian(1,5,i)= 0;
                controlPointsJacobian(2,5,i)= 0;
                controlPointsJacobian(3,5,i)= 0;	
                
                controlPointsJacobian(1,6,i)= 0;
                controlPointsJacobian(2,6,i)= 0;
                controlPointsJacobian(3,6,i)= 0;
                
                controlPointsJacobian(1,7,i)= 0;
                controlPointsJacobian(2,7,i)= 0;
                controlPointsJacobian(3,7,i)= 0;
            end
            for i=1:1:FOREARM_CONTROL_POINTS
                l=LINK3_LENGHT*(i-1)/FOREARM_CONTROL_POINTS;

                controlPointsJacobian(1,1,i+ARM_CONTROL_POINTS)=l2*s1*s2 - l*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
                controlPointsJacobian(2,1,i+ARM_CONTROL_POINTS)=- l*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - l2*c1*s2;
                controlPointsJacobian(3,1,i+ARM_CONTROL_POINTS)= 0;
 
                controlPointsJacobian(1,2,i+ARM_CONTROL_POINTS)=-c1*(l*(c2*c4 + c3*s2*s4) + l2*c2);
                controlPointsJacobian(2,2,i+ARM_CONTROL_POINTS)=-s1*(l*(c2*c4 + c3*s2*s4) + l2*c2);
                controlPointsJacobian(3,2,i+ARM_CONTROL_POINTS)= s1*(l*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - l2*s1*s2) - c1*(l*(s4*(s1*s3 -c1*c2*c3) + c1*c4*s2) + l2*c1*s2);

                controlPointsJacobian(1,3,i+ARM_CONTROL_POINTS)=- c2*(l*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - l2*s1*s2) - s1*s2*(l*(c2*c4 +c3*s2*s4) + l2*c2);
                controlPointsJacobian(2,3,i+ARM_CONTROL_POINTS)=c1*s2*(l*(c2*c4 + c3*s2*s4) + l2*c2) - c2*(l*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) +l2*c1*s2);
                controlPointsJacobian(3,3,i+ARM_CONTROL_POINTS)=- s1*s2*(l*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + l2*c1*s2) - c1*s2*(l*(s4*(c1*s3 +c2*c3*s1) - c4*s1*s2) - l2*s1*s2);

                controlPointsJacobian(1,4,i+ARM_CONTROL_POINTS)=l*(c1*c3 - c2*s1*s3)*(c2*c4 + c3*s2*s4) + l*s2*s3*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
                controlPointsJacobian(2,4,i+ARM_CONTROL_POINTS)=l*(c3*s1 + c1*c2*s3)*(c2*c4 + c3*s2*s4) + l*s2*s3*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2);
                controlPointsJacobian(3,4,i+ARM_CONTROL_POINTS)=l*(c1*c3 - c2*s1*s3)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - l*(s4*(c1*s3 +c2*c3*s1) - c4*s1*s2)*(c3*s1 + c1*c2*s3);


                controlPointsJacobian(1,5,i+ARM_CONTROL_POINTS)=0;
                controlPointsJacobian(2,5,i+ARM_CONTROL_POINTS)=0;
                controlPointsJacobian(3,5,i+ARM_CONTROL_POINTS)=0;
                
                controlPointsJacobian(1,6,i+ARM_CONTROL_POINTS)=0;
                controlPointsJacobian(2,6,i+ARM_CONTROL_POINTS)=0;
                controlPointsJacobian(3,6,i+ARM_CONTROL_POINTS)=0;
                
                controlPointsJacobian(1,7,i+ARM_CONTROL_POINTS)=0;
                controlPointsJacobian(2,7,i+ARM_CONTROL_POINTS)=0;
                controlPointsJacobian(3,7,i+ARM_CONTROL_POINTS)=0;
            end
        end
        
        function [Ori,Ori_z]=computeEnd_effectorOrientation(obj)
            A=obj.transMat(obj.q(1),0,1,0,obj.d1);
            A=A*obj.transMat(obj.q(2),0,-1,0,0);
            A=A*obj.transMat(obj.q(3),0,-1,0,obj.d3);
            A=A*obj.transMat(obj.q(4),0,1,0,0);
            A=A*obj.transMat(obj.q(5),0,1,0,obj.d5);
            A=A*obj.transMat(obj.q(6),0,-1,0,0);
            A=A*obj.transMat(0,1,0,0,obj.d7);
            Ori=A(1:3,1:3);
            Ori_z=A(1:3,3);
        end
        
        function pdot=computeEnd_effectorPosVelocity(obj)
            J=obj.computeJacobianPos();
            pdot=J*obj.qdot;
        end
        
        function pdot=computeEnd_effectorOriVelocity(obj,zd)
            [~,ze]=obj.computeEnd_effectorOrientation;
            [~,Jori]=obj.computeJacobianOri(zd,ze);
            pdot=Jori*obj.qdot;
        end
        
        function A=transMat(~,q,ca,sa,a,d)
            sq=sin(q);
            cq=cos(q);
            A=sparse([cq, -sq*ca, sq*sa, a*cq; sq, cq*ca, -cq*sa, a*sq; 0, sa, ca, d; 0 0 0 1]);
        end
        
        function plotarm(obj)
            p0=[0;0;0;1];
            A1=obj.transMat(obj.q(1),0,1,0,obj.d1);
            A2=A1*obj.transMat(obj.q(2),0,-1,0,0);
            A3=A2*obj.transMat(obj.q(3),0,-1,0,obj.d3);
            A4=A3*obj.transMat(obj.q(4),0,1,0,0);
            A5=A4*obj.transMat(obj.q(5),0,1,0,obj.d5);
            A6=A5*obj.transMat(obj.q(6),0,-1,0,0);
            A7=A6*obj.transMat(0,1,0,0,obj.d7);
            p1=A1*p0;
            p2=A2*p0;
            p3=A3*p0;
            p4=A4*p0;
            p5=A5*p0;
            p6=A6*p0;
            p7=A7*p0;
            plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],'ok');
            plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],'Color','r');
            hold on
            axis([-1 1 -1 1 -1 1])
            plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'ok');
            plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'r');
            hold on
            plot3([p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)],'ok');
            plot3([p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)],'r');
             hold on
            plot3([p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)],'ok');
             plot3([p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)],'r');
             hold on
            plot3([p4(1),p5(1)],[p4(2),p5(2)],[p4(3),p5(3)],'ok');
             plot3([p4(1),p5(1)],[p4(2),p5(2)],[p4(3),p5(3)],'r');
             hold on
            plot3([p5(1),p6(1)],[p5(2),p6(2)],[p5(3),p6(3)],'ok');
            plot3([p5(1),p6(1)],[p5(2),p6(2)],[p5(3),p6(3)],'r');
             hold on
            plot3([p6(1),p7(1)],[p6(2),p7(2)],[p6(3),p7(3)],'ok');
            plot3([p6(1),p7(1)],[p6(2),p7(2)],[p6(3),p7(3)],'r');
             hold on
            %axis square;
            %axis([-3 3 -3 3]);
        end
    end
end