classdef RigidBody < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ID;
        Mass;
        CenterOfMass; % from the local frame
        Inertia; % about the center of mass alighed with the local frame
        Pose; % R, p , T = [R p ; 0 0 0 1] wiht respect to base frame
        Jacobian; % linear, angular from the local frame to base frame
    end
    
    methods
        function obj = RigidBody(ID,m,cm,I,pose,jacobian)
            obj.ID = ID;
            obj.Mass = m;
            obj.CenterOfMass = cm;
            obj.Inertia = I;
            obj.Pose = pose;
            obj.Jacobian = jacobian;
        end
        
        function D = generalizedInertiaMatrix(obj)
            % calculaute generalized inertia
            m = obj.Mass;
            I = obj.Inertia;
            pose = obj.Pose;
            R = pose.Rotation;
            J = obj.Jacobian;
            J_v = J.Linear;
            J_w = J.Angular;
            
            % calculate D for each link
            
            linear_term = linearGeneralizedInertia(m,J_v);
            angular_term = angularGeneralizedInertia(I,R,J_w);
            D = linear_term + angular_term;
            
            function linear_term = linearGeneralizedInertia(m,J_v)
                linear_term = m*(J_v')*J_v;
            end
            function angular_term = angularGeneralizedInertia(I,R,J_w)
                angular_term = J_w'*R*I*R'*J_w;
            end
        end
        
    end
    
end

