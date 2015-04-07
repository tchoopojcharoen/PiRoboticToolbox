classdef Pose < handle
    % Pose
    
    properties
        Rotation;
        Translation;
        Transformation;
    end
    
    methods
        function obj = Pose(R,p) % might be euler angle or screw in the future
            obj.Rotation = R;
            obj.Translation = reshape(p,3,1);
            obj.Transformation = [R p ; 0 0 0 1];
        end
        
        function numeric_pose = eval(obj,q_value)
            % evaluate jacobian at given configuration
            % q_value must have same dimension with obj.q
            [m,n] = size(obj.q);
            numeric_pose.Rotation = subs(obj.Rotation,q,reshape(q_value,m,n));
            numeric_pose.Translation = subs(obj.Translation,q,reshape(q_value,m,n));
            numeric_pose.Total = [numeric_pose.Rotation numeric_pose.Translation ; 0 0 0 1];
            
        end
    end
    
end

