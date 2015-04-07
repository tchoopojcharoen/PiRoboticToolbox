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
    end
    
end

