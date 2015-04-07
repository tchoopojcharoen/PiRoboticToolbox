classdef Joint < handle
    
    properties
        Type;     % revolute/ prismatic
        DegreeOfFreedom;
        BaseLink; % name / ID of the base link
        FollowerLink; % name / ID of the follower link
%         Axis % axis of actuation
        State;
        Parameter;
        % position
        % axis of actuation (rotation or translation)
    end
    
    methods
        function obj = Joint(type,DOF,base,follower,state,param)
            obj.Type = type;
            obj.DegreeOfFreedom = DOF;
            obj.BaseLink = base;
            obj.FollowerLink = follower;
%             obj.Axis = axis;
            obj.State = state;
            obj.Parameter = param;
        end
        
        function obj = updateJoint(obj,state)
%             obj.Axis = axis;
            obj.State = state;
        end
        
    end
    
end

