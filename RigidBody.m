classdef RigidBody
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ID;
        Mass;
        CenterOfMass; 
        Inertia; % about the center of mass alighed with the local frame
    end
    
    methods
        function obj = RigidBody(ID,m,cm,I)
            obj.ID = ID;
            obj.Mass = m;
            obj.CenterOfMass = cm;
            obj.Inertia = I;
        end
        
    end
    
end

