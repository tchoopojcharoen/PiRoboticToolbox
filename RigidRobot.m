classdef (Abstract) RigidRobot < handle
    
    properties
        Environment;  % Gravity , IsSym
        KinematicChain; % graph with body as a node and joint as an edge
        %TotalMass; % total mass of the entire robot
        State; % q, qd, qdd
    end
    
    methods
        function obj = RigidRobot(kinematicChain,gravity_vector,isSym)
            
            % environment
            env.Gravity = gravity_vector;
            env.IsSym = isSym;
            obj.Environment = env;
            
            obj.KinematicChain = kinematicChain;
        end
        
        function m = mass(obj)
            m = 0;
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            for i = 1:numel(ID)%each body i in the chain
                body = chain.getBodyFromID(ID{i});
                m = m + body.Mass;
            end
        end
        
        function cm = centerOfMass(obj)
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            
            cm = zeros(3,1);
            m = 0;
            for i = 1:numel(ID)%each body i in the chain
                body = chain.getBodyFromID(ID{i});
                mass = body.Mass;
                com = body.CenterOfMass;
                m = m + mass;
                cm = cm + mass*com;
            end
            
            cm = cm/m;
            
        end
        
        function T = kineticEnergy(obj)
        end
        
        function T = potentialEnergy(obj) % gravity + spring
        end
        
    end
    
end

