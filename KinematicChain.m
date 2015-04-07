classdef KinematicChain < handle
    
    properties
        Tree % for wrapper
        BodyID;
    end
    
    methods
        function obj = KinematicChain(tree)
            % constructor
            obj.Tree = tree;
            obj.BodyID = getListIDRigidBody(obj);
        end
        
        function listID = getListIDRigidBody(obj,varargin)
            listID = getListLinkID(obj.Tree,varargin);
        end
        
        function body = getBodyFromID(obj,ID)
            link = getChildByID(obj.Tree,ID);
            body = link.Body;
        end
        
    end
    
end

