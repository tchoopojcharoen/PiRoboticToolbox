classdef KinematicChain < handle
    
    properties
        Tree % for wrapper
    end
    
    methods
        function obj = KinematicChain(tree)
            % constructor
            obj.Tree = tree;
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

