classdef KinematicChain < handle
    
    properties
        Tree % for wrapper
        BodyID;
    end
    
    methods
        function obj = KinematicChain(tree)
            % constructor
            obj.Tree = tree;
            obj.BodyID = getListRigidBodyID(obj);
        end
        function DOF = degreeOfFreedom(obj)
            listBodyID = getListRigidBodyID(obj);
            listJointID = getListJointID(obj);
            numBody = numel(listBodyID);
            numJoint = numel(listJointID);
            listJointDOF = getListJointDOF(obj);
            DOF = 6*(numBody-1-numJoint)+sum(listJointDOF);
            
        end
        function T = forwardKinematic(obj,ID,varargin)
            % chain.forwardKinematic(ID) given ID of a rigid body, return 
            % the position of center of mass and orientation of a rigid body 
            % (Pose object) with specified ID w.r.t ground
            % chain.forwardKinematic(ID,T) return Pose of a frame that
            % attach to a rigid with given ID. The transformation from the
            % center of mass and the frame is T
            
            % ex: given position vectorof a point p w.r.t the attached frame, the
            % position of the point w.r.t thet center of mass can be
            % calculuated by  T*p
            if isempty(varargin)
                T_atttached = eye(4);
            else
                T_atttached = varargin{1};
            end
            
            % TO DO: implement using direct acyclic graph
            
            cm = 1;
        end
        function listBodyID = getListRigidBodyID(obj,varargin)
            listBodyID = getListLinkID(obj.Tree,varargin);
        end
        function listJointID = getListJointID(obj,varargin)
            % for now
            % need to have separate method to find list of joint ID
            % decouple from Link
            listJointID = getListLinkID(obj.Tree,varargin);
            listJointID = listJointID(2:end);
        end
        function listJointDOF = getListJointDOF(obj,varargin)
            % for now 
            listJointID = getListLinkID(obj.Tree,varargin);
            numJoint = numel(listJointID);
            listJointDOF = zeros(1,numJoint);
            % for now, use link
            for i = 1:numJoint-1
                 joint = getJointFromID(obj,listJointID{i+1});
                 listJointDOF(i) = joint.DegreeOfFreedom;
            end
        end
        
        function body = getBodyFromID(obj,ID)
            link = getChildByID(obj.Tree,ID);
            body = link.Body;
        end
        
        function joint = getJointFromID(obj,ID)
            link = getChildByID(obj.Tree,ID);
            joint = link.Joint;
        end
    end
    
end

