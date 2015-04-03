classdef LinkTree < handle
    properties
        ID;
        Body;
        Joint;
        Transform;
        JointParams;
        Parent;
        Children = {};
    end
    
    methods
        % LinkTree('name') constructs empty linktree with no transform
        % LinkTree('name',[],tf) constructs empty linktree with transform
        % Otherwise, all arguments must be supplied as shown
        function obj = LinkTree(ID,parent,tf,mass,COM_tf,inertia,joint_params)
            obj.ID = ID;
            if nargin==1
                obj.Transform = eye(4);
                return
            end
            obj.Parent = parent;
            obj.Transform = tf;
            if nargin==3
                return
            end
            obj.Body = RigidBody(ID,mass,COM_tf,inertia);
            obj.JointParams = joint_params;
            obj.Parent = parent;
        end
        
        % Add new link to tree
        % Parent can be either an ID or a LinkTree
        function link = addLink(obj,ID,parent,tf,mass,COM_tf,inertia,joint_params)
            %get parent node
            if isequal(class(parent),'char')
                parent = obj.getChildByID(parent);
            end
            
            %Validate parent
            if isempty(parent)
                disp('Invalid parent');
                return;
            end
            
            link = LinkTree(ID,parent,tf,mass,COM_tf,inertia,joint_params);
            parent.addChild(link);            
        end       
        
        %Add child link
        function addChild(obj,child)
            obj.Children{end+1} = child;
        end
        
        %Find the link with a given id, return [] if not found
        function link = getChildByID(obj,ID)
            link = [];
            
            if isequal(ID,obj.ID)
                link = obj;
                return;
            end
            
            for k=1:length(obj.Children)
                link = obj.Children{k}.getChildByID(ID);
                if ~isempty(link)
                    return
                end
            end
        end
        
        %Transform between two links by ID
        function trans = transform(obj,from_ID,to_ID)
            from_link = obj.getChildByID(from_ID);
            to_link = obj.getChildByID(to_ID);
            trans = from_link.transformTo(to_link);
        end
        
        %Get transformation to this from ancestor
        function trans = transformFromAncestor(obj,ancestor)
            trans = obj.Transform;
            next_node = obj.Parent;
            while next_node~=ancestor;
                trans = next_node.Transform*trans; %figure out direction
                next_node = next_node.Parent;
            end
        end
        
        %Transform from this link to another
        function transform = transformTo(obj,target)
            if obj==target
                transform = eye(4);
                return
            end
            
            baseParents = {target};
            next_node = obj.Parent;
           
            while ~isempty(next_node)
                baseParents{end+1} = next_node;
                next_node = next_node.Parent;
            end
            
            next_node = target;
            
            while isempty(find([baseParents{:}] == next_node, 1))
                next_node = next_node.Parent;
            end
            
            common_ancestor = next_node;
            
            trans1 = transformFromAncestor(obj,common_ancestor);
            trans2 = transformFromAncestor(target,common_ancestor);
            
            transform = trans1\trans2; %equivalent to inv(trans1)*trans2
        end
    end
end