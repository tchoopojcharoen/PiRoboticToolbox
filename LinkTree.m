%Need to fix transform function

classdef LinkTree < handle
    properties
        ID;
        Body;
        Joint;
        Transform;
        Parent;
        Children = {};
    end
    
    methods
        % LinkTree('name') constructs empty linktree with no transform
        % LinkTree('name',[],tf) constructs empty linktree with transform
        % Otherwise, all arguments must be supplied as shown
        function obj = LinkTree(parent,body,joint,T)
            
            if nargin==0
                obj.Transform = eye(4);
                obj.ID = 'base';
                return
            end
            obj.ID = body.ID;
            obj.Parent = parent;
            obj.Transform = T;
            if nargin==3
                return
            end
            obj.Body = body;
            obj.Joint = joint;
            obj.Parent = parent;
        end
        
        % Add new link to tree
        % Parent can be either an ID or a LinkTree
        function link = addLink(obj,parent,body,joint,T)
            %get parent node
            if isequal(class(parent),'char')
                parent = obj.getChildByID(parent);
            end
            
            %Validate parent
            if isempty(parent)
                error('Invalid parent');
            end
            
            link = LinkTree(parent,body,joint,T);
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
            trans = 1; %use eye(4) instead?
            next_node = obj;
            while next_node~=ancestor;
                trans = next_node.Transform*trans; 
                next_node = next_node.Parent;
            end
        end
        
        %Transform from this link to another
        function transform = transformTo(obj,target)            
            ancestors = {target};            
            next_node = obj;
           
            k=1;
            %while next_node not in ancestors
            while ~isempty(next_node) && isempty(find([ancestors{:}] == next_node, 1))
                ancestors{end+1} = next_node; %add node to end
                next_node = ancestors{end-k}.Parent;
                if isempty(next_node)
                    k=k-1;
                    next_node = ancestors{end}.Parent;
                end
            end
            
            if isempty(next_node)
                error('No transfomation found between frames')
            end
            
            common_ancestor = next_node;
            
            trans1 = obj.transformFromAncestor(common_ancestor);
            trans2 = target.transformFromAncestor(common_ancestor);
            
            transform = trans1\trans2; %equivalent to inv(trans1)*trans2
        end
    end
end