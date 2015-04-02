classdef Frame < handle
    %BodyNode node of tree
    
    properties
        Children = {};
        Parent = {};
        Transform;
    end
    
    methods
        %Construct node with transform FROM parent
        function obj = Frame(trans,parent)
            obj.Transform = trans;
            if nargin == 2
                obj.Parent = parent;
                obj.Parent.addChild(obj);
            end                
        end
        
        function addChild(obj,child)
            obj.Children{end+1} = child;
        end
        
        function trans = transformFromAncestor(obj,ancestor)
            trans = obj.Transform;
            next_node = obj.Parent;
            while next_node~=ancestor;
                trans = next_node.Transform*trans; %figure out direction
                next_node = next_node.Parent;
            end
        end
        
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

