classdef FrameTree < handle
    
    properties
        NodeList = {};
        NodeLabel = [];
    end
    
    methods
        function obj = FrameTree()
            base = Frame(eye(4));
            obj.NodeList{end+1} = base;
            obj.NodeLabel(end+1) = 1;
        end
        
        function [obj,newID] = addFrame(obj,parent,T)
            i = 1;
            while obj.NodeLabel(i)~=parent
                i = i+1;
            end
            newNode = Frame(T,obj.NodeList{i});
            newID = obj.NodeLabel(end)+1;
            
            obj.NodeList{end+1} = newNode;
            obj.NodeLabel(end+1) = newID;
        end
        
        function T = forwardKinematic(obj,baseID,targetID)
            i = 1;
            while obj.NodeLabel(i)~=baseID
                i = i+1;
            end
            
            j = 1;
            while obj.NodeLabel(j)~=targetID
                j = j+1;
            end

            T = transformTo(obj.NodeList{i},obj.NodeList{j});
        end
    end
    
end

