classdef Jacobian < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Linear
        Angular
        Total
    end
    
    methods
        function obj = Jacobian(J_v,J_w)
            obj.Linear = J_v;
            obj.Angular = J_w;
            obj.Total = [J_v ; J_w];
            
        end
    end
    
end

