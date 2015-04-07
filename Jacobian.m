classdef Jacobian < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % jacobianis always symbolic in term of q
        Linear
        Angular
        Total
        q % independent variable q for symbolic differentiation
    end
    
    methods
        function obj = Jacobian(J_v,J_w,q)
            obj.Linear = J_v;
            obj.Angular = J_w;
            obj.Total = [J_v ; J_w];
            obj.q = q;
            
        end
        function numeric_jacobian = eval(obj,q_value)
            % evaluate jacobian at given configuration
            % q_value must have same dimension with obj.q
            [m,n] = size(obj.q);
            numeric_jacobian.Linear = subs(obj.Linear,q,reshape(q_value,m,n));
            numeric_jacobian.Angular = subs(obj.Angular,q,reshape(q_value,m,n));
            numeric_jacobian.Total = [numeric_jacobian.Linear;numeric_jacobian.Total];
            
        end
    end
    
end

