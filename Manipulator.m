classdef Manipulator < RigidRobot

    properties
        DHParameters   % DH
    end
    
    methods
        function obj = Manipulator(DH,mass,centerOfMass,inertia,rho,gravity_vector,isSym)          
            
            % state
            N = size(DH,1);
            
            if isSym
                [q,qd,qdd] = Manipulator.symbolicState(N);
            else
                q = zeros(N,1);
                qd = zeros(N,1);
                qdd = zeros(N,1);
            end
            qsym  = Manipulator.symbolicState(N); % for jacobian/ should also appear in DH
            
            state.q = q;
            state.qd = qd;
            state.qdd = qdd;
            
%             obj.State = state;  ******
            
            % frames for serial manipulator
            tree = LinkTree;
            parent = 'base';
            
            for i = 1:N
                
                currentLink = sprintf('Link%d',i);
                
                % Transform for DH convention
                theta = DH(i,1);
                d = DH(i,2);
                a = DH(i,3);
                alpha = DH(i,4);
                T = Manipulator.DHtransform(theta,d,a,alpha,isSym);
                
                % single state
                tempState.q = state.q(i);
                tempState.qd = state.qd(i);
                tempState.qdd = state.qdd(i);
                
                % body
                % undetermined pose and jacobian
                pose = {};
                J = {};
                body = RigidBody(currentLink,mass(i),centerOfMass(:,i),inertia(:,:,i),pose,J);
                
                % joint
                jointParam.stiffness = 0; % for now
                jointParam.damping = 0; % for now

                joint = Joint(rho(i),parent,currentLink,tempState,jointParam);
                
                % add link
                tree.addLink(parent,body,joint,T);
                
                parent = currentLink;
            end
            obj = obj@RigidRobot(state,KinematicChain(tree),gravity_vector,isSym);
            % parameter
            obj.DHParameters = DH;
            % update pose and jacobian of each link (sub function ?)
            listID = obj.KinematicChain.Tree.getListLinkID;
            numLink = numel(listID);
            
            for i = 1:numLink-1
                link = obj.KinematicChain.Tree.getChildByID(listID{i+1});
                % pose
                position = ForwardKinematicsCOM(obj,i); % to base
                rotation = Rinertial(obj,i); % to base
                pose = Pose(rotation,position); 
                % jacobian
                J_v = linearJacobianCOM(obj,i);
                J_w = angularJacobian(obj,i);
                J = Jacobian(J_v,J_w,qsym);
                % update
                link.Body.Pose = pose;
                link.Body.Jacobian = J;
                
            end
            
            dynamicParam.Inertia = obj.inertiaMatrix;
            dynamicParam.Coriolis = obj.coriolisMatrix;
            dynamicParam.Gravity = obj.gravityMatrix;
            obj.DynamicParameters = dynamicParam;      
            
        end
        function h = plotRobot(obj,varargin)
            if isempty(varargin)
                L = 0.1;
            else
                L = varargin{1};
            end
            if ~obj.Environment.IsSym
                
                % get parameters
                dh_table = obj.DHParameters;
                N = obj.DegreeOfFreedom; % might change this
                [q,~,~] = obj.symbolicState(N);
                T_intermediate = zeros(4,4,N);
                for i = 1:N
                    T = eye(4);
                    for j=1:i
                        theta = dh_table(j,1);
                        d = dh_table(j,2);
                        a = dh_table(j,3);
                        alpha = dh_table(j,4);
                        T = T*obj.DHtransform(theta,d,a,alpha);
                    end
                    T = subs(T,q,obj.State.q);
                    T_intermediate(:,:,i) = T;
                end

                x_previous = 0;
                y_previous = 0;
                z_previous = 0;
                h = axes;
                hold on;
                plot3(x_previous,y_previous,z_previous,'ro','LineWidth',3)
                drawFrame(eye(4),L);
                for i = 1:N,
                    T_to_joint = eye(4);
                    for j = 1:i,
                        T_to_joint = T_to_joint*T_intermediate(:,:,j);
                    end
                    
                    x_next = T_to_joint(1,4);
                    y_next = T_to_joint(2,4);
                    z_next = T_to_joint(3,4);
                    
                    
                    plot3([x_previous x_next],[y_previous y_next],[z_previous z_next],'k','LineWidth',3);
                    plot3(x_next,y_next,z_next,'ro','LineWidth',3)
                    
                    
                    drawFrame(T_to_joint,L);
                    
                    x_previous = x_next;
                    y_previous = y_next;
                    z_previous = z_next;
                    
                end
                axis equal;
                view(3)
            end
            function drawFrame(T,L)
                x = T(1,4);
                y = T(2,4);
                z = T(3,4);
                
                p = [x y z]';
                
                normal = T(1:3,1);
                sliding = T(1:3,2);
                approach = T(1:3,3);
                
                unit_x = p + L*normal;
                unit_y = p + L*sliding;
                unit_z = p + L*approach;
                
                plot3([p(1) unit_x(1)],[p(2) unit_x(2)],[p(3) unit_x(3)],'r','LineWidth',3);
                plot3([p(1) unit_y(1)],[p(2) unit_y(2)],[p(3) unit_y(3)],'g','LineWidth',3);
                plot3([p(1) unit_z(1)],[p(2) unit_z(2)],[p(3) unit_z(3)],'b','LineWidth',3);
                
            end

end
    end
    methods (Access = private)
        %% Helpers
        function p = ForwardKinematicsCOM(obj,i)
            % return symbolic position of center of mass of ith link
            
            % get parameters
            dh_table = obj.DHParameters;
            
            N = obj.DegreeOfFreedom;
            
            if obj.Environment.IsSym
                cm = sym(zeros(3,N));
            else
                cm = zeros(3,N);
            end
            
            listID = obj.KinematicChain.Tree.getListLinkID;
            numLink = numel(listID);
            for idx = 1:numLink-1, % not including base/ ground
                link = obj.KinematicChain.Tree.getChildByID(listID{idx+1});
                cm(:,idx) = link.Body.CenterOfMass;
            end
            
            T = sym(eye(4));
            
            % TO DO:
            % change to while loop (loop the frame back to the base)
            for j=1:i
                theta = dh_table(j,1);
                d = dh_table(j,2);
                a = dh_table(j,3);
                alpha = dh_table(j,4);
                T = T*obj.DHtransform(theta,d,a,alpha);
            end
            
            
            T = T*obj.transl(cm(:,i),'all');
            
            p = T(1:3,4);
            p = simplify(p);
        end
        function J_v = linearJacobianCOM(obj,i)
            %return 3xn linear Jacobian matrix at the center of mass of ith link
            
            % TO DO:
            % in the future, should take an ID of rigid body, and fixed
            % transformation and return linear Jacobian to the point
            N = obj.DegreeOfFreedom;
            [q,~,~] = obj.symbolicState(N);
            p = obj.ForwardKinematicsCOM(i);
            J_v = simplify(jacobian(p,q));
            
        end     
        function J_w = angularJacobian(obj,i)
            %return 3xn angular Jacobian matrix at the center of mass of ith link
            
            % get parameters
            dh_table = obj.DHParameters;
            N = obj.DegreeOfFreedom;
            rho = zeros(1,N);
            
            listID = obj.KinematicChain.Tree.getListLinkID;
            numLink = numel(listID);
            for idx = 1:numLink-1, % not including base/ ground
                link = obj.KinematicChain.Tree.getChildByID(listID{idx+1});
                rho(:,idx) = link.Joint.Type;
            end                       
            
            J_w = sym(zeros(3,N));
            
            
            for k = 1:i
                T = sym(eye(4));
                
                for j = 1:i
                    theta = dh_table(j,1);
                    d = dh_table(j,2);
                    a = dh_table(j,3);
                    alpha = dh_table(j,4);
                    T = T*obj.DHtransform(theta,d,a,alpha);
                end
                J_w(:,k) = rho(k)*T(1:3,3);                
                J_w(:,k) = simplify(J_w(:,k)); 
            end
            
        end
        function R = Rinertial(obj,i,varargin)
            %oriantation transformation from the ith body attached fram and the
            %inertial (global) frame
            
            if ~isempty(varargin)
                isSymExplicit = strcmp('symbolic',varargin{1});
            else
                isSymExplicit = false;
            end            
            % get parameters
            dh_table = obj.DHParameters;
            N = obj.DegreeOfFreedom;
            if obj.Environment.IsSym || isSymExplicit
                cm = sym(zeros(3,N));
            else
                cm = zeros(3,N);
            end
            
            listID = obj.KinematicChain.Tree.getListLinkID;
            numLink = numel(listID);
            for idx = 1:numLink-1, % not including base/ ground
                link = obj.KinematicChain.Tree.getChildByID(listID{idx+1});
                cm(:,idx) = link.Body.CenterOfMass;
            end
            
            % transform
            
            if obj.Environment.IsSym || isSymExplicit
                T = sym(eye(4));
            else
                T = eye(4);
            end
            
            for j = 1:i
                theta = dh_table(j,1);
                d = dh_table(j,2);
                a = dh_table(j,3);
                alpha = dh_table(j,4);
                T = T*obj.DHtransform(theta,d,a,alpha);
                
            end
            T = T*obj.transl(cm(:,i),'all');
            R = T(1:3,1:3);
            
        end
    end
    methods (Static)
        function T = DHtransform(theta,d,a,alpha,varargin)
            %take in four standard DH parameters between two consecutive frames and
            %return 4x4 homogeneous intermediate transformation matrix between
            %the links
            if ~isempty(varargin)
                isSym = varargin{1};
            else
                isSym = 0;
            end
            T = Manipulator.rot(theta,'z',isSym)*...
                Manipulator.transl(d,'z',isSym)*...
                Manipulator.transl(a,'x',isSym)*...
                Manipulator.rot(alpha,'x',isSym);
        end
        function T = rot(theta,ax,varargin)
            % take value in radian and standard axis of rotation (x,y,z)
            % and return 4x4 homogeneous transformation matrix of this pure rotation
            
            if ~isempty(varargin)
                isSym = varargin{1};
            else
                isSym = 0;
            end
            
            if isSym
                T = sym(zeros(4));
                p = sym(zeros(3,1));
            else
                T = zeros(4);
                p = zeros(3,1);
            end
            
            if lower(ax) == 'x'
                R = [1          0               0;...
                    0      cos(theta)      -sin(theta);...
                    0      sin(theta)      cos(theta)];
            elseif lower(ax) == 'y'
                R = [cos(theta) 0   sin(theta);...
                    0          1       0     ;...
                    -sin(theta) 0  cos(theta)];
            elseif lower(ax) == 'z'
                R = [cos(theta)      -sin(theta)       0;...
                    sin(theta)      cos(theta)        0;...
                    0               0              1];
            else
                error('not a standard axis')
                
            end
            
            T = [R p;...
                zeros(1,3) 1];
        end      
        function T = transl(d,ax,varargin)
            % take value in radian and standard axis of rotation (x,y,z)
            % and return 4x4 homogeneous transformation matrix of this pure rotation
            
            if ~isempty(varargin)
                isSym = 1;
            else
                isSym = 0;
            end
            
            if isSym
                T = sym(zeros(4));
                R = sym(eye(3));
                p = sym(zeros(3,1));
            else
                T = zeros(4);
                R = eye(3);
                p = zeros(3,1);
            end
            
            if lower(ax) == 'x'
                p(1) = d;
            elseif lower(ax) == 'y'
                p(2) = d;
            elseif lower(ax) == 'z'
                p(3) = d;
            elseif strcmp(ax,'all')
                p = reshape(d,3,1);
            else
                error('not a standard axis')
            end
            
            T = [R p;...
                zeros(1,3) 1];
        end
        function [q,qd,qdd] = symbolicState(N)
            % TO DO :
            % http://www.mathworks.com/matlabcentral/answers/
            % 242-how-to-generate-symbolic-variables-dynamically-at-run-time

            q = sym('q',[N 1]);
            qd = sym('qd',[N 1]);
            qdd = sym('qdd',[N 1]);
            
            assume(q,'real');
            assume(qd,'real');
            assume(qdd,'real');
            
        end
    end
end
