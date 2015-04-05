classdef Manipulator
    % call the following for example
    % TwoDOFSpec;
    % TwoDOF = Robot(DH_table,m,cm,I,rho,gravity_vector,1);
    % TwoDOF.State.Matrix.Inertia
    % TwoDOF.State.Matrix.Coriolis
    % TwoDOF.State.Matrix.Gravity
    properties
        Environment % Gravity , IsSym
        DHParameters   % DH
        DynamicParameters % Matrices -> D,C,G,B,K , TO DO : should return mass, and com of entire
        LinkList = {};
        State       % Joint -> (q,qd,qdd)
    end
    
    methods
        function obj = Manipulator(DH,mass,centerOfMass,inertia,rho,gravity_vector,isSym)
            % environment
            env.Gravity = gravity_vector;
            env.IsSym = isSym;
            obj.Environment = env;
            
            % parameter
            obj.DHParameters = DH;
            
            % state
            N = obj.degreeOfFreedom;
            
            if isSym
                [q,qd,qdd] = obj.symbolicState;
            else
                q = zeros(N,1);
                qd = zeros(N,1);
                qdd = zeros(N,1);
            end
            
            state.q = q;
            state.qd = qd;
            state.qdd = qdd;
            
            obj.State = state;
            
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
                T = DHtransform(obj,theta,d,a,alpha);
                
                % single state
                tempState.q = state.q(i);
                tempState.qd = state.qd(i);
                tempState.qdd = state.qdd(i);
                
                % body
                body = RigidBody(currentLink,mass(i),centerOfMass(:,i),inertia(:,:,i));
                
                % joint
                jointParam.stiffness = 0; % for now
                jointParam.damping = 0; % for now

                joint = Joint(rho(i),parent,currentLink,tempState,jointParam);
                
                % add link
                tree.addLink(parent,body,joint,T);
                
                parent = currentLink;
            end
            obj.LinkList = tree;

            
            dynamicParam.Inertia = obj.inertiaMatrix;
            dynamicParam.Coriolis = obj.coriolisMatrix;
            dynamicParam.Gravity = obj.gravityMatrix;
            obj.DynamicParameters = dynamicParam;
                            
                
            
        end
        function N = degreeOfFreedom(obj)
            N = size(obj.DHParameters,1);
        end
        function D = inertiaMatrix(obj)
            %return nxn generalized inertia matrix
            
            N = obj.degreeOfFreedom;
            
            % get parameters
            if obj.Environment.IsSym
                m = sym(zeros(1,N));
                I = sym(zeros(3,3,N));
            else
                m = zeros(1,N);
                I = zeros(3,3,N);
            end
            
            listID = obj.LinkList.getListLinkID;
            numLink = numel(listID);
            for i = 2:numLink, % not including base/ ground
                link = obj.LinkList.getChildByID(listID{i});
                m(i-1) = link.Body.Mass;
                I(:,:,i-1) = link.Body.Inertia;
            end
            
            
            if obj.Environment.IsSym
                D = sym(zeros(N));
            else
                D = zeros(N);
            end
            
            % for each rigid body in the system
            
            for i = 1:N
                J_v = obj.linearJacobianCOM(i);
                J_w = obj.angularJacobian(i);
                R = obj.Rinertial(i);
                linear_term = m(i)*(J_v')*J_v;
                angular_term = J_w'*R*I(:,:,i)*R'*J_w;
                D = D + linear_term + angular_term;
            end
            if obj.Environment.IsSym
                D = simplify(D);
            else
                D = eval(D);
            end
        end
        function C = coriolisMatrix(obj)
            %return nxn generalized inertia Coriolis matrix
            
            N = obj.degreeOfFreedom;
            [q,qd,~] = obj.symbolicState;
            
            D = symbolicInertiaMatrix(obj);
            
            C = sym(zeros(N));
            c_kj = sym(zeros(N));
            
            for j = 1:N
                
                for k = 1:N
                    c_kj(k,j) = 0;
                    for i = 1:N
                        c_ijk = 1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)));
                        c_kj(k,j) = c_kj(k,j)+c_ijk*qd(i);
                    end
                    C(k,j) = c_kj(k,j);
                end
                
            end
            
            % substitute for numeric values
            if ~obj.Environment.IsSym
                
                C = subs(C,q,obj.State.q);
                C = subs(C,qd,obj.State.qd);
                C = eval(C);
            else
                C = simplify(C);
            end
            
            % nested help function
            function D = symbolicInertiaMatrix(obj)
                %return nxn symbolic generalized inertia matrix
                n = obj.degreeOfFreedom;
                % get parameters
                m = sym(zeros(1,n));
                I = sym(zeros(3,3,n));
                
                listID = obj.LinkList.getListLinkID;
                numLink = numel(listID);
                for idx = 2:numLink, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{idx});
                    m(idx-1) = link.Body.Mass;
                    I(:,:,idx-1) = link.Body.Inertia;
                end
                
                D = sym(zeros(n));
                for idx = 1:n
                    J_v = symbolicLinearJacobian(obj,idx);
                    J_w = symbolicAngularJacobian(obj,idx);
                    R = symbolicRinertial(obj,idx);
                    linear_term = m(idx)*(J_v')*J_v;
                    angular_term = J_w'*R*I(:,:,idx)*R'*J_w;
                    D = D + linear_term + angular_term;
                end
                
                D = simplify(D);
                
            end
            function J_v = symbolicLinearJacobian(obj,i)
                %return 3xn linear Jacobian matrix at the center of mass of ith link
                
                [qv,~,~] = obj.symbolicState;
                p = obj.ForwardKinematicsCOM(i);
                J_v = simplify(jacobian(p,qv));
            end
            function J_w = symbolicAngularJacobian(obj,i)
                %return 3xn angular Jacobian matrix at the center of mass of ith link
                
                % get parameters
                dh_table = obj.DHParameters;
                n = obj.degreeOfFreedom;
                rho = zeros(1,n);
                listID = obj.LinkList.getListLinkID;
                numLink = numel(listID);
                for idx = 2:numLink, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{idx});
                    rho(:,idx-1) = link.Joint.Type;
                end
                                  
                J_w = sym(zeros(3,n));
                
                for kdx = 1:i
                    
                    T = sym(eye(4));
                    
                    for jdx = 1:i
                        theta = dh_table(jdx,1);
                        d = dh_table(jdx,2);
                        a = dh_table(jdx,3);
                        alpha = dh_table(jdx,4);
                        T = T*obj.DHtransform(theta,d,a,alpha);
                    end
                    J_w(:,kdx) = rho(kdx)*T(1:3,3);
                    J_w(:,kdx) = simplify(J_w(:,kdx));
                end
            end
            function R = symbolicRinertial(obj,i)
                %oriantation transformation from the ith body attached fram and the
                %inertial (global) frame
                
                % get parameters
                dh_table = obj.DHParameters;
                
                
                n = obj.degreeOfFreedom;
                cm = sym(zeros(3,n));
                listID = obj.LinkList.getListLinkID;
                numLink = numel(listID);
                for idx = 2:numLink, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{idx});
                    cm(:,idx-1) = link.Body.CenterOfMass;
                end
                
                T = sym(eye(4));
                
                for jdx = 1:i
                    theta = dh_table(jdx,1);
                    d = dh_table(jdx,2);
                    a = dh_table(jdx,3);
                    alpha = dh_table(jdx,4);
                    T = T*obj.DHtransform(theta,d,a,alpha);
                    
                end
                T = T*obj.transl(cm(:,i),'all'); % x,y, and z
                R = T(1:3,1:3);
                
            end
        end
        function G = gravityMatrix(obj)
            
            [q,~,~] = obj.symbolicState;
            P = symbolicPotentialEnergy(obj);
            G = jacobian(P,q)';
            % substitute for numeric values
            if ~obj.Environment.IsSym
                G = subs(G,q,obj.State.q);
                G = eval(G);
            end
            function P = symbolicPotentialEnergy(obj)
                % return symbolic potential energy of the robot
                
                % get parameters
                gravity_vector = obj.Environment.Gravity;
                
                N = obj.degreeOfFreedom;
                m = sym(zeros(1,N));
                
                listID = obj.LinkList.getListLinkID;
                numLink = numel(listID);
                for i = 2:numLink, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{i});
                    m(i-1) = link.Body.Mass;
                end
                
                P = sym(0);
                for i = 1:N
                    rc = obj.ForwardKinematicsCOM(i);
                    P = P + m(i)*gravity_vector'*rc;
                end
            end
        end
        function obj = updateState(obj,q,qd,qdd)
            % update D,C,G matrices with corresponding numeric values of
            % q,qd, and qdd
            
            N = obj.degreeOfFreedom;
            state.q = reshape(q,N,1);
            state.qd = reshape(qd,N,1);
            state.qdd = reshape(qdd,N,1);
            
            obj.State = state;
            for i = 1:N,
                joint = obj.JointList{i};
                joint.State.q = state.q(i); % TO DO :have to change the name
                joint.State.qd = state.qd(i); % TO DO :have to change the name
                joint.State.qdd = state.qdd(i); % TO DO :have to change the name
                
                obj.JointList{i} = joint;
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
                [q,~,~] = obj.symbolicState;
                N = obj.degreeOfFreedom;
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
            
            N = obj.degreeOfFreedom;
            
            if obj.Environment.IsSym
                cm = sym(zeros(3,N));
            else
                cm = zeros(3,N);
            end
            
            listID = obj.LinkList.getListLinkID;
            numLink = numel(listID);
            for idx = 2:numLink, % not including base/ ground
                link = obj.LinkList.getChildByID(listID{idx});
                cm(:,idx-1) = link.Body.CenterOfMass;
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
            
            [q,~,~] = obj.symbolicState;
            p = obj.ForwardKinematicsCOM(i);
            J_v = simplify(jacobian(p,q));
            
            if ~obj.Environment.IsSym
                J_v = subs(J_v,q,obj.State.q);
            end
            
        end
        function J_w = angularJacobian(obj,i)
            %return 3xn angular Jacobian matrix at the center of mass of ith link
            
            % get parameters
            dh_table = obj.DHParameters;
            N = obj.degreeOfFreedom;
            rho = zeros(1,N);
            
            listID = obj.LinkList.getListLinkID;
            numLink = numel(listID);
            for idx = 2:numLink, % not including base/ ground
                link = obj.LinkList.getChildByID(listID{idx});
                rho(:,idx-1) = link.Joint.Type;
            end                       
            
            if obj.Environment.IsSym
                J_w = sym(zeros(3,N));
            else
                J_w = zeros(3,N);
            end
            
            for k = 1:i
                
                if obj.Environment.IsSym
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
                J_w(:,k) = rho(k)*T(1:3,3);
                if obj.Environment.IsSym
                    J_w(:,k) = simplify(J_w(:,k));
                end
                
            end
            
        end
        function R = Rinertial(obj,i)
            %oriantation transformation from the ith body attached fram and the
            %inertial (global) frame
            
            % get parameters
            dh_table = obj.DHParameters;
            N = obj.degreeOfFreedom;
            if obj.Environment.IsSym
                cm = sym(zeros(3,N));
            else
                cm = zeros(3,N);
            end
            
            listID = obj.LinkList.getListLinkID;
            numLink = numel(listID);
            for idx = 2:numLink, % not including base/ ground
                link = obj.LinkList.getChildByID(listID{idx});
                cm(:,idx-1) = link.Body.CenterOfMass;
            end
            
            % transform
            
            if obj.Environment.IsSym
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
        function [q,qd,qdd] = symbolicState(obj)
            % TO DO :
            % http://www.mathworks.com/matlabcentral/answers/
            % 242-how-to-generate-symbolic-variables-dynamically-at-run-time
            N = obj.degreeOfFreedom;

            q = sym('q',[N 1]);
            qd = sym('qd',[N 1]);
            qdd = sym('qdd',[N 1]);
            
            assume(q,'real');
            assume(qd,'real');
            assume(qdd,'real');
            
        end
        function T = DHtransform(obj,theta,d,a,alpha)
            %take in four standard DH parameters between two consecutive frames and
            %return 4x4 homogeneous intermediate transformation matrix between
            %the links
            T = obj.rot(theta,'z')*...
                obj.transl(d,'z')*...
                obj.transl(a,'x')*...
                obj.rot(alpha,'x');
        end
        function T = rot(obj,theta,ax)
            % take value in radian and standard axis of rotation (x,y,z)
            % and return 4x4 homogeneous transformation matrix of this pure rotation
            
            if obj.Environment.IsSym
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
        function T = transl(obj,d,ax)
            % take value in radian and standard axis of rotation (x,y,z)
            % and return 4x4 homogeneous transformation matrix of this pure rotation
            if obj.Environment.IsSym
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
    end
end
