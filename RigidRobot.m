classdef (Abstract) RigidRobot < handle
    
    properties
        Environment;  % Gravity , IsSym
        KinematicChain; % graph with body as a node and joint as an edge
        %TotalMass; % total mass of the entire robot
        State; % q, qd, qdd
    end
    
    methods
        function obj = RigidRobot(kinematicChain,gravity_vector,isSym)
            
            % environment
            env.Gravity = gravity_vector;
            env.IsSym = isSym;
            obj.Environment = env;
            
            obj.KinematicChain = kinematicChain;
        end
        function N = degreeOfFreedom(obj)
            
            % for now, use serial manipulator
            % use mobility equation inthe future
            listID = obj.KinematicChain.Tree.getListLinkID;
            N = numel(listID)-1;
        end
        function m = mass(obj)
            m = 0;
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            for i = 1:numel(ID)%each body i in the chain
                body = chain.getBodyFromID(ID{i});
                m = m + body.Mass;
            end
        end    
        function cm = centerOfMass(obj)
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            
            cm = zeros(3,1);
            m = 0;
            for i = 1:numel(ID)-1%each body i in the chain
                body = chain.getBodyFromID(ID{i+1});
                mass = body.Mass;
                com = body.CenterOfMass;
                m = m + mass;
                cm = cm + mass*com;
            end
            
            cm = cm/m;
            
        end      
        function T = kineticEnergy(obj)
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            T = 0;
            for i = 1:numel(ID)-1
                body = chain.getBodyFromID(ID{i+1});
                qd = obj.State.qd;
                D_i = body.generalizedInertiaMatrix;
                T_i = (1/2)*qd'*D_i*qd;
                T = T + T_i;
            end
        end        
        function V = potentialEnergy(obj) % gravity + spring
            % no spring for now
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            V = 0;
            for i = 1:numel(ID)-1
                body = chain.getBodyFromID(ID{i+1});
                m = body.Mass;
                gravity_vector = obj.Environment.Gravity;
                pose = body.Pose;
                y = pose.Translation(2);
                V_i = m*gravity_vector*y;
                V = V + V_i;
            end
        end
        function D = inertiaMatrix(obj)
            %return nxn generalized inertia matrix
            
            N = obj.degreeOfFreedom;
             
            if obj.Environment.IsSym
                D = sym(zeros(N));    
            else
                D = zeros(N);
            end
            
            listID = obj.KinematicChain.getListIDRigidBody;
            numLink = numel(listID);
            for i = 1:numLink-1, % not including base/ ground
                
                % get parameters
                link = obj.KinematicChain.getBodyFromID(listID{i+1});
                body = link.Body;
                D_i = body.generalizedInertiaMatrix;
                D = D + D_i;
                
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
                for idx = 1:numLink-1, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{idx+1});
                    m(idx) = link.Body.Mass;
                    I(:,:,idx) = link.Body.Inertia;
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
                for idx = 1:numLink-1, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{idx+1});
                    rho(:,idx) = link.Joint.Type;
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
                for idx = 1:numLink-1, % not including base/ ground
                    link = obj.LinkList.getChildByID(listID{idx+1});
                    cm(:,idx) = link.Body.CenterOfMass;
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
    end
    
end

