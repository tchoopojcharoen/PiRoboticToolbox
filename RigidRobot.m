classdef (Abstract) RigidRobot < handle
    
    properties
        DegreeOfFreedom;
        State; % q, qd, qdd
        KinematicChain; % graph with body as a node and joint as an edge
        %TotalMass; % total mass of the entire robot
        DynamicParameters;
        Environment;  % Gravity , IsSym
    end
    
    methods
        function obj = RigidRobot(state,kinematicChain,gravity_vector,isSym)
            
            % environment
            env.Gravity = gravity_vector;
            env.IsSym = isSym;
            obj.Environment = env;
            obj.State = state;
            obj.KinematicChain = kinematicChain;
            listID = obj.KinematicChain.Tree.getListLinkID;
            obj.DegreeOfFreedom = numel(listID)-1;
        end
        function m = mass(obj)
            m = 0;
            chain = obj.KinematicChain;
            ID = chain.getListIDRigidBody;
            for i = 1:numel(ID)-1%each body i in the chain
                body = chain.getBodyFromID(ID{i+1});
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
                T = body.Pose.Transformation;
                com_base = simplify([eye(3) zeros(3,1)]*T*[com;1]);
                m = m + mass;
                cm = cm + mass*com_base;
            end
            
            cm = simplify(cm/m);
            
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
                rc = pose.Translation;
                V_i = m*gravity_vector'*rc;
                V = V + V_i;
            end
        end
        function D = inertiaMatrix(obj,varargin)
            %return nxn generalized inertia matrix
            
            N = obj.DegreeOfFreedom;
            
            if ~isempty(varargin)
                isSymExplicit = strcmp('symbolic',varargin{1});
            else
                isSymExplicit = false;
            end 
            
            if obj.Environment.IsSym || isSymExplicit
                D = sym(zeros(N));    
            else
                D = zeros(N);
            end
            
            listID = obj.KinematicChain.Tree.getListLinkID;
            numLink = numel(listID);
            for i = 1:numLink-1, % not including base/ ground
                
                % get parameters
                link = obj.KinematicChain.Tree.getChildByID(listID{i+1});
                body = link.Body;
                
                D_i = body.generalizedInertiaMatrix;
                D = D + D_i;
                
            end
            
            if obj.Environment.IsSym || isSymExplicit
                D = simplify(D);
            else
                [qsym,qdsym,~] = RigidRobot.symbolicState(N);
                D = subs(D,qsym,obj.State.q);
                D = subs(D,qdsym,obj.State.qd);
                D = eval(D);
            end
        end
        function C = coriolisMatrix(obj)
            %return nxn generalized inertia Coriolis matrix
            
            % one needs symbolic jacobian to differentiate
            % jacobian need to be defined with symbolic q 
            N = obj.DegreeOfFreedom;
            [q,qd,~] = RigidRobot.symbolicState(N);
            
            D = inertiaMatrix(obj,'symbolic');
            
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
            
        end
        function G = gravityMatrix(obj)
            N = obj.DegreeOfFreedom;
            [q,~,~] = RigidRobot.symbolicState(N);
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
                
                listID = obj.KinematicChain.Tree.getListLinkID;
                numLink = numel(listID);
                P = sym(0);
                for i = 1:numLink-1, % not including base/ ground
                    link = obj.KinematicChain.Tree.getChildByID(listID{i+1});
                    body = link.Body;
                    m = body.Mass;
                    rc = body.Pose.Translation;
                    P = P + m*gravity_vector'*rc;
                end
            end
        end
        function updateState(obj,q,qd,qdd)
            % update D,C,G matrices with corresponding numeric values of
            % q,qd, and qdd
            
            N = obj.DegreeOfFreedom;
            state.q = reshape(q,N,1);
            state.qd = reshape(qd,N,1);
            state.qdd = reshape(qdd,N,1);
            
            obj.State = state;
            
            listID = obj.KinematicChain.Tree.getListLinkID;
            numLink = numel(listID);
            
            for i = 1:numLink-1,
                
                link = obj.KinematicChain.Tree.getChildByID(listID{i+1});
                link.Joint.State.q = state.q(i);
                link.Joint.State.qd = state.qd(i);
                link.Joint.State.qdd = state.qdd(i);
                
            end
            
            dynamicParam.Inertia = obj.inertiaMatrix;
            dynamicParam.Coriolis = obj.coriolisMatrix;
            dynamicParam.Gravity = obj.gravityMatrix;
            
            obj.DynamicParameters = dynamicParam;
            
        end
    end
    methods (Static)
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

