% Transform function needs to be fixed

% normally, just do LinkTree('base')
tree = LinkTreeNew();

armBody = RigidBody('arm',2,[1 0 0]',eye(3));
jointParam.stiffness = 'k_arm';
jointParam.damping = 'b_arm';
armJoint = Joint(1,'base','arm','q_arm',jointParam);
tree.addLink('base',armBody,armJoint,2*eye(4));

handBody = RigidBody('hand',3,[2 0 0]',0.5*eye(3));
jointParam.stiffness = 'k_hand';
jointParam.damping = 'b_hand';
handJoint = Joint(1,'arm','hand','q_hand',jointParam);
tree.addLink('arm',handBody,handJoint,3*eye(4));


finger1Body = RigidBody('finger1',1,[2 0 0]',2.5*eye(3));
jointParam.stiffness = 'k_finger1';
jointParam.damping = 'b_finger1';
finger1Joint = Joint(1,'hand','finger1','q_finger1',jointParam);
tree.addLink('hand',finger1Body,finger1Joint,3*eye(4));

finger2Body = RigidBody('finger2',1,[2 0 0]',0.25*eye(3));
jointParam.stiffness = 'k_finger2';
jointParam.damping = 'b_finger2';
finger2Joint = Joint(1,'hand','finger2','q_finger2',jointParam);
tree.addLink('hand',finger2Body,finger2Joint,5*eye(4));

tree.transform('arm','finger2')
 