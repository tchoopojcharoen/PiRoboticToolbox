% Transform function needs to be fixed

% normally, just do LinkTree('base')
tree = LinkTree('base',[],1);

tree.addLink('arm','base',2,0,0,0,0);
tree.addLink('hand','arm',3,0,0,0,0);
tree.addLink('finger1','hand',4,0,0,0,0);
finger2 = tree.addLink('finger2','hand',5,0,0,0,0);

tree.transform('arm','finger2')
 