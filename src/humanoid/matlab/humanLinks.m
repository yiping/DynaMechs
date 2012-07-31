% Torso Link
clear link;
link.mass = .578 * Mass;
link.length = (.818-.530)*Height;
link.width  = .259 * Height;
link.depth  = .3 * link.width;
link.com    = [0 0 link.length*.66];
link.I      = 1/12 * link.mass * diag([link.depth^2 + link.width^2, ...
                    link.length^2+link.width^2, link.length^2+link.depth^2]);
link.Ibar   = link.I + link.mass*cross(link.com)*cross(link.com)';
link.hipWidth = .191*Height;
link.type   = 'MobileBaseLink';
link.model  = '"./human_model/torso.xan"';
link.contacts = [];
link.quat   = [0 0 0 1];
link.pos    = [2 2 Height];
link.offset = [-link.depth/2 -link.width/2 0];
link.scale  = [link.depth link.width link.length];
torsoLink = link;
totalMass = totalMass + link.mass;

clear link;
link.type = 'ZScrewTxLink';
link.params = [0 0];
zScrewLink = link;


% Thigh Link
clear link;
link.mass = .1 * Mass;
totalMass = totalMass + 2*link.mass;
link.length = (.530-.285)*Height;
link.width  = .2 * link.length;
link.depth  = .15 * link.length;
link.com    = [.433*link.length 0 0];
link.I      = 1/12 * link.mass * diag([link.depth^2 + link.width^2, ...
                    link.length^2+link.width^2, link.length^2+link.depth^2]);
link.Ibar   = link.I + link.mass*cross(link.com)*cross(link.com)';
link.contacts = [link.length	0	0];
link.type   = 'QuaternionLink';
link.quat   = [0 0 0 1];
link.model  = '"./human_model/thigh.xan"';
link.offset = [0 -link.depth/2 -link.width/2];
link.scale  = [link.length link.depth,link.width];

hipLink = link;


% Shank Link
clear link;
link.mass = .0465 * Mass;
totalMass = totalMass + 2*link.mass;
link.length = (.285-.039)*Height;
link.width  = .2 * link.length;
link.depth  = .15 * link.length;
link.com    = [.433*link.length 0 0];
link.I      = 1/12 * link.mass * diag([link.depth^2 + link.width^2, ...
                    link.length^2+link.width^2, link.length^2+link.depth^2]);
link.Ibar   = link.I + link.mass*cross(link.com)*cross(link.com)';
link.contacts = [link.length	0	0];
link.mdh    = [hipLink.length 0 0 0];
link.type   = 'RevoluteLink';
link.model  = '"./human_model/shank.xan"';
link.offset = [0 -link.depth/2 -link.width/2];
link.scale  = [link.length link.depth,link.width];
shankLink = link;

% Ankle Link
clear link;
link.mass = 0;
link.com = [0,0,0];
link.I = zeros(3,3);
link.Ibar = link.I;
link.type = 'RevoluteLink';
link.mdh  = [shankLink.length	0	0	0];
link.model  = '"./human_model/ankle.xan"';
link.contacts = [];
ankleLink = link;


% Foot Link
clear link;
link.mass = .0145 * Mass;
totalMass = totalMass + 2*link.mass;
link.length = .152*Height;
link.width  = .055 * Height;
link.depth  = .039 * Height;
%%%%%%%%%%%%%%
link.ankleLocation = [0 0 .25*link.length];
link.contacts = [link.depth	link.width/2	0; ...
                 link.depth -link.width/2   0; ...
                 link.depth link.width/2    link.length; ...
                 link.depth -link.width/2   link.length] - ...
                [link.ankleLocation;link.ankleLocation; ...
                    link.ankleLocation;link.ankleLocation];
link.com    = [.5*link.depth 0 .5*link.length] - link.ankleLocation;
link.I      = 1/12 * link.mass * diag([link.length^2 + link.width^2, ...
                    link.length^2+link.depth^2, link.width^2+link.depth^2]);
link.Ibar   = link.I + link.mass*cross(link.com)*cross(link.com)';
link.type   = 'RevoluteLink';
link.mdh    = [0	pi/2	0	0];
link.model  = '"./human_model/foot.xan"';
link.offset = [0 -link.width/2 0] - link.ankleLocation;
link.scale  = [link.depth link.width link.length];
footLink = link;


% Upper Arm
clear link;
link.mass = .028 * Mass;
totalMass = totalMass + 2*link.mass;
link.length = (.186)*Height;
link.width  = .2 * link.length;
link.depth  = .15 * link.length;
link.com    = [.436*link.length 0 0];
link.I      = 1/12 * link.mass * diag([link.depth^2 + link.width^2, ...
                    link.length^2+link.depth^2, link.length^2+link.width^2]);
link.Ibar   = link.I + link.mass*cross(link.com)*cross(link.com)';
link.contacts = [link.length	0	0];
link.type   = 'QuaternionLink';
link.quat   = [0 0 0 1];
link.model  = '"./human_model/upperArm.xan"';
link.offset = [0 -link.depth/2 -link.width/2];
link.scale  = [link.length link.depth,link.width];

upperArmLink = link;


% Fore Arm
clear link;
link.mass = .022 * Mass;
totalMass = totalMass + 2*link.mass;
link.length = (.146+.108)*Height;
link.width  = .1 * link.length;
link.depth  = .06 * link.length;
link.com    = [.682*link.length 0 0];
link.I      = 1/12 * link.mass * diag([link.depth^2 + link.width^2, ...
                    link.length^2+link.depth^2, link.length^2+link.width^2]);
link.Ibar   = link.I + link.mass*cross(link.com)*cross(link.com)';
link.contacts = [link.length	0	0];
link.type   = 'RevoluteLink';
link.mdh    = [upperArmLink.length	0	0	0];
link.model  = '"./human_model/foreArm.xan"';
link.offset = [0 -link.depth/2 -link.width/2];
link.scale  = [link.length link.depth,link.width];
foreArmLink = link;