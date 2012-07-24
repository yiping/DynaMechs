%%Generate Config
addpath ~/Documents/MATLAB/DynamicsLibrary/

Height = 1.8288; % 6 feet
Mass = 72.5748; % 160 pounds

Height = Height*70/Mass; % 5' 9.5"
Mass = 70;               %


totalMass = 0;



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
link.model  = '"./humanoid_model/torso.xan"';
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
link.model  = '"./humanoid_model/thigh.xan"';
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
link.model  = '"./humanoid_model/shank.xan"';
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
link.model  = '"./humanoid_model/ankle.xan"';
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
link.model  = '"./humanoid_model/foot.xan"';
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
link.model  = '"./humanoid_model/upperArm.xan"';
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
link.model  = '"./humanoid_model/foreArm.xan"';
link.offset = [0 -link.depth/2 -link.width/2];
link.scale  = [link.length link.depth,link.width];
foreArmLink = link;
                

links{1} = torsoLink;       links{1}.name='"Torso"';            links{1}.parent = 0;
links{2} = zScrewLink;      links{2}.name='"ZScrewTorso"';      links{2}.parent = 1;
links{3} = hipLink;         links{3}.name='"RightThigh"';       links{3}.parent = 2;
links{4} = shankLink;       links{4}.name='"RightShank"';       links{4}.parent = 3;
links{5} = ankleLink;       links{5}.name='"RightAnkle1"';      links{5}.parent = 4;
links{6} = footLink;        links{6}.name='"RightAnkle2"';      links{6}.parent = 5;
links{7} = hipLink;         links{7}.name='"LeftThigh"';        links{7}.parent = 2;
links{8} = shankLink;       links{8}.name='"LeftShank"';        links{8}.parent = 7;
links{9} = ankleLink;       links{9}.name='"LeftAnkle1"';       links{9}.parent = 8;
links{10}= footLink;        links{10}.name='"LeftAnkle2"';      links{10}.parent = 9;
links{11}= upperArmLink;    links{11}.name='"RightArm"';        links{11}.parent = 1;
links{12}= foreArmLink;     links{12}.name='"RightForeArm"';    links{12}.parent = 11;
links{13}= upperArmLink;    links{13}.name='"LeftArm"';         links{13}.parent = 1;
links{14}= foreArmLink;     links{14}.name='"RightArm"';        links{14}.parent = 13;


links{3}.pos = [0 -torsoLink.hipWidth/2 0];
links{7}.pos = [0  torsoLink.hipWidth/2 0];
links{11}.pos = [0 -torsoLink.width/2 torsoLink.length];
links{13}.pos = [0 torsoLink.width/2 torsoLink.length];




N=14;

for i=1:N
   links{i}.children=[];
end
for i=1:N
   if links{i}.parent
      links{links{i}.parent}.children(end+1) = i; 
   end
end



fid = fopen('../config/humanoid.dm','w');
links{1}.tabs = 1;
fprintf(fid,'# DynaMechs V 4.0 ascii\n');
fprintf(fid,'# humanoid parameter file\n\n');


fprintf(fid,'Articulation {\n\t');
fprintf(fid,'Name\t\t\t\t"Articulation"\n\t');
fprintf(fid,'Graphics_Model\t\t""\n\t');
fprintf(fid,'Position\t\t\t0\t0\t0\n\t');
fprintf(fid,'Orientation_Quat\t0\t0\t0\t1\n');

printHeaders(fid,links,1); 
	
fprintf(fid,'}\n');
fprintf(fid,'# End Articulation\n\n');
fclose(fid);

createXan('../config/humanoid_model/torso.xan',torsoLink);
createXan('../config/humanoid_model/upperArm.xan',upperArmLink);
createXan('../config/humanoid_model/foreArm.xan',foreArmLink);
createXan('../config/humanoid_model/thigh.xan',hipLink);
createXan('../config/humanoid_model/shank.xan',shankLink);
createXan('../config/humanoid_model/foot.xan',footLink);




