%%Generate Config
addpath ~/Documents/MATLAB/DynamicsLibrary/

Height = 1.8288; % 6 feet
Mass = 72.5748; % 160 pounds

Height = Height*70/Mass; % 5' 9.5"
%Mass = 70;               %
Mass = 19.12;
totalMass = 0;



humanoidLinks
                

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


%addpath ../DynamicsLibrary


% Initial Orientation for Legs
R = [0 0 -1;-1 0 0;0 1 0]';
rotFact = .3;
% Rotate about y axis for initial fore/aft hip angle
R2 = expm(cross([0 -.2 0]*rotFact));
quat = RtoQuat(R2*R);
dmQuat = [quat(2:4) quat(1)];
links{3}.quat = dmQuat;
links{4}.mdh(4) = .5*rotFact;
links{5}.mdh(4) = -.3*rotFact;

links{7}.quat = dmQuat;
links{8}.mdh(4) = .5*rotFact;
links{9}.mdh(4) = -.3*rotFact;

% Initial Orientation for Right Arm
R = [1 0 0;0 0 1;0 -1 0]';
R2 = expm(cross([0 1.1*pi/2 0]));
R3 = expm(cross([-.2 0 0]));
quat = RtoQuat(R3*R2*R);
dmQuat = [quat(2:4) quat(1)];
links{11}.quat = dmQuat;
links{12}.mdh(4) = 0.50000;

% Initial Orientation for Left Arm
R = [1 0 0;0 0 1;0 -1 0]';
R2 = expm(cross([0 1.1*pi/2 0]));
R3 = expm(cross([.2 0 0]));
quat = RtoQuat(R3*R2*R);
dmQuat = [quat(2:4) quat(1)];
links{13}.quat = dmQuat;
links{14}.mdh(4) = 0.50000;


N=14;

for i=1:N
   links{i}.children=[];
end
for i=1:N
   if links{i}.parent
      links{links{i}.parent}.children(end+1) = i; 
   end
end



fid = fopen('../config/humanoid_box.dm','w');
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

%createXan('../config/humanoid_model/torso.xan',torsoLink);
%createXan('../config/humanoid_model/upperArm.xan',upperArmLink);
%createXan('../config/humanoid_model/foreArm.xan',foreArmLink);
%createXan('../config/humanoid_model/thigh.xan',hipLink);
%createXan('../config/humanoid_model/shank.xan',shankLink);
%createXan('../config/humanoid_model/foot.xan',footLink);

createXanTorso('../config/humanoid_box_model/torso.xan',torsoLink);
createXan('../config/humanoid_box_model/upperArm.xan',upperArmLink);
createXan('../config/humanoid_box_model/foreArm.xan',foreArmLink);
createXan('../config/humanoid_box_model/thigh.xan',hipLink);
createXan('../config/humanoid_box_model/shank.xan',shankLink);
createXan('../config/humanoid_box_model/foot.xan',footLink);


