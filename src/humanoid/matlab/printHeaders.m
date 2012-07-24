function printHeaders( fid,links,index )

link = links{index};
numChildren = length(link.children);
fprintf('Link %s - %d\n',link.name,numChildren);

fprintf(fid,'\n');

printTabs(fid,link);
fprintf(fid,'%s {\n',link.type);

printTabs(fid,link);
fprintf(fid,'\tName\t%s\n',link.name);

if ~strcmp(link.type,'ZScrewTxLink')
    printTabs(fid,link);
    fprintf(fid,'\tGraphics_Model\t%s\n',link.model);
    
    printTabs(fid,link);
    fprintf(fid,'\tMass\t\t\t\t%.6f\n',link.mass);
    
    printTabs(fid,link);
    fprintf(fid,'\tInertia\t\t\t\t');
    for row=1:3
        if row>1
           printTabs(fid,link);
           fprintf(fid,'\t\t\t\t\t\t');
        end
        fprintf(fid,'%.6f\t%.6f\t%.6f\n',link.Ibar(row,1),link.Ibar(row,2),link.Ibar(row,3));
    end
    
    printTabs(fid,link);
    fprintf(fid,'\tCenter_of_Gravity\t%.6f\t%.6f\t%.6f\n',link.com(1),link.com(2),link.com(3));

    printTabs(fid,link);
    [r c]= size(link.contacts);
    fprintf(fid,'\tNumber_of_Contact_Points\t%d\n',r);
    
    for i=1:r
       printTabs(fid,link);
       fprintf(fid,'\t');
       if i==1
           fprintf(fid,'Contact_Locations\t');
       else
           fprintf(fid,'\t\t\t\t\t');
       end
       fprintf(fid,'%.6f\t%.6f\t%.6f\n',link.contacts(i,1),link.contacts(i,2),link.contacts(i,3)); 
    end     
end

if strcmp(link.type,'QuaternionLink')
    printTabs(fid,link);

    fprintf(fid,'\tPosition_From_Inboard_Link\t%.6f\t%.6f\t%.6f\n',link.pos(1),link.pos(2),link.pos(3));
    printTabs(fid,link);
    fprintf(fid,'\tOrientation_Quat\t%.6f\t%.6f\t%.6f\t%.6f\n',link.quat(1),link.quat(2),link.quat(3),link.quat(4));
    printTabs(fid,link);
    fprintf(fid,'\tInitial_Angular_Velocity\t0\t0\t0\n');
    printTabs(fid,link);
    fprintf(fid,'\tJoint_Friction\t0\n');             
end    


if strcmp(link.type,'RevoluteLink')
    printTabs(fid,link);
    fprintf(fid,'\tMDH_Parameters\t%.6f\t%.6f\t%.6f\t%.6f\n',link.mdh(1),link.mdh(2),link.mdh(3),link.mdh(4));
    printTabs(fid,link);
    fprintf(fid,'\tInitial_Joint_Velocity\t0\n');
    printTabs(fid,link);
    fprintf(fid,'\tJoint_Limits\t0\t0\n');
    printTabs(fid,link);
    fprintf(fid,'\tJoint_Limit_Spring_Constant\t0\n');
    printTabs(fid,link);
    fprintf(fid,'\tJoint_Limit_Damper_Constant\t0\n');
    printTabs(fid,link);
    fprintf(fid,'\tActuator_Type\t0\n');
    printTabs(fid,link);
    fprintf(fid,'\tJoint_Friction\t0\n');
end

if strcmp(link.type,'MobileBaseLink')
    printTabs(fid,link);
    fprintf(fid,'\tPosition\t%.6f\t%.6f\t%.6f\n',link.pos(1),link.pos(2),link.pos(3));
    
	printTabs(fid,link);
    fprintf(fid,'\tOrientation_Quat\t%.6f\t%.6f\t%.6f\t%.6f\n',link.quat(1),link.quat(2),link.quat(3),link.quat(4));
	
    printTabs(fid,link);
    fprintf(fid,'\tVelocity 0 0 0 0 0 0\n');
    
end

if strcmp(link.type,'ZScrewTxLink')
    printTabs(fid,link);
    fprintf(fid,'\tZScrew_Parameters\t%.6f\t%.6f\n',link.params(1),link.params(2));
end

printTabs(fid,link);
fprintf(fid,'}\n');
printTabs(fid,link);
fprintf(fid,'# End %s\n',link.name);


for i=1:numChildren
    if numChildren > 1
        fprintf(fid,'\n');
        printTabs(fid,link);
        fprintf(fid,'Branch {\n');
        links{link.children(i)}.tabs=link.tabs+1;
    else
        links{link.children(i)}.tabs=link.tabs;
    end
    
   
    printHeaders(fid,links,link.children(i));
    
    if numChildren > 1
        printTabs(fid,link);
        fprintf(fid,'}\n');
        printTabs(fid,link);
        fprintf(fid,'# End Branch\n');
    end  
    
end




end

