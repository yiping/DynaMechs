function drawStamps(tList,stateList,modheight)
    %h=axes();]
    height = 1000;
    if nargin==3
        height = modheight;
    end
    ys=ylim;
    axis tight
    xs=xlim;
    for i=1:length(tList)
        color=':k';
        w=1;
        if(stateList(i)==3)
            color='k-.';
            w=1;
        end
        plot([tList(i) tList(i)],[-height,height],color,'LineWidth',w);
    end
    axis([xs ys])
end