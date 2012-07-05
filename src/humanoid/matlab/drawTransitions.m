function drawStamps(tList,stateList)
    %h=axes();]
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
        plot([tList(i) tList(i)],[-1000,1000],color,'LineWidth',w);
    end
    axis([xs ys])
end