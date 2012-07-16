function drawStamps(stampList,stampStates)
    %h=axes();]
    xs=xlim;
    ys=ylim;
    for i=1:length(stampList)
        color=':k';
        w=1;
        if(stampStates(i)==3)
            color='k-.';
            w=1;
        end
        plot([stampList(i) stampList(i)],[-1000,1000],color,'LineWidth',w);
    end
    axis([xs ys])
end