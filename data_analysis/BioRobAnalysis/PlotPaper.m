function [] = PlotPaper(titles,fes_percent,means,stds,xtitle,ytitle,normalize,single_plot)
%PLOTPAPER Summary of this function goes here
%   Detailed explanation goes here
    alpha = 0.2;
    colors = [249, 80, 70;
               77,170, 87;
              118,190,208;
              249,190, 0]/255;
          
    font_size = 13;
          
%     colors = [249,110, 70;
%               249,200, 70;
%                77,170, 87;
%               118,190,208]/255;

    if normalize
        for i = 1:4
            norm_val = means(i,1);
            means(i,:) = means(i,:)/norm_val;
            stds(i,:)  = stds(i,:)/norm_val;
        end
        ytitle = "Normalized " + ytitle;
    end
    for i = 1:4
        if ~single_plot
            subplot(2,2,i)
        end
%         e = errorbar(fes_percent,means(i,:),2*stds(i,:));
        e = plot(fes_percent,means(i,:),'Color',colors(i,:));
        if single_plot
            e.LineWidth = 2;
        end
        hold on
        fill([fes_percent fliplr(fes_percent)],...
             [means(i,:)+stds(i,:) fliplr(means(i,:)-stds(i,:))],...
             colors(i,:),'FaceAlpha', alpha,'linestyle','none','HandleVisibility','off');
%         title(titles(i))

        ax = gca;
        ax.FontSize = font_size-3;
%         ax.Interpreter = 'latex'

        xlabel(xtitle,'Interpreter','latex','FontSize',font_size)
        ylabel(ytitle,'Interpreter','latex','FontSize',font_size)
        ylim([0,2])
        xlim([0,max(fes_percent)])
        grid on
        hold on
    end
    if single_plot
        legend(titles,'Location','NW','Interpreter','latex','FontSize',font_size-4)
    end
    
    
end

