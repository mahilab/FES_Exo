close all;

colors = [249, 80, 70;
           77,170, 87;
          118,190,208;
          249,190, 0]/255;

font_size = 13;

h = figure();
hold on;
for i = 1:4
    plot(rampInputs{i}/14,curves{i}/max(test_y),'Color',colors(i,:),'Marker','.','LineStyle','none','MarkerSize',10);
end
plot(rampInputs{1}/14,test_y/max(test_y),'k','LineWidth',2)

xticks(0:0.2:1)

xlabel("$\alpha$",'Interpreter','latex','FontSize',font_size);
ylabel("Normalized Torque",'Interpreter','latex','FontSize',font_size);
legend("Data Set 1","Data Set 2","Data Set 3","Data Set 4","Line of Fit","Location","southeast",'Interpreter','latex','FontSize',font_size-1);

title('Recruitment Curve Fit','Interpreter','latex','FontSize',font_size+2);
grid on;

set(h,'Position',[100,100,600,500],'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3),pos(4)]);%[pos(3)-1.7, pos(4)+0.25])
print(h,"recruitment_curve_plot.pdf",'-dpdf','-r0')

