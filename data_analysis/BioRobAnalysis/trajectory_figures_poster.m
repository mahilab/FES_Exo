multi_filepath = "C:/Git/FES_Exo/data/data_collection/S9004/multi/f50_e50_kp0.000000_kd0.000000_2020_10_08_15_50_43.csv";
single_filepath = "C:/Git/FES_Exo/data/data_collection/S9004/single/f50_e50_kp0.000000_kd0.000000_2020_10_08_16_35_40.csv";

filenames = [single_filepath, multi_filepath];

x_lims = [0,27;0,14];

colors = [249, 80, 70;
           77,170, 87;
          118,190,208;
          249,190, 0]/255;
      
font_size = 13;
line_width = 2;

close all;
h = figure();
for filenum = 1:2

    tbl = readtable(filenames(filenum));
    
    tbl = tbl(find(tbl.time_s_>=2,1):end,:);
    tbl.time_s_ = tbl.time_s_ - tbl.time_s_(1);
    
    subplot(2,1,filenum);
    plot(tbl.time_s_,rad2deg(tbl.com_elbow_fe_rad_),'Color',colors(1,:),'LineWidth',line_width)
    hold on
    plot(tbl.time_s_,rad2deg(tbl.com_forearm_ps_rad_),'Color',colors(2,:),'LineWidth',line_width)
    hold on
    plot(tbl.time_s_,rad2deg(tbl.com_wrist_fe_rad_),'Color',colors(3,:),'LineWidth',line_width)
    hold on
    plot(tbl.time_s_,rad2deg(tbl.com_wrist_ru_rad_),'Color',colors(4,:),'LineWidth',line_width)
    hold on
    ylabel("Position (deg)",'Interpreter','latex','FontSize',font_size);
    if filenum == 2
        xlabel("Time (s)",'Interpreter','latex','FontSize',font_size);
    end
    xlim(x_lims(filenum,:))
    if filenum == 1
        legend("Elbow F/E", "Forearm P/S", "Wrist F/E", "Wrist R/U","Location","southeast",'Interpreter','latex','FontSize',font_size-3);
    end
    grid on;
end

set(h,'Position',[100,100,600,500],'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3),pos(4)]);%[pos(3)-1.7, pos(4)+0.25])
print(h,"trajectory_plots.pdf",'-dpdf','-r0')