function [ dataStruct ] = ProcessGprCalDataWithPassive( filepath )
%PROCESSDATA processes data in a specific file for use in generating a GP
%model
%   Detailed explanation goes here

    for i = 1:9
        dataStruct(i).angleData = [];
        dataStruct(i).torqueData = [];
    end

    positions = dir(char(filepath));
    for i = 1:size(positions,1)
        position = positions(i).name;
        if ~strcmp(position,'.') && ~strcmp(position,'..') && ~strcmp(position,'Models')
            stim_files = dir(char(filepath + "/" + position));
            for j = 1:size(stim_files)
                stim_file = stim_files(j).name;
                if ~strcmp(stim_file,'.') && ~strcmp(stim_file,'..')
                    stim_num  = str2num(stim_file(1));
                    if stim_num == 0
                        stim_num = 9;
                    end
                    data_mat = csvread(filepath + "/" + position + "/" + stim_file,1,0);
                    position_data = mean(data_mat(:,3:6) ,1);
                    torque_data   = mean(data_mat(:,11:14),1);
                    passive_data  = mean(data_mat(:,15:18),1); % This line added
                    dataStruct(stim_num).torqueData = [dataStruct(stim_num).torqueData; passive_data - torque_data]; % This line changed
                    dataStruct(stim_num).angleData = [dataStruct(stim_num).angleData;position_data];
                end
            end
        end
    end
end

