%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPUTERECRUITMENT.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Descritpion: This script reads recruitment curve data from xpc and
% computes the parameters of the recruitment curve.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eric Schearer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created: 26 November 2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Updated: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gather the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;

addpath("HelperFiles");

% muscle_names = ["Bicep", "Tricep", "Pronator Teres", "Brachioradialis", "Flexor Carpi Radialis"...
%                 "Palmaris Longus", "Flexor Carpi Ulnaris", "Extensor Carpi Radialis Longus"];

% OPTIONS FOR THE USER
subject_num = 9011;
save_figs = false;
show_figs  = false;
save_models = true;
save_model_nums = [1;  % Bicep
                   2;  % Tricep
                   2;  % Pronator Teres
                   1;  % Brachioradialis
                   2;  % Flexor Carpi Radialis
                   1;  % Palmaris Longus
                   1;  % Flexor Carpi Ulnaris
                   2]; % Extensor Carpi Radialis Longus

warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames')

if show_figs
    set(0,'DefaultFigureVisible','on');
else
    set(0,'DefaultFigureVisible','off');
end

filepath = "C:/Git/FES_Exo/data/S" + num2str(subject_num) + "/RC_Cal";
folder_info = dir(filepath);
num_muscles = 1;
for i = 1:length(folder_info)
    if (folder_info(i).name ~= "..") && (folder_info(i).name ~= ".") && (folder_info(i).name ~= "TestingNums.xlsx") && ...
       (folder_info(i).name ~= "~$TestingNums.xlsx") && (folder_info(i).name ~= "Models")
        muscle_names(num_muscles) = convertCharsToStrings(folder_info(i).name);
        num_muscles = num_muscles + 1;
    end
end
% muscle_names = muscle_names(1);
num_files = 0; % number to track total number of files
muscle_num = 0;
for muscle_name = muscle_names
    folder_info = dir(filepath+"/" + muscle_name);
    folder_name = extractAfter(folder_info(1).folder,"Cal\");
    muscle_num = str2num(folder_name(1))+1;
    muscle_file_num = 0; % number to find out if we want to save the file or not
    for i = 1:length(folder_info)
        if (folder_info(i).name ~= "..") && (folder_info(i).name ~= ".") && (folder_info(i).name(end-3:end) ~= ".fig") && (folder_info(i).name(end-3:end) ~= ".png")
            num_files = num_files + 1;
            muscle_file_num = muscle_file_num + 1;
            data_files(num_files) = filepath + "/" + muscle_name + "/" + convertCharsToStrings(folder_info(i).name);
            
            if muscle_file_num == save_model_nums(muscle_num)
                save_mdl(num_files) = true;
            else
                save_mdl(num_files) = false;
            end
            muscle_nums(num_files) = muscle_num;
            
        end
    end
end
muscle_names = ["Bicep", "Tricep", "Pronator Teres", "Brachioradialis", "Flexor Carpi Radialis"...
                "Palmaris Longus", "Flexor Carpi Ulnaris", "Extensor Carpi Radialis Longus"];
% num_files = 2
for file_num = 1:num_files
    close all;
%     data_files(file_num)
    [save_filepath,save_filename,ext] = fileparts(data_files(file_num));
    
    fprintf("Analyzing file numer %i: %s\n",file_num,save_filename)

    data = readtable(data_files(file_num));

    time  = data.time_s_ - data.time_s_(1);
    hmForce = [data.elbow_fe_trq_Nm_, data.forearm_ps_trq_Nm_, data.wrist_fe_trq_Nm_, data.wrist_ru_trq_Nm_];
    stimInputRaw = data.stim_pw_us_;
    stim_offset = stimInputRaw(end);
    stimInput = stimInputRaw-stim_offset;

    muscle=1;
    Fs = 1000;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % set values of the stimulation train
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    delay           = 1; % delay before anything happens
    condTime        = 1; % duration of conditioning stim
    condRest        = 5; % rest after conditioning stim
    pulseRest       = 2; % rest after an impulse
    rampRest        = 2; % rest after a ramp
    halfRamp        = 2; % duration of half ramp
    
    % Building time profiles from the given data
    pulseTimes(1)   = delay+condTime+condRest;
    pulseTimes(2)   = pulseTimes(1) + pulseRest;
    pulseTimes(3)   = pulseTimes(2) + pulseRest;
    pulseTimes(4)   = pulseTimes(3) + pulseRest;
    rampTimes(1)    = delay+condTime+condRest+4*pulseRest;
    rampTimes(2)    = rampTimes(1) + 2*halfRamp + rampRest;
    rampTimes(3)    = rampTimes(2) + 2*halfRamp + rampRest;
    rampTimes(4)    = rampTimes(3) + 2*halfRamp + rampRest;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % compute and average the peak time for each of the impulses
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1:4
        peakTimes(i) = computePeakTime(time,hmForce,pulseTimes(i));
    end
    
    peakTime = mean(peakTimes);
    if peakTime < 0.05 || peakTime > 0.2
        warning("Calculated peak time is too large or too small. Using defaultin value of 0.1")
        peakTime = 0.1;
    end
   
    % length of FFT
    nfft = 5000;
    % frequency vector
    f = [(0:nfft/2-1)*Fs/nfft (-nfft/2:-1)*Fs/nfft ];
    frad = f*2*pi;
    % cutoff frequency for low pass filter
    %cutoff = 2;             % muscle 1
    %filterOrder = 2;        % muscle 1
    %cutoff = 1;             % muscle 2
    %filterOrder = 2;        % muscle 2
    %cutoff = 2;             % muscle 3
    %filterOrder = 2;        % muscle 3
    %cutoff = 1;             % muscle 4
    %filterOrder = 2;        % muscle 4
    %cutoff = 1;             % muscle 5
    %filterOrder = 2;        % muscle 5
    %cutoff = 2;             % muscle 6
    %filterOrder = 2;        % muscle 6
    %cutoff = 0.5;           % muscle 7
    %filterOrder = 3;        % muscle 7
    %cutoff = 1;             % muscle 8
    %filterOrder = 2;        % muscle 8
    cutoff = 0.5;            % muscle 9
    filterOrder = 2;         % muscle 9
    % evaluate the LDS transfer function at the frequencies of the ramp
    % response
    H = tf([1/peakTime^2],[1 2/peakTime 1/peakTime^2]);
    HofFreq = zeros(length(f),1);
    magHofFreq = zeros(length(f),1);
    for i = 1:length(f)
        HofFreq(i) = evalfr(H,1j*frad(i));
        magHofFreq(i) = norm(HofFreq(i));
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ramp deconvolution
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for i = 1:4
        [curve, rampInput, rampOutput] = evalRamp(time, hmForce, rampTimes(i), halfRamp, stimInput, Fs, f, nfft, cutoff, filterOrder, HofFreq);
        if i == 1
            vec_size = length(curve);
        end
        curves{i} = curve;
        rampInputs{i} = rampInput;
        rampOutputs{i} = rampOutput;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % compute RC and plot all the deconvolved ramps together
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % inputs = reshape(rampInputs',[],1);
    inputs = [rampInputs{1}; rampInputs{2}; rampInputs{3}; rampInputs{4}];
    curves_row = [curves{1}; curves{2}; curves{3}; curves{4}];
    % curves_row = reshape(curves',[],1);

    % fit the data
    fo = fitoptions('Method','NonLinearLeastSquares',...
                   'Lower',[0,0],...
                   'Upper',[max(curves_row),inf],...
                   'StartPoint',[max(curves_row),1]);
    ft = fittype(['a/(1+exp(-b*(x-' (num2str(max(inputs)/2)) ')))-a/(1+exp(b*' (num2str(max(inputs)/2)) '))'],'options',fo);
    [curvefit,~] = fit(inputs,curves_row,ft);
    test_y = curvefit.a./(1+exp(-curvefit.b.*(rampInputs{1}-max(inputs)/2)))-curvefit.a./(1+exp(curvefit.b*max(inputs)/2));

    curve_styles  = ["b+", "g+", "r+", "c+"];
    output_styles = ["b.", "g.", "r.", "c."];

%     [save_filepath,save_filename,ext] = fileparts(data_files(file_num));
    
    h9 = figure(9);
    hold on
    for i = 1:4
        plot(rampInputs{i},curves{i},curve_styles(i));
%         plot(rampInputs{i},rampOutputs{i},output_styles(i));
    end
    plot(rampInputs{1},test_y,'k','LineWidth',2)
    if save_figs
        saveas(h9, save_filepath + "/" + save_filename + "_fit.png")
    end
    
%     if save_filename == "amp_65_pw_18_33_rc_calibration_data_2020_10_08_13_29_09"
%         break
%     end

    h10 = figure(10);
    hold on
    plot(time(1:size(rampInputs{1},1)),rampInputs{1});
    plot(time(1:size(curves{1},1)),curves{1}*100,'b');
    plot(time(1:size(rampOutputs{1},2)),rampOutputs{1}*100,'b');
    plot(time(1:size(curves{2},1)),curves{2}*100,'g');
    plot(time(1:size(rampOutputs{2},2)),rampOutputs{2}*100,'g');
    plot(time(1:size(curves{3},1)),curves{3}*100,'r');
    plot(time(1:size(rampOutputs{3},2)),rampOutputs{3}*100,'r');
    plot(time(1:size(curves{4},1)),curves{4}*100,'c');
    plot(time(1:size(rampOutputs{4},2)),rampOutputs{4}*100,'c');
    if save_figs
        saveas(h10, save_filepath + "/" + save_filename + "_ramp.png")
    end
    
    if save_models
        if save_mdl(file_num)
            RCModel.a        = curvefit.a;
            RCModel.b        = curvefit.b;
            RCModel.c        = max(inputs)/2;
            RCModel.amp      = data.stim_amp_mA_(1);
            RCModel.pw_low   = min(stimInputRaw);
            RCModel.pw_high  = max(stimInputRaw);
            RCModel.name     = muscle_names(muscle_nums(file_num));
            saveRCModel(RCModel, filepath, muscle_nums(file_num));
        end
    end
end

set(0,'DefaultFigureVisible','on');
% h10 = figure(10);
% hold on
% plot(time,pw)
% saveas(h10,['./figures/pwMuscle',num2str(muscle),'.fig'])
% 
% h11 = figure(11);
% hold on
% plot(time,hmForce)
% saveas(h11,['./figures/forceMuscle',num2str(muscle),'.fig'])

% save(['./datafiles/rcparams',num2str(muscle)],'curve.a','curve.b','curve.c','time','hmForce','pw','stimInput')

function saveRCModel(RCModel, filepath, muscle_num)
    json_filepath = filepath + "/Models/" + muscle_num + "_mdl.json";
    fid=fopen(json_filepath,'w');
    encodedJSON = jsonencode(RCModel);
    fprintf(fid, encodedJSON);
    fclose(fid);
end