%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPUTEMODELS.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Descritpion: This script loads data from a folder and does GPR for
% each muscle where the inputs are joint angles and the outputs are joint
% torques.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nathan Dunkelberger
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created: 07/31/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Updated:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('HelperFiles') 

% clear all
close all
base_filepath = "C:/Git/FES_Exo/data/S";
subject_num = 9998;
save_models = true;

% nathanData = ProcessData(base_filepath + num2str(subject_num));

nathanData = ProcessGprCalDataWithPassive(base_filepath + num2str(subject_num) + "/GPR_Cal");
base_filepath = base_filepath + num2str(subject_num) + "/GPR_Cal";

% allTargets = readtable("C:/Git/FES_Exo/data/CollectionPoints.csv");
% elbowForearmTargets = allTargets(1:9,:);
% wristTargets        = allTargets(10:end,:);

% randomizedTargets = readtable("C:/Git/FES_Exo/data/CollectionPointsRandomized.csv");
% 
% for i = 1:height(randomizedTargets)
%     nathanDataElbowForearm = nathanData
%     nathanDataWrist = nathanData    
% end

%%

n_muscles = 9;
n_joints  = 4;

muscle_names = ["Bicep", "Tricep", "Pronator Teres", "Brachioradialis", "Flexor Carpi Radialis"...
                "Palmaris Longus", "Flexor Carpi Ulnaris", "Extensor Carpi Radialis Longus", "Passive"];
            
joint_names  = ["ElbowFE", "ForearmPS", "WristFE", "WristRU"];

joint_vars   = ["Var1", "Var2", "Var3", "Var4"];

for i = 1:(n_muscles-1)
    if ~isempty(nathanData(i).angleData)
        modeldata.muscle(i).label = muscle_names(i);

        % Segment the data to the appropriate muscle
        angleData = nathanData(i).angleData;
        torqueData= -(nathanData(i).torqueData-nathanData(9).torqueData);

        for j = 1:n_joints
            modeldata.muscle(i).joint(j).label = joint_names(j);

            tbl = array2table([torqueData(:,j),angleData]);

            joint_torque = torqueData(:,j);
            for k = 1:24
                torqueData_adj((1:3)+(k-1)*3) = joint_torque((1:3)+(k-1)*3)-mean(joint_torque((1:3)+(k-1)*3));
            end

            sigma0 = std(torqueData_adj);
            sigmaF0 = sigma0;
            sigmaM0 = [1,1,1,1]';

            gprMdl = fitrgp(tbl,'Var1','KernelFunction','ardsquaredexponential',...
                               'FitMethod','exact','PredictMethod','exact','BasisFunction','linear',...
                               'Standardize',false,...
                               'KernelParameters',[sigmaM0;sigmaF0],'Sigma',sigma0);

            subplot(4,8,8*(j-1)+(i-1)+1)
            plot(tbl.Var1,resubPredict(gprMdl),'.')

            gprMdls{i,j} = gprMdl;

            if save_models
                filepath = base_filepath + "/Models/m" + ...
                           num2str(i) + "j" + num2str(j) + "model.json"; 
                save_model(gprMdl, filepath, i, j, muscle_names, joint_names);
            end
        end
    end
end

%% Test predictions vs actual values
close all
% joint_num = 4
% muscle_num = 1
for joint_num = 1:4
    for muscle_num = 1:8
        if ~isempty(nathanData(muscle_num).angleData)
            for i = 1:length(angleData)
    %             y = resubPredict(gprMdls{muscle_num,joint_num});
                [y_temp,~,yint] = gprMdls{muscle_num,joint_num}.predict(angleData(i,:));
                y(i) = y_temp;
                y_95_above(i) = yint(1);
                y_95_below(i) = yint(2);
                x(i) = -(nathanData(muscle_num).torqueData(i,joint_num)-nathanData(9).torqueData(i,joint_num));
            end
            subplot(4,8,8*(joint_num-1)+(muscle_num-1)+1)
    %         plot(x,'r.'); hold on;
    %         plot(y,'b.')
            plot(y, 'b.'); hold on;
            plot(y_95_above, 'b-'); hold on;
            plot(y_95_below, 'b-'); hold on;
            plot(x, 'r.'); hold on;
        end
    end
end

%%
% for i = 1:n_joints
%     figure(i)
%     for j = 1:(n_muscles-1)
%         subplot(2,4,j)
%         sigmaM = gprMdls{j,i}.KernelInformation.KernelParameters(1:n_joints,1);
%         plot((1:n_joints)',log(sigmaM),'bo-');
% %         bar_names = categorical({'Elbow','Forearm','WristFE','WristRU'});
% %         bar(bar_names,log(sigmaM));
%         title(muscle_names(j) + ' ' + joint_names(i) + ' torque');
%         xticklabels({'Elbow','Forearm','WristFE','WristRU'})
%         ylabel('Log Length Scale')
%         sigmas(j,1:4) = log(sigmaM);
%     end
%     figure(i+4)
%     plot((1:n_joints)',mean(sigmas,1),'bo');
%     title(joint_names(i) + ' mean log length scales');
% end

for j = 1:(n_muscles-1)
    figure(j)
    for i = 1:n_joints
        subplot(2,2,i)
        sigmaM = gprMdls{j,i}.KernelInformation.KernelParameters(1:n_joints,1);
        plot((1:n_joints)',log(sigmaM),'bo-');
%         bar_names = categorical({'Elbow','Forearm','WristFE','WristRU'});
%         bar(bar_names,log(sigmaM));
        title(muscle_names(j) + ' ' + joint_names(i) + ' torque');
        xticks([1 2 3 4])
        xticklabels({'Elbow','Forearm','WristFE','WristRU'})
        ylabel('Log Length Scale')
        sigmas(j,1:4) = log(sigmaM);
    end
    figure(9)
    subplot(2,4,j);
    plot((1:n_joints)',mean(sigmas(j,:),1),'bo');
    title(muscle_names(j) + ' mean log length scales');
    xticks([1 2 3 4])
    xticklabels({'Elbow','Forearm','WristFE','WristRU'})
end
%%
gprMdl = fitrgp(tbl,'Var1','KernelFunction','ardsquaredexponential','FitMethod','exact','PredictMethod','exact','BasisFunction','linear')
% linear_fit = fitlm(angleData(:,j),torqueData(:,j));
% B = table2array(linear_fit.Coefficients(:,1));
% gprMdl = fitrgp(tbl,'Var1','KernelFunction','ardsquaredexponential','FitMethod','none','PredictMethod','exact','BasisFunction','linear','Beta',[B;0;0;0],...
%                 'KernelParameters',[exp(0.2),exp(0.2),exp(0.2),exp(0.2),100]);
% gprMdl = fitrgp(tbl,'Var1','FitMethod','exact','PredictMethod','exact','BasisFunction','linear',...
%                 'KernelFunction',@cov1,...
%                 'KernelParameters',sometm,...
%                 'BasisFunction',@mean1)
% num_points = 100;

pred_min_max = [-pi/2,    0;
                -pi/2, pi/2;
                    0,    0;
                    0,    0];
            
% for i = 1:n_joints
%     x_test(:,i) = linspace(pred_min_max(i,1),pred_min_max(i,2),num_points);
% end
wrist_nums = [-15,0,15];

for i = 1:3
    [x1, x2] = meshgrid(linspace(pred_min_max(1,1),pred_min_max(1,2),100),linspace(pred_min_max(2,1),pred_min_max(2,2),100));
    x3 = ones(100,100)*deg2rad(wrist_nums(i));
    x4 = ones(100,100)*deg2rad(wrist_nums(i));

    ypred = predict(gprMdl,[reshape(x1,[],1) reshape(x2,[],1) reshape(x3,[],1) reshape(x4,[],1)]);
    % ypred = resubPredict(gprMdl);

    ypred_resized = reshape(ypred,100,100);

    hm_tbl = array2table([reshape(x1,[],1) reshape(x2,[],1) reshape(x3,[],1) reshape(x4,[],1),ypred]);

    subplot(1,3,i);
    surf(x1,x2,ypred_resized)
%     heatmap(hm_tbl,'Var1','Var2','ColorVariable','Var5')
end

% figure()
% plot(tbl.Var1,'b');
% hold on
% plot(ypred,'r--');
% legend('True','Predicted');
% surf(x_test(:,1),x_test(:,2),reshape(ypred,10,10))

%% Testing fitting
clc;
X_new = [0,0,0,0];

predict(gprMdls{1,1},X_new)


% train_inputs  = gprMdls{1,1}.X{:,:};
train_inputs = gprMdls{1,1}.ActiveSetVectors;
alpha = gprMdls{1,1}.Alpha;
beta = gprMdls{1,1}.Beta;
theta = gprMdls{1,1}.KernelInformation.KernelParameters;

model.name         = muscle_names(1) + "_" + joint_names(1);
model.train_inputs = train_inputs;
model.alpha        = alpha;
model.beta         = beta;
model.theta        = theta;

json_filename = 'C:/Git/FES_Exo/data/S104/Models/model.json';
fid=fopen(json_filename,'w');
encodedJSON = jsonencode(model);
fprintf(fid, encodedJSON);
fclose(fid);

KernelComp = computeKernel(train_inputs,X_new,theta);

Guess = KernelComp * alpha + [1,X_new]*beta

function K = computeKernel(Train,New,theta)
    LS = (theta(1:size(Train,2)));
    SigmaF = (theta(end));
    for i = 1:size(Train,1)
        ard_sum = 0;
        for j = 1:size(Train,2)
            ard_sum = ard_sum + (Train(i,j)-New(j))^2/(LS(j)^2);
        end
        K(i) = SigmaF^2*exp(-1/2*ard_sum);
    end
end

function save_model(grpMdl, json_filepath, muscle_num, joint_num, muscle_names, joint_names)
    model.name         = muscle_names(muscle_num) + "_" + joint_names(joint_num);
    model.train_inputs = grpMdl.ActiveSetVectors;
    model.alpha        = grpMdl.Alpha;
    model.beta         = grpMdl.Beta;
    model.theta        = grpMdl.KernelInformation.KernelParameters;
    fid=fopen(json_filepath,'w');
    encodedJSON = jsonencode(model);
    fprintf(fid, encodedJSON);
    fclose(fid);
end
