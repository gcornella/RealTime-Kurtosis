clear all; close all;
task_title = 'Exploration';
%% Import robot DH 
% Define robot links using the DH parameters
L1 = Link('d', 0.056 , 'a', 0, 'alpha', -pi/2); % Define first link
L2 = Link('d', 0, 'a', 0, 'alpha', pi/2);       % Define second link
L3 = Link('d', 0.3, 'a', 0, 'alpha', 0);        % Define third link
bot = SerialLink([L1 L2 L3], 'name', 'my robot');

%% Create the ground truth
fs = 50; % Sampling frequency

data_slow = load('import_data/exploration_slow_results.mat');
data_fast = load('import_data/exploration_fast_results.mat');

data_size_slow = length(data_slow.watch_tiltAngle_filt);
data_size_fast = length(data_fast.watch_tiltAngle_filt);

traj_repetitions = 15;
all_length_slow = ceil(data_size_slow/(traj_repetitions*2));
all_length_fast = ceil(data_size_fast/(traj_repetitions*2));

[rot_gt_slow, ext_gt_slow, sup_gt_slow] = one_trajectory(all_length_slow);
[rot_gt_fast, ext_gt_fast, sup_gt_fast] = one_trajectory(all_length_fast);

%% Plot just 1 repetition
figure; 

plot(rot_gt_slow, 'color', 'r', LineStyle='-', LineWidth=3, DisplayName='rotation GT'); hold on;
plot(ext_gt_slow, 'color', 'b', LineStyle='-', LineWidth=3,DisplayName='extension GT')
plot(sup_gt_slow, 'color','g', LineStyle='-', LineWidth=3,DisplayName='supination GT')

%xlabel('Sample (f = 50Hz)','FontSize',22); 
ylabel('Joint angles (º)','FontSize',22)
ax = gca; % Get current axes

ax.YAxis.FontSize = 18;

set(gca, 'XTickLabel', []);
% ax.XAxis.FontSize = 16;

xlim([0,length(sup_gt_slow)]); ylim([-100,200])
grid on;
% legend(FontSize=16)
% title(task_title,'FontSize',22)

%% Repeat trajectories 

% Repeat
rot_gt_slow = repmat(rot_gt_slow, [1,traj_repetitions]);
ext_gt_slow = repmat(ext_gt_slow, [1,traj_repetitions]);
sup_gt_slow = repmat(sup_gt_slow, [1,traj_repetitions]);

rot_gt_fast = repmat(rot_gt_slow, [1,traj_repetitions]);
ext_gt_fast = repmat(ext_gt_fast, [1,traj_repetitions]);
sup_gt_fast = repmat(sup_gt_fast, [1,traj_repetitions]);

%% Calculate the tilt angle based on Robot Kinematics

v = [1; 0; 0]; % vector
B = [0 0 1];   % The normal direction (End-Effector pointing upwards). Defined as Z here.

[tiltAngles_fromGT_slow] = get_GT_tiltangle(rot_gt_slow, ext_gt_slow, sup_gt_slow, v, B, data_size_slow, bot);
[tiltAngles_fromGT_fast] = get_GT_tiltangle(rot_gt_fast, ext_gt_fast, sup_gt_fast, v, B, data_size_fast, bot);


%% Tilt angles
% Comparing angles and kurtosis values
tiltAngles_fromWatch_slow = data_slow.watch_tiltAngle_filt;
tiltAngles_fromWatch_fast = data_fast.watch_tiltAngle_filt;

figure; sgtitle(task_title)

subplot(2,1,1); title('Slow'); 
plot(tiltAngles_fromWatch_slow, 'r', 'LineWidth',1.5), hold on; 
plot(tiltAngles_fromGT_slow,'color', 'black', 'LineWidth',1); 
grid on; ylabel('Tilt angles (º)'); legend('watch Slow', 'GT'); 

subplot(2,1,2); title('Fast'); 
plot(tiltAngles_fromWatch_fast, 'b', 'LineWidth',1.5), hold on; 
plot(tiltAngles_fromGT_fast,'color', 'black', 'LineWidth',1); 
grid on; ylabel('Tilt angles (º)'); legend('watch Fast', 'GT'); 

xlabel('Sample (f = 50Hz)'); 

%% Histograms of angles in barplot
figure,
gt_hist_slow = histogram(tiltAngles_fromGT_slow, 36, 'FaceColor','k', 'FaceAlpha', 0.5); hold on; 
gt_hist_fast = histogram(tiltAngles_fromGT_fast, 36, 'FaceColor','k', 'FaceAlpha', 0.5);  
watch_hist_slow = histogram(tiltAngles_fromWatch_slow, 36,'FaceColor','r','FaceAlpha', 0.3), 
watch_hist_fast = histogram(tiltAngles_fromWatch_fast, 36,'FaceColor','b','FaceAlpha', 0.3), 
legend('Ground Truth Slow', 'Ground Truth Fast','Watch slow', 'Watch fast')


%% Histogram in lines
figure; %sgtitle(task_title, 'FontSize', 20)
% Interpolate the plots for better visualization

line_histogram(gt_hist_slow, [0.6350 0.0780 0.1840], ":", 2)
line_histogram(gt_hist_fast,  [0, 0, 0.5451], ":",2)
line_histogram(watch_hist_slow, 'r', "-",4)
line_histogram(watch_hist_fast, 'b', "-",4)

xlabel('Tilt angle (deg)'); xlim([50, 180]);  ylim([1, 2700]); grid on
set(gca, 'FontSize', 16); % Axis ticks and axis labels
set(findall(gcf,'type','text'), 'FontSize', 16); % Title and any other text objects
legend('Ground Truth Slow; k = '+string(round(kurtosis(tiltAngles_fromGT_slow),2)), 'Ground Truth Fast; k = '+string(round(kurtosis(tiltAngles_fromGT_fast),2)),'Watch slow; k = '+string(round(kurtosis(tiltAngles_fromWatch_slow),2)), 'Watch fast; k = '+string(round(kurtosis(tiltAngles_fromWatch_fast),2)))

%% Kurtosis Saturation
figure; title('Slow')
[kurt_cut_gt_slow, kurt_cut_watch_slow] = plot_kurtosis_saturation(tiltAngles_fromGT_slow, tiltAngles_fromWatch_slow, fs, traj_repetitions, all_length_slow, 'r');
figure; title('Fast')
[kurt_cut_gt_fast, kurt_cut_watch_fast] = plot_kurtosis_saturation(tiltAngles_fromGT_fast, tiltAngles_fromWatch_fast, fs, traj_repetitions, all_length_fast, 'b');

% save('exploration_GT_decay_slow.mat', 'kurt_cut_gt_slow');
% save('exploration_watch_decay_slow.mat', 'kurt_cut_watch_slow');
% save('exploration_GT_decay_fast.mat', 'kurt_cut_gt_fast');
% save('exploration_watch_decay_fast.mat', 'kurt_cut_watch_fast');
%% Saturation error
% figure;
% title('Kurtosis Saturation')
% error_ = kurt_cut_watch - kurt_cut_watch(end)
% plot(error); hold on;
% yline(error(end)+0.02)
% yline(error(end)-0.02)

%% Correlation
figure
plot(kurtosis(tiltAngles_fromGT_slow), kurtosis(tiltAngles_fromWatch_slow), 'r*'); hold on;
plot(kurtosis(tiltAngles_fromGT_fast), kurtosis(tiltAngles_fromWatch_fast), 'b*');
hold on; plot(0:0.01:4*pi, 0:0.01:4*pi, 'k')
xlabel('Planned kurtosis'); ylabel('Experimental kurtosis')
xlim([0,10])
grid on
legend('slow','fast')

%% Stats / metrics
[kurt_gt_slow, kurt_watch_slow] = stats(tiltAngles_fromGT_slow, tiltAngles_fromWatch_slow)
[kurt_gt_fast, kurt_watch_fast] = stats(tiltAngles_fromGT_fast, tiltAngles_fromWatch_fast)

%% Functions

function [rot_gt, ext_gt, sup_gt] = one_trajectory(all_length)
    % Rotation 
    rot_gt = [0:90/all_length:90];
    rot_gt = [rot_gt(1:end-1), flip(rot_gt)]
    rot_gt = rot_gt(1:end-1); % dont save the last 0
        
    % Extension
    ext_gt = [0:-90/all_length:-90];
    ext_gt = [ext_gt(1:end-1), flip(ext_gt)]
    ext_gt = ext_gt(1:end-1); % dont save the last 0
   
    % Supination
    sup_gt = [0:180/all_length:180];
    sup_gt = [sup_gt(1:end-1), flip(sup_gt)]
    sup_gt = sup_gt(1:end-1); % dont save the last 0
   
end

function tiltAngles_fromGT = get_GT_tiltangle(rot_gt, ext_gt, sup_gt, v, B, data_size, bot)
    tiltAngles_fromGT = [];
    for i = 1:data_size
        qtrue = [deg2rad(rot_gt(:,i)), deg2rad(ext_gt(:,i)), deg2rad(sup_gt(:,i))];
        Ttrue = bot.fkine(qtrue);           % Apply direct kinematics
        Rtrue = Ttrue.R;                    % Extract the rotation matrix    
        Atrue = Rtrue * v;                  % Rotate the vector original vector of the wrist x direction
        radAngle_gt = acos(dot(Atrue, B) / (norm(Atrue) * norm(B))); % Calculate the angle between A and B (the tilt angle)
        degAngle_gt = rad2deg(radAngle_gt); % Convert the angle from radians to degrees
        tiltAngles_fromGT = [tiltAngles_fromGT,  degAngle_gt];  % Get a list with the joint angles and the tilt angle
    end
end

function [] = line_histogram(H, color, linestyle, width)
    x = [H.BinLimits(1), linspace(H.BinLimits(1), H.BinLimits(2), 36), H.BinLimits(2)];
    % [min(H.BinEdges)+2.5:(max(H.BinEdges)-min(H.BinEdges))/36:max(H.BinEdges)-2.5];
    yq = [0, movmean(H.Values, 5), 0];
    %plot(x, H.Values, color, 'LineWidth',1, 'LineStyle','--'); hold on;     % Plot the histogram
    plot(x, yq, 'Color', color, 'LineWidth', width, 'LineStyle',linestyle); hold on;                                          % Plot the moving average
end



function [kurt_cut_gt, kurt_cut_watch] = plot_kurtosis_saturation(tiltAngles_fromGT, tiltAngles_fromWatch, fs, traj_repetitions, all_length, color)

    kurt_wsize = [1:1:length(tiltAngles_fromWatch)];
    kurt_cut_watch = [];
    kurt_cut_gt = [];
    for k = 1:length(kurt_wsize)
        kurt_cut_watch = [kurt_cut_watch, kurtosis(tiltAngles_fromWatch(1:kurt_wsize(k)))];
        kurt_cut_gt = [kurt_cut_gt, kurtosis(tiltAngles_fromGT(1:kurt_wsize(k)))];
    end
    
    repetitions = [1:traj_repetitions]*(all_length*2);
    time = [kurt_wsize/fs];

    plot(time, kurt_cut_watch, color); hold on; % Plot watch kurtosis
    plot(time, kurt_cut_gt, 'k')                % Plot GT kurtosis
    yline(kurtosis(tiltAngles_fromWatch), color, 'LineStyle','--')       % Kurtosis in steady state for the watch
    yline(kurtosis(tiltAngles_fromGT), 'k--')   % Kurtosis in steady state for the ground truth
    xline(repetitions/fs, 'Color', 'magenta', 'LineStyle', '--')    % End of every repetition

    legend('Watch kurtosis', 'GT kurtosis', 'Watch Steady State kurtosis', 'GT Steady State kurtosis', 'End of repetition')
    xlabel('Time (s)'); ylabel('Kurtosis'); grid on
end

function [kurt_x, kurt_y] = stats(x, y)

    me = mean((x - y));             % ME
    mae = mean(abs(x - y));         % MAE
    mse = mean((x - y).^2);         % MSE
    rmse = sqrt(mean((x - y).^2));  % RMSE
    corr = xcorr(x, y, 'coeff');
    max_corr = max(corr);           % Cross-corr
    R = corrcoef(x,y);
    pears_corr = R(1,2);            % Pearson Correlation Coefficient
    [h,p] = kstest2(x, y)           % Kolmogorov-Smirnov Test
    
    
    mean_x = mean(x)        % Means
    mean_y = mean(y)
    cov_x = cov(x)          % Covs
    cov_y = cov(y)
    skew_x = skewness(x)    % Skeweness
    skew_y = skewness(y)
    kurt_x = kurtosis(x)    % kurtosis
    kurt_y = kurtosis(y)
end