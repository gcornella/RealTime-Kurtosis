clear all; close all; clc;

shuffling_cards_gt_slow =  1.8;
shuffling_cards_gt_fast = 1.8003;
shuffling_cards_watch_slow = 1.7187;
shuffling_cards_watch_fast = 1.4616;

cup_stacking_gt_slow = 3.9837;
cup_stacking_gt_fast = 3.9958;
cup_stacking_watch_slow =  3.3378;
cup_stacking_watch_fast = 5.0882;

arm_wrestling_gt_slow = 1.8008;
arm_wrestling_gt_fast = 1.8004;
arm_wrestling_watch_slow =  1.6731;
arm_wrestling_watch_fast =  1.3626;

handshaking_gt_slow = 7.1879;
handshaking_gt_fast = 4.6240;
handshaking_watch_slow = 5.4034;
handshaking_watch_fast = 4.1182;

exploration_gt_slow = 2.2906;
exploration_gt_fast = 2.2826;
exploration_watch_slow = 1.9055;
exploration_watch_fast = 1.7951 ;

simulated_normal_gt_slow =  2.9178;
simulated_normal_gt_fast = 2.9294;
simulated_normal_watch_slow = 2.7838;
simulated_normal_watch_fast = 2.4831;

%% % Linear regression
gt_slow = [shuffling_cards_gt_slow, cup_stacking_gt_slow, arm_wrestling_gt_slow, handshaking_gt_slow, exploration_gt_slow, simulated_normal_gt_slow]';
gt_fast = [shuffling_cards_gt_fast, cup_stacking_gt_fast, arm_wrestling_gt_fast, handshaking_gt_fast, exploration_gt_fast, simulated_normal_gt_fast]';

watch_slow = [shuffling_cards_watch_slow, cup_stacking_watch_slow, arm_wrestling_watch_slow, handshaking_watch_slow, exploration_watch_slow, simulated_normal_watch_slow]';
watch_fast = [shuffling_cards_watch_fast, cup_stacking_watch_fast, arm_wrestling_watch_fast, handshaking_watch_fast, exploration_watch_fast, simulated_normal_watch_fast]';

% Excess kurtosis
gt_slow = gt_slow-3;
gt_fast = gt_fast-3;

watch_slow = watch_slow-3;
watch_fast = watch_fast-3;

%% Correlation

figure
plot(gt_slow, watch_slow, 'r*', 'MarkerSize',10); hold on;
plot(gt_fast, watch_fast, 'b*', 'MarkerSize',10); 

[m_slow, n_slow, r2_slow, p_val_slow] = fit_slow(gt_slow, watch_slow, 'r')
[m_fast, n_fast, r2_fast, p_val_fast] = fit_slow(gt_fast, watch_fast, 'b')

% Plot a line y=x
plot(0:0.01:8, 0:0.01:8, 'k')

title('Kurtosis')
xlabel('Planned kurtosis'); ylabel('Measured kurtosis'); 
xlim([-2 , 5]); grid on;

set(gca, 'FontSize', 18); % Axis ticks and axis labels
set(findall(gcf,'type','text'), 'FontSize', 18); % Title and any other text objects

legend('slow data points; r^2 = '+string(round(r2_slow,2))+ '; p = '+string(round(p_val_slow,4)), 'fast data points; r^2 = '+string(round(r2_fast,2))+ '; p = '+string(round(p_val_fast,4)),'slow fit: y = '+string(round(m_slow,2))+'x '+string(round(n_slow,2)), 'fast fit: y = '+string(round(m_fast,2))+'x '+string(round(n_fast,2)),'')


%%
function [m, n, r2, p_val] = fit_slow(x_gt, y_watch, color)
    % Fit the data
    mdl = fitlm(x_gt, y_watch)

    % Predict y values using the fitted model
    y_fit = predict(mdl, x_gt);  
    
    % Plot the fitted line
    plot(x_gt, y_fit, 'Color',color, 'LineWidth', 4);  
    
    % Calculate the correlation coefficient between the original data and the fit line
    [R, P] = corrcoef(x_gt, y_watch);
    pears_corr = R(1,2);
    r2 = pears_corr^2;
    p_val = P(1,2);

    % Get the slope and intercept from the model
    m = mdl.Coefficients.Estimate(2); % slope
    n = mdl.Coefficients.Estimate(1); % intercept
end