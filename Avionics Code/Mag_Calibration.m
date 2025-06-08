% Load data
data = readmatrix('LSM-Mag_data.csv');  % or use readtable for named columns

% Extract raw data columns
RX = data(:, 1);
RY = data(:, 2);
RZ = data(:, 3);

%Magcal format
M = [RX, RY, RZ];

%Calculate callibations
[A, B] = magcal(M);

M_calibrated = (M-B)*A;

% 3D scatter plot
subplot(1,2,1);  % 1 row, 2 columns, position 1
scatter3(RX, RY, RZ, 25, "red", 'filled');  % Color by Z
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Uncalibrated');
grid on;
axis equal;

subplot(1,2,2);  % 1 row, 2 columns, position 1
scatter3(M_calibrated(:,1), M_calibrated(:,2), M_calibrated(:,3), 25, "red", 'filled');  % Color by Z
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Calibration');
grid on;
axis equal;