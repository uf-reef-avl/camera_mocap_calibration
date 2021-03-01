clear;
clc;
addpath('quaternions');
calib_transforms;
%calib_transforms_no_outliers;
%tf_cam_to_rgb_optical_XTion_12_21_11_41_17;
%tf_cam_to_rgb_optical_XTion_12_17_14_48_43a;
%tf_cam_to_rgb_optical_XTion_12_17_14_48_43b;
%tf_cam_to_rgb_optical_XTion_5_4_2016;
calib_data = tf_cam_to_rgb_optical_calibration_data;
%calib_data = calib_data(1:100,:);
[rows,cols]=size(calib_data);
t_estim = calib_data(:, 1:3);
q_estim = calib_data(:, 4:7);
for rowidx=2:rows,
    R_estim_prev = qGetR([q_estim(rowidx-1,4), q_estim(rowidx-1,1:3)]);
    t_estim_prev = t_estim(rowidx-1,1:3);
    Tx_estim_prev = [R_estim_prev, t_estim_prev'; 0 0 0 1];
    R_estim_cur = qGetR([q_estim(rowidx,4), q_estim(rowidx,1:3)]);
    t_estim_cur = t_estim(rowidx,1:3);
    Tx_estim_cur = [R_estim_cur, t_estim_cur'; 0 0 0 1];
    Tx_estim_change = Tx_estim_cur*inv(Tx_estim_prev);
    qval = q_estim(rowidx,:);
    yaw = atan2(2*(qval(1)*qval(2)+qval(3)*qval(4)),1-2*(qval(2)*qval(2)+qval(3)*qval(3)));
    pitch = asin(2*(qval(1)*qval(3)-qval(2)*qval(4)));
    roll = atan2(2*(qval(1)*qval(4)+qval(2)*qval(3)),1-2*(qval(3)*qval(3)+qval(4)*qval(4)));
    if (abs(yaw+pi)<30*pi/180)
        yaw = yaw+2*pi;
    end
    t_vec_estim(rowidx-1,:) = t_estim(rowidx,:)';
    alpha_vec_estim(rowidx-1,:) = (180/pi)*[roll pitch yaw];
    dt_vec_estim(rowidx-1,:) = Tx_estim_change(1:3,4)';
    dalpha_vec_estim(rowidx-1,:) = [-Tx_estim_change(2,3), Tx_estim_change(1,3), -Tx_estim_change(1,2)];
end
T_data=[t_vec_estim(:,1), alpha_vec_estim(:,1), ...
    t_vec_estim(:,2), alpha_vec_estim(:,2), ...
    t_vec_estim(:,3), alpha_vec_estim(:,3)];
T_data_labelvec={{'x position (m)'},{'\phi, roll (degrees)'}, ...
    {'y position (m)'},{'\theta, pitch (degrees)'}, ...
    {'z position (m)'},{'\psi, yaw (degrees)'}};
dT_data=[dt_vec_estim(:,1), dalpha_vec_estim(:,1), ...
    dt_vec_estim(:,2), dalpha_vec_estim(:,2), ...
    dt_vec_estim(:,3), dalpha_vec_estim(:,3)];
dT_data_labelvec={{'\Deltax, position (m)'},{'\Delta\phi, roll (degrees)'}, ...
    {'\Deltay, position (m)'}, {'\Delta\theta, pitch (degrees)'}, ...
    {'\Deltaz, position (m)'},{'\Delta\psi, yaw (degrees)'}};
mean_vals=mean(T_data)
std_dev_vals=std(T_data)
tcalib = mean(t_estim);
qcalib = mean(q_estim)/norm(mean(q_estim));
%fprintf(1,'Tran_xyz=[%7f %7f %7f]\n',tcalib(1:3));
fprintf(1,'Tran_xyz=[%.16f %.16f %.16f]\n',tcalib(1:3));
%fprintf(1,'Quat_xyzw=[%7f %7f %7f %7f]\n',qcalib(1:4));
fprintf(1,'Quat_xyzw=[%.16f %.16f %.16f %.16f]\n',qcalib(1:4));
for plotidx=1:6,
    figure(1), subplot(3,2,plotidx), hold off, plot(T_data(:,plotidx)), ...
        ylabel(T_data_labelvec{plotidx}), , xlabel('sample index');
end

for plotidx=1:6,
    figure(2), subplot(3,2,plotidx), hold off, plot(dT_data(:,plotidx)), ...
        ylabel(dT_data_labelvec{plotidx}), xlabel('sample index');
end
