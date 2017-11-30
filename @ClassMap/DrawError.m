function DrawError(this, measure, calib, setting, options)
%DRAWERROR 此处显示有关此函数的摘要
%   此处显示详细说明

if ~isfield(options, 'str_algorithm')
    options.str_algorithm = 'temp';
end

%% compute error
struct_errvslam = Err_vSlam( measure, calib, this, setting);

%% remove outliers
struct_errvslam.mat_errPts = ...
    RemoveOutliers(struct_errvslam.mat_errPts, 5, -1e6, 1e6);

%% print rmse to console
rmse_odo = rms(struct_errvslam.mat_errOdo);
rmse_img = rms(struct_errvslam.mat_errPts);

disp(['rmse with algorithm: ', options.str_algorithm]);
disp(['rmse odo:', ...
    ' rmse_x = ',num2str(rmse_odo(1)),...
    ' rmse_y = ',num2str(rmse_odo(2)),...
    ' rmse_theta = ',num2str(rmse_odo(3))]);
disp(['rmse img:', ...
    ' rmse_u = ',num2str(rmse_img(1)),...
    ' rmse_v = ',num2str(rmse_img(2))]);

lim_ratio = 3;

%% draw and save image error
figure;
set(gcf, 'Position', [800,1,640,480]);

hold on; grid on; axis equal;
title(['Scatter Image Error UV: Algorithm ', options.str_algorithm], 'FontWeight','bold');
xlabel('Image Error U (pixel)');
ylabel('Image Error V (pixel)');
box on;
% set(gca, 'xlim', [-lim_ratio*rmse_img(1), lim_ratio*rmse_img(1)]);
set(gca, 'ylim', [-lim_ratio*rmse_img(2), lim_ratio*rmse_img(2)]);

plot(struct_errvslam.mat_errPts(:,1), struct_errvslam.mat_errPts(:,2), '.');

print(['./temp/err-', options.str_algorithm, '-imguv'], '-depsc', '-r0');
print(['./temp/err-', options.str_algorithm, '-imguv'], '-djpeg', '-r0');


%% draw and save odo error xy
figure;
set(gcf, 'Position', [800,1,640,480]);

hold on; grid on; axis equal;
title(['Scatter Odometry Error XY: Algorithm ', options.str_algorithm], 'FontWeight','bold');
xlabel('Odometry Error X (mm)');
ylabel('Odometry Error Y (mm)');
box on;

set(gca, 'ylim', [-lim_ratio*rmse_odo(2), lim_ratio*rmse_odo(2)]);

plot(struct_errvslam.mat_errOdo(:,1), struct_errvslam.mat_errOdo(:,2), '.');

print(['./temp/err-', options.str_algorithm, '-odoxy'], '-depsc', '-r0');
print(['./temp/err-', options.str_algorithm, '-odoxy'], '-djpeg', '-r0');


%% draw and save image error
figure;
set(gcf, 'Position', [800,1,640,480]);

hold on; grid on;
title(['Scatter Odometry Error XTheta: Algorithm ', options.str_algorithm], 'FontWeight','bold');
xlabel('Odometry Error X (mm)');
ylabel('Odometry Error Theta (rad)');
box on;

set(gca, 'xlim', [-lim_ratio*rmse_odo(1), lim_ratio*rmse_odo(1)]);
set(gca, 'ylim', [-lim_ratio*rmse_odo(3), lim_ratio*rmse_odo(3)]);

plot(struct_errvslam.mat_errOdo(:,1), struct_errvslam.mat_errOdo(:,3), '.');

print(['./temp/err-', options.str_algorithm, '-odoxtheta'], '-depsc', '-r0');
print(['./temp/err-', options.str_algorithm, '-odoxtheta'], '-djpeg', '-r0');




end

