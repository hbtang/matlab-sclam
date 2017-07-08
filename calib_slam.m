%% init: create class objs and load data
clear;
close all;

% load configure
setting = YAML.read('setting-slam-sim-ceiling.yml');

% measure
measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
measure.ReadRecData;
measure_raw = ClassMeasure();
measure.CopyTo(measure_raw);
measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time);
measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time);

% solver
solver = ClassSolverSlam(setting);

% map
map = ClassMap;

% calib
calib = ClassCalib(setting);


%% do auto init with encoder
solver.DoAutoInitEnc(measure, calib);
disp('after auto init.')
calib.DispCalib;
measure_from_enc = solver.Enc2Odo(measure, calib);

%% init map from auto init
map.InitMap(measure_from_enc, calib);
% options_drawmap = struct('strTitle', 'title', 'fileNameFigOut', '.\temp\init', ...
%     'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
% map.DrawMap(measure_from_enc, calib, setting, options_drawmap);


%% do joint opt with encoder
options_vslam = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibEnc', true);
solver.SolveJointOptVSlamEnc(measure_from_enc, calib, map, options_vslam);
disp('after v-joint-opt.')
calib.DispCalib;
% options_drawmap = struct('strTitle', 'title', 'fileNameFigOut', '.\temp\opt', ...
%     'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
% map.DrawMap(measure_from_enc, calib, setting, options_drawmap);

return;


%% calib with v-slam
% % do v-slam-calib
% options_vslam = struct(...
%     'bCalibExtRot', true, 'bCalibExtLin', true,...
%     'bCalibTmp', true, 'bCalibOdo', true, ...
%     'bCalibCamMat', false, 'bCalibCamDist', false);
% solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
% % display v-slam-calib results
% calib.DispCalib;
% % draw v-slam results
% options_drawmap = struct('strTitle', 'v-SLAM Result: v-Calib.', 'fileNameFigOut', '.\temp\vslam-all', ...
%     'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
% map.DrawMap(measure, calib, setting, options_drawmap);
% % compute v-slam error
% options_errvslam = struct('bCalibTmp', true, 'bCalibOdo', true);
% struct_errvslam_vcalib = Err_vSlam( measure, calib, map, setting, options_errvslam );

%% Draw Error
% % to be blocked...
% figure; grid on; axis equal; hold on;
% plot(struct_errvslam_init.mat_errImg(:,1), struct_errvslam_init.mat_errImg(:,2), '.', 'Color', 'b');
% plot(struct_errvslam_init.mat_errImg(:,3), struct_errvslam_init.mat_errImg(:,4), '.', 'Color', 'g');
% plot(struct_errvslam_init.mat_errImg(:,5), struct_errvslam_init.mat_errImg(:,6), '.', 'Color', 'r');
% plot(struct_errvslam_init.mat_errImg(:,7), struct_errvslam_init.mat_errImg(:,8), '.', 'Color', 'c');
%
% % figure; grid on; axis equal; hold on;
% % plot(struct_errvslam_mcalib.mat_errImg(:,1), struct_errvslam_mcalib.mat_errImg(:,2), '.', 'Color', 'b');
% % plot(struct_errvslam_mcalib.mat_errImg(:,3), struct_errvslam_mcalib.mat_errImg(:,4), '.', 'Color', 'g');
% % plot(struct_errvslam_mcalib.mat_errImg(:,5), struct_errvslam_mcalib.mat_errImg(:,6), '.', 'Color', 'r');
% % plot(struct_errvslam_mcalib.mat_errImg(:,7), struct_errvslam_mcalib.mat_errImg(:,8), '.', 'Color', 'c');
%
% figure; grid on; axis equal; hold on;
% plot(struct_errvslam_vcalib.mat_errImg(:,1), struct_errvslam_vcalib.mat_errImg(:,2), '.', 'Color', 'b');
% plot(struct_errvslam_vcalib.mat_errImg(:,3), struct_errvslam_vcalib.mat_errImg(:,4), '.', 'Color', 'g');
% plot(struct_errvslam_vcalib.mat_errImg(:,5), struct_errvslam_vcalib.mat_errImg(:,6), '.', 'Color', 'r');
% plot(struct_errvslam_vcalib.mat_errImg(:,7), struct_errvslam_vcalib.mat_errImg(:,8), '.', 'Color', 'c');

%% Debug








