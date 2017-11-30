clear;
close all;

%% manually set configures

% please manually set these variables ...

% path of configure file
path_setting_file = './sample/setting_dataset_sample_1.yml';

% bUseAutoInit: set true if use auto init method, fale if manually set
% initial guess
bUseAutoInit = false;


% bUseMSlam: use mslam method if set true
% require noise configure setting.mk.stdratio_x(y,z) from configure file
% require landmark measurement measure.mk.tvec
bUseMSlam = false;


% bUseVslam: use vlsma method if set true
% require noise configure: setting.std_imgu(v)
% require landmark info: setting.aruco
% require camera intrinsic param.: setting.camera
bUseVSlam = true;


% bDrawSlamRes: if draw slam results
bDrawSlamRes = true;

% please **DO NOT** change these variables !!!
bUseEnc = false;
bKeepAllMk = true;


%% init: create class objs and load data

% load configure
setting = YAML.read(path_setting_file);

% measure
measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
measure.ReadRecData;
measure_raw = ClassMeasure();
measure.CopyTo(measure_raw);
measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot, bUseEnc, bKeepAllMk);
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time);
measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time);

% solver
solver = ClassSolverSlam(setting);

% map
map = ClassMap;

% calib
calib = ClassCalib(setting);


%% do autoinit
if bUseAutoInit
    solver.DoAutoInitOdo(measure, calib);
    disp('after auto init.');
    calib.DispCalib;
end


%% init map
map.InitMap(measure, calib);


%% do joint opt with landmark measurement
if bUseMSlam
    options_mslam = struct(...
        'bCalibExtRot', true, 'bCalibExtLin', true,...
        'bCalibTmp', false, 'bCalibOdo', false);
    solver.SolveJointOptMSlam(measure, calib, map, options_mslam);
    disp('after v-joint-opt.');
    calib.DispCalib;
end


%% do joint opt with visual measurement
if bUseVSlam
    options_vslam = struct(...
        'bCalibExtRot', true, 'bCalibExtLin', true,...
        'bCalibTmp', false, 'bCalibOdo', false);
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    disp('after v-joint-opt.');
    calib.DispCalib;
end


%% draw slam results
if bDrawSlamRes
    options_drawmap = struct('strTitle', 'SLAM results', 'fileNameFigOut', '.\temp\opt', ...
        'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 1.5);
    map.DrawMap(measure, calib, setting, options_drawmap);
end











