function SolveJointOptVSlamEnc(this, measure, calib, map, options)

%% init

if nargin < 5
    options = [];
end

if ~isfield(options, 'bCalibExtRot')
    options.bCalibExtRot = true;
end
if ~isfield(options, 'bCalibExtLin')
    options.bCalibExtLin = true;
end
if ~isfield(options, 'bCalibEnc')
    options.bCalibEnc = true;
end


disp(['Start calibration with V-SLAM...']);

mk = measure.mk;
odo = measure.odo;
time = measure.time;

% init state vector q ...
idxStMk = options.bCalibExtRot*3 + options.bCalibExtLin*2 ...
    + options.bCalibEnc*3;
q = zeros(idxStMk+3*mk.numMkId+3*odo.num,1);
q_init = [];
count = 1;
if options.bCalibExtRot
    q_init_extrot = calib.rvec_b_c;
    q_init = [q_init; q_init_extrot];
    vecrow_extrot = count:count+2;
    count = count + 3;
end
if options.bCalibExtLin
    q_init_extlin = calib.tvec_b_c(1:2);
    q_init = [q_init; q_init_extlin];
    vecrow_extlin = count:count+1;
    count = count + 2;
end
if options.bCalibEnc
    mat_odo = calib.mat_odo;
    beta_l = mat_odo(2,1);
    beta_r = mat_odo(2,2);
    b = 2*mat_odo(1,2)/mat_odo(2,2);
    q_init_enc = [beta_l; beta_r; b];
    q_init = [q_init; q_init_enc];
    vecrow_enc = count:count+2;
    count = count + 3;
end

q(1:idxStMk) = q_init;
for i = 1:mk.numMkId
    q(idxStMk+i*6-5:idxStMk+i*6-3) = map.mks.rvec_w_m(i,:).';
    q(idxStMk+i*6-2:idxStMk+i*6) = map.mks.tvec_w_m(i,:).';
end
idxStOdo = idxStMk+6*mk.numMkId;
for i = 1:odo.num
    q(idxStOdo+3*i-2: idxStOdo+3*i) = map.kfs.ps2d_w_b(i,:).';
end

%% solve nonlinear least square, mapping and calibration
options_optim = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter', 50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);
[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
    @(x)CostJointOptVSlamEnc(x, mk, odo, time, calib, this.setting, options), q, [], [], options_optim);

%% save results
% refresh calib
if options.bCalibExtRot
    q_extrot = q(vecrow_extrot);
    calib.rvec_b_c = q_extrot;
    calib.RefreshByVecbc;
end
if options.bCalibExtLin
    q_extlin = q(vecrow_extlin);
    calib.tvec_b_c = [q_extlin;0];
    calib.RefreshByVecbc;
end
if options.bCalibEnc
    q_enc = q(vecrow_enc);    
    beta_l = q_enc(1);
    beta_r = q_enc(2);
    b = q_enc(3);
    mat_odo = [-b*beta_l/2, b*beta_r/2; beta_l, beta_r];
    calib.mat_odo = mat_odo;
end


% refresh map
mat_tvec_w_m = zeros(mk.numMkId, 3);
mat_rvec_w_m = zeros(mk.numMkId, 3);
for i = 1:mk.numMkId
    mat_rvec_w_m(i,:) = q(idxStMk+6*i-5:idxStMk+6*i-3).';
    mat_tvec_w_m(i,:) = q(idxStMk+6*i-2:idxStMk+6*i).';
end

vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:odo.num
    vecPs2d_w_b(i,:) = q(idxStOdo+3*i-2:idxStOdo+3*i).';
end

map.mks.rvec_w_m = mat_rvec_w_m;
map.mks.tvec_w_m = mat_tvec_w_m;
map.kfs.ps2d_w_b = vecPs2d_w_b;
map.RefreshKfsByPs2dwb;

disp('End calibration with V-SLAM.');
disp(' ');

function [ vecCost, matJacobian ] = CostJointOptVSlamEnc( q, mk, odo, time, calib, setting, options )
%COST

% error configure
stdErrRatioOdoLin = max(setting.error.odo.stdratio_lin, 1e-3);
stdErrRatioOdoRot = max(setting.error.odo.stdratio_rot, 1e-3);
MinStdErrOdoLin = setting.error.odo.stdmin_lin;
MinStdErrOdoRot = setting.error.odo.stdmin_rot;
stdErrImgU = max(setting.error.mk.std_imgu, 1e-2);
stdErrImgV = max(setting.error.mk.std_imgv, 1e-2);

% vecCost: 8*mk.num + 3*odo.num vector of projection error
vecCost = zeros(8*mk.num + 3*odo.num,1);

%% parse q: rvec_b_c(1:3), tvec_b_c(1:2), vecPt3d_w_m, vecPs2d_w_b
numParam = options.bCalibExtRot*3 + options.bCalibExtLin*2 + options.bCalibEnc*3;
idxStMk = numParam;
q_param = q(1:idxStMk);
count = 1;
if options.bCalibExtRot
    rvec_b_c = q_param(count:count+2);
    vecrow_extrot = count:count+2;
    count = count+3;
else
    rvec_b_c = calib.rvec_b_c;
end
if options.bCalibExtLin
    tvec_b_c = [q_param(count:count+1);0];
    vecrow_extlin = count:count+1;
    count = count+2;
else
    tvec_b_c = calib.tvec_b_c;
end
if options.bCalibEnc
    beta_l = q_param(count);
    beta_r = q_param(count+1);
    b = q_param(count+2);
    mat_odo = [-b*beta_l/2, b*beta_r/2; beta_l, beta_r];
    vecrow_enc = count:count+2;
    count = count + 3;
else
    mat_odo = calib.mat_odo;
    beta_l = mat_odo(2,1);
    beta_r = mat_odo(2,2);
    b = 2*mat_odo(1,2)/beta_r;
end
mat_camera = calib.mat_camera;
vec_distortion = calib.vec_distortion;


[ T3d_b_c, R3d_b_c ] = FunVec2Trans3d( rvec_b_c, tvec_b_c );
T3d_c_b = [R3d_b_c.' -R3d_b_c.'*tvec_b_c; 0 0 0 1];

vec_rvec_w_m = zeros(mk.numMkId, 3);
vec_tvec_w_m = zeros(mk.numMkId, 3);
vec_ps2d_w_b = zeros(odo.num, 3);
for i = 1:mk.numMkId
    vec_rvec_w_m(i,:) = q(idxStMk+i*6-5:idxStMk+i*6-3).';
    vec_tvec_w_m(i,:) = q(idxStMk+i*6-2:idxStMk+i*6).';
end
idxStOdo = idxStMk+6*mk.numMkId;
for i = 1:odo.num
    vec_ps2d_w_b(i,:) = q(idxStOdo+3*i-2: idxStOdo+3*i).';
end
vecDt = zeros(numel(time.lp), 1);
for i = 1:numel(time.lp)
    dtTmp = cnstr2period(time.t_mk(i) - time.t_odo(i), 30, -30);
    vecDt(i,1) = dtTmp;
end
tvec_m_pt1 = setting.aruco.tvec_m_pt1.';
tvec_m_pt2 = setting.aruco.tvec_m_pt2.';
tvec_m_pt3 = setting.aruco.tvec_m_pt3.';
tvec_m_pt4 = setting.aruco.tvec_m_pt4.';

%% Compute Cost Function
% mark observation part: image offset between model and undistorted
% cordinates
mats_std_mk = cell(mk.num,1);
for i = 1:mk.num
    lp = mk.lp(i);
    row_odo = find(odo.lp == lp,1);
    x_w_b = vec_ps2d_w_b(row_odo,1);
    y_w_b = vec_ps2d_w_b(row_odo,2);
    theta_w_b = vec_ps2d_w_b(row_odo,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    [T3d_w_b, R3d_w_b] = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    T3d_b_w = [R3d_w_b.' -R3d_w_b.'*tvec_w_b; 0 0 0 1];
    
    mkId = mk.id(i);
    mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
    
    rvec_w_m = vec_rvec_w_m(mkIdOrd,:).';
    tvec_w_m = vec_tvec_w_m(mkIdOrd,:).';
    T3d_w_m = FunVec2Trans3d(rvec_w_m, tvec_w_m);
    
    T3d_c_m = T3d_c_b*T3d_b_w*T3d_w_m;
    
    [ rvec_c_m, tvec_c_m ] = FunTrans2Vec3d( T3d_c_m );
    
    img_c_pt1_measure = mk.pt1(i,:).';
    img_c_pt2_measure = mk.pt2(i,:).';
    img_c_pt3_measure = mk.pt3(i,:).';
    img_c_pt4_measure = mk.pt4(i,:).';
    
    img_c_pt1 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt1, mat_camera, vec_distortion );
    img_c_pt2 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt2, mat_camera, vec_distortion );
    img_c_pt3 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt3, mat_camera, vec_distortion );
    img_c_pt4 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt4, mat_camera, vec_distortion );
    
    errimg_c_pt1 = img_c_pt1 - img_c_pt1_measure;
    errimg_c_pt2 = img_c_pt2 - img_c_pt2_measure;
    errimg_c_pt3 = img_c_pt3 - img_c_pt3_measure;
    errimg_c_pt4 = img_c_pt4 - img_c_pt4_measure;
    
    vec_err_temp = [errimg_c_pt1; errimg_c_pt2; errimg_c_pt3; errimg_c_pt4];
    
    mat_std = diag([stdErrImgU stdErrImgV stdErrImgU stdErrImgV stdErrImgU stdErrImgV stdErrImgU stdErrImgV]);
    mats_std_mk{i} = mat_std;
    vecCost(8*i-7: 8*i) = inv(mat_std)*(vec_err_temp);
end

% odometry part
idStOdoOutput = 8*mk.num;
mats_std_odo = cell(odo.num, 1);
for i = 1:odo.num
    if i == 1
        x_w_b1 = 0; y_w_b1 = 0; theta_w_b1 = 0;
        ps2d_b1_b2_odo = [0;0;0];
    else
        x_w_b1 = vec_ps2d_w_b(i-1,1);
        y_w_b1 = vec_ps2d_w_b(i-1,2);
        theta_w_b1 = vec_ps2d_w_b(i-1,3);
        %         ps2d_w_b1_odo = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
        %         ps2d_w_b2_odo = [odo.x(i);odo.y(i);odo.theta(i)];
        %         ps2d_b1_b2_odo = FunRelPos2d(ps2d_w_b1_odo, ps2d_w_b2_odo);
        enc_l_b1 = odo.enc_l(i-1);
        enc_r_b1 = odo.enc_r(i-1);
        enc_l_b2 = odo.enc_l(i);
        enc_r_b2 = odo.enc_r(i);
        d_enc_l = enc_l_b2 - enc_l_b1;
        d_enc_r = enc_r_b2 - enc_r_b1;
        ps2d_b1_b2_odo = FunEnc2Ps2d(d_enc_l, d_enc_r, mat_odo);
    end
    
    x_w_b2 = vec_ps2d_w_b(i,1);
    y_w_b2 = vec_ps2d_w_b(i,2);
    theta_w_b2 = vec_ps2d_w_b(i,3);
    
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    ps2d_b1_b2_mod = FunRelPos2d(ps2d_w_b1, ps2d_w_b2);
    ps2d_b1_b2_mod(3) = FunPrdCnst(ps2d_b1_b2_mod(3), ps2d_b1_b2_odo(3)+pi, ps2d_b1_b2_odo(3)-pi);
    
    std_trans = max(norm(ps2d_b1_b2_odo(1:2))*stdErrRatioOdoLin, MinStdErrOdoLin);
    std_rot = max(abs(ps2d_b1_b2_odo(3))*stdErrRatioOdoRot, MinStdErrOdoRot);
    
    mat_std = diag([std_trans;std_trans;std_rot]);
    mats_std_odo{i} = mat_std;
    
    vecCost(idStOdoOutput+3*i-2:idStOdoOutput+3*i) = inv(mat_std)*(ps2d_b1_b2_mod-ps2d_b1_b2_odo);
end


%% Compute Jacobian
if nargout > 1
    % allocate J
    matJacobian = zeros(8*mk.num+3*odo.num, numel(q));
    
    % calculate J of mark observation
    for i = 1:mk.num
        lp = mk.lp(i);
        row_odo = find(odo.lp == lp,1);
        mkId = mk.id(i);
        mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
        
        rvec_w_m = vec_rvec_w_m(mkIdOrd,:).';
        tvec_w_m = vec_tvec_w_m(mkIdOrd,:).';
        
        tvec_w_b = [vec_ps2d_w_b(row_odo,1); vec_ps2d_w_b(row_odo,2); 0];
        rvec_w_b = [0;0;vec_ps2d_w_b(row_odo,3)];
        
        [J_pt1_bc, J_pt1_wb, J_pt1_wm, J_pt1_cam] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt1, mat_camera, vec_distortion);
        [J_pt2_bc, J_pt2_wb, J_pt2_wm, J_pt2_cam] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt2, mat_camera, vec_distortion);
        [J_pt3_bc, J_pt3_wb, J_pt3_wm, J_pt3_cam] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt3, mat_camera, vec_distortion);
        [J_pt4_bc, J_pt4_wb, J_pt4_wm, J_pt4_cam] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt4, mat_camera, vec_distortion);
        
        J_pt_bc = [J_pt1_bc; J_pt2_bc; J_pt3_bc; J_pt4_bc];
        J_pt_wb = [J_pt1_wb; J_pt2_wb; J_pt3_wb; J_pt4_wb];
        J_pt_wm = [J_pt1_wm; J_pt2_wm; J_pt3_wm; J_pt4_wm];
        J_pt_cam = [J_pt1_cam; J_pt2_cam; J_pt3_cam; J_pt4_cam];
        
        vecrow_jac = i*8-7:i*8;
        veccol_jac_wb = idxStOdo+3*row_odo-2:idxStOdo+3*row_odo;
        veccol_jac_wm = idxStMk+6*mkIdOrd-5:idxStMk+6*mkIdOrd;
        
        mat_std = mats_std_mk{i};
        
        matJacobian(vecrow_jac, veccol_jac_wb) = inv(mat_std)*J_pt_wb;
        matJacobian(vecrow_jac, veccol_jac_wm) = inv(mat_std)*J_pt_wm;
        
        if options.bCalibExtRot
            veccol_jac_bcrot = vecrow_extrot;
            matJacobian(vecrow_jac, veccol_jac_bcrot) = inv(mat_std)*J_pt_bc(:,1:3);
        end
        if options.bCalibExtLin
            veccol_jac_bclin = vecrow_extlin;
            matJacobian(vecrow_jac, veccol_jac_bclin) = inv(mat_std)*J_pt_bc(:,4:5);
        end        
    end
    
    % calculate J of odometry
    row_st = 8*mk.num;
    mat_std = mats_std_odo{1};
    matJacobian(row_st+1:row_st+3, idxStOdo+1:idxStOdo+3) = inv(mat_std);
    for i = 2:odo.num
        ps_w_b1 = vec_ps2d_w_b(i-1,:);
        ps_w_b2 = vec_ps2d_w_b(i,:);
        [J1, J2] = FunJacobianRelPs2d(ps_w_b1, ps_w_b2);
        mat_std = mats_std_odo{i};
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-5:idxStOdo+3*i-3) = inv(mat_std)*J1;
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-2:idxStOdo+3*i) = inv(mat_std)*J2;
        
        % odometric matrix part, TODO!
        if options.bCalibEnc
            J_beta_l = [ ...
                (d_enc_l*sin((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2)*((b*beta_l*d_enc_l)/2 - (b*beta_r*d_enc_r)/2))/2 - (b*d_enc_l*cos((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2))/2; ...
                - (d_enc_l*cos((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2)*((b*beta_l*d_enc_l)/2 - (b*beta_r*d_enc_r)/2))/2 - (b*d_enc_l*sin((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2))/2; ...
                d_enc_l];
            
            J_beta_r = [ ...
                (b*d_enc_r*cos((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2))/2 + (d_enc_r*sin((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2)*((b*beta_l*d_enc_l)/2 - (b*beta_r*d_enc_r)/2))/2; ...
                (b*d_enc_r*sin((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2))/2 - (d_enc_r*cos((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2)*((b*beta_l*d_enc_l)/2 - (b*beta_r*d_enc_r)/2))/2; ...
                d_enc_r];
            
            J_b = [ ...
                -cos((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2)*((beta_l*d_enc_l)/2 - (beta_r*d_enc_r)/2); ...
                -sin((beta_l*d_enc_l)/2 + (beta_r*d_enc_r)/2)*((beta_l*d_enc_l)/2 - (beta_r*d_enc_r)/2); ...
                0];
            J_enc = [J_beta_l, J_beta_r, J_b];
            matJacobian(row_st+3*i-2:row_st+3*i, vecrow_enc) = -inv(mat_std)*J_enc;
        end
    end
end



