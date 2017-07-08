function SolveJointOptMSlamEnc(this, measure, calib, map, options)
%SOLVERJOINTOPTMSLAMENC 此处显示有关此函数的摘要
%   此处显示详细说明

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

disp(['Start calibration with M-SLAM...']);

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
    q(idxStMk+i*3-2:idxStMk+i*3) = map.mks.tvec_w_m(i,:).';
end
idxStOdo = idxStMk+3*mk.numMkId;
for i = 1:odo.num
    q(idxStOdo+3*i-2: idxStOdo+3*i) = map.kfs.ps2d_w_b(i,:).';
end

%% solve nonlinear least square, mapping and calibration
options_optim = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter', 50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);
[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
    @(x)CostJointOptMSlamEnc(x, mk, odo, time, calib, this.setting, options), q, [], [], options_optim);

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
for i = 1:mk.numMkId
    mat_tvec_w_m(i,:) = q(idxStMk+3*i-2:idxStMk+3*i).';
end

vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:odo.num
    vecPs2d_w_b(i,:) = q(idxStOdo+3*i-2:idxStOdo+3*i).';
end

map.mks.tvec_w_m = mat_tvec_w_m;
map.kfs.ps2d_w_b = vecPs2d_w_b;
map.RefreshKfsByPs2dwb;

disp('End calibration with M-SLAM.');
disp(' ');

%% cost function
function [ vecCost, matJacobian ] = CostJointOptMSlamEnc( q, mk, odo, time, calib, setting, options )
%COST

stdErrRatioMkX = setting.error.mk.stdratio_x;
stdErrRatioMkY = setting.error.mk.stdratio_y;
stdErrRatioMkZ = setting.error.mk.stdratio_z;
stdErrRatioOdoLin = setting.error.odo.stdratio_lin;
stdErrRatioOdoRot = setting.error.odo.stdratio_rot;
MinStdErrOdoLin = setting.error.odo.stdmin_lin;
MinStdErrOdoRot = setting.error.odo.stdmin_rot;

% vecCost: 3*mk.num + 3*odo.num vector of projection error
vecCost = zeros(3*mk.num + 3*odo.num,1);

%% parse q

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

[ T3d_b_c, R3d_b_c ] = FunVec2Trans3d( rvec_b_c, tvec_b_c );
T3d_c_b = [R3d_b_c.' -R3d_b_c.'*tvec_b_c; 0 0 0 1];

vec_tvec_w_m = zeros(mk.numMkId, 3);
vec_ps2d_w_b = zeros(odo.num, 3);

for i = 1:mk.numMkId
    vec_tvec_w_m(i,:) = q(idxStMk+i*3-2:idxStMk+i*3).';
end
idxStOdo = idxStMk+3*mk.numMkId;
for i = 1:odo.num
    vec_ps2d_w_b(i,:) = q(idxStOdo+3*i-2: idxStOdo+3*i).';
end
vecDt = zeros(numel(time.lp), 1);
for i = 1:numel(time.lp)
    dtTmp = cnstr2period(time.t_mk(i) - time.t_odo(i), 30, -30);
    vecDt(i,1) = dtTmp;
end

%% calculate cost function F
% mark observation part
for i = 1:mk.num
    lp = mk.lp(i);
    lpOdo = find(odo.lp == lp,1);
    x_w_b = vec_ps2d_w_b(lpOdo,1);
    y_w_b = vec_ps2d_w_b(lpOdo,2);
    theta_w_b = vec_ps2d_w_b(lpOdo,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    T3d_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    
    tvec_c_m = mk.tvec(i,:).';
    mkId = mk.id(i);
    mkid_ord_1 = find(mk.vecMkId(:,1) == mkId, 1);
    tvec_w_m_model = vec_tvec_w_m(mkid_ord_1,:).';
    
    T3d_w_c = T3d_w_b*T3d_b_c;
    tvec_w_m = T3d_w_c(1:3,1:3)*tvec_c_m + T3d_w_c(1:3,4);
    
    dist = norm(tvec_c_m);
    std_z_c2 = max(dist*stdErrRatioMkZ, 0.001);
    std_x_c2 = max(dist*stdErrRatioMkX, 0.001);
    std_y_c2 = max(dist*stdErrRatioMkY, 0.001);
    mat_std_c2 = diag([std_x_c2;std_y_c2;std_z_c2]);
    R3d_c_w = inv(T3d_w_c(1:3,1:3));
    R3d_c_c2 = FunRotPt2ZAxle(tvec_c_m);
    R3d_c2_w = inv(R3d_c_c2)*R3d_c_w;
    mat_std = mat_std_c2*R3d_c2_w;
    
    mats_std_mk{i} = mat_std;
    
    vecCost(3*i-2: 3*i) = inv(mat_std)*(tvec_w_m_model-tvec_w_m);
end

% odometry part
idStOdoOutput = 3*mk.num;

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

% map part, to determine scale, constraint of the distance between mks
% map_pre = setting.map;
% size_map_pre = size(map_pre);
% row_map_pre = size_map_pre(1);
% if row_map_pre ~= 2
%     error('Error, no map info!');
% end
% 
% mkid_1 = map_pre(1,1);
% tvec_w_m1_pre = map_pre(1,5:7).';
% mkid_2 = map_pre(2,1);
% tvec_w_m2_pre = map_pre(2,5:7).';
% dist_pre = norm(tvec_w_m2_pre - tvec_w_m1_pre);
% 
% mkid_ord_1 = find(mk.vecMkId(:,1) == mkid_1, 1);
% tvec_w_m1_model = vec_tvec_w_m(mkid_ord_1,:).';
% mkid_ord_2 = find(mk.vecMkId(:,1) == mkid_2, 1);
% tvec_w_m2_model = vec_tvec_w_m(mkid_ord_2,:).';
% dist_model = norm(tvec_w_m1_model-tvec_w_m2_model);
% 
% cost_map = dist_model*dist_model - dist_pre*dist_pre;
% vecCost(end) = cost_map;



%% calculate Jacobian of F
if nargout > 1
    % allocate J
    matJacobian = zeros(3*mk.num+3*odo.num, numel(q));
    
    % compute differentiation of R3d_b_c to rvec_b_c
    [ d_R3d_rvec_b_c_1, d_R3d_rvec_b_c_2, d_R3d_rvec_b_c_3 ] = DiffRotvec2Rotmat( rvec_b_c );
    
    % calculate J of mark observation
    for i = 1:mk.num
        lp = mk.lp(i);
        lpOdo = find(odo.lp == lp,1);
        mkId = mk.id(i);
        mkid_ord_1 = find(mk.vecMkId(:,1) == mkId, 1);
        
        theta_w_b = vec_ps2d_w_b(lpOdo,3);
        R3d_w_b = rodrigues([0;0;theta_w_b]);
        
        tvec_c_m = mk.tvec(i,:).';
        
        mat_std = mats_std_mk{i};
        
        % Jacoian on camera extrinsic ps2d_b_cg
        if options.bCalibExtRot
            J_r = zeros(3,3);
            J_r(1:3, 1) = -R3d_w_b*d_R3d_rvec_b_c_1*tvec_c_m;
            J_r(1:3, 2) = -R3d_w_b*d_R3d_rvec_b_c_2*tvec_c_m;
            J_r(1:3, 3) = -R3d_w_b*d_R3d_rvec_b_c_3*tvec_c_m;
            matJacobian(3*i-2:3*i, vecrow_extrot) = inv(mat_std)*J_r;
        end
        if options.bCalibExtLin
            J_l = -R3d_w_b(1:3,1:2);
            matJacobian(3*i-2:3*i, vecrow_extlin) = inv(mat_std)*J_l;
        end
        
        % Jacobian on marker position pt3d_w_m
        matJacobian(3*i-2:3*i, 3*mkid_ord_1+idxStMk-2:3*mkid_ord_1+idxStMk) = inv(mat_std)*eye(3);
        
        %Jacobian on robot pose ps2d_w_b
        tvec_b_m = T3d_b_c(1:3,1:3)*tvec_c_m + T3d_b_c(1:3,4);
        R2d_w_b = [cos(theta_w_b) -sin(theta_w_b); sin(theta_w_b) cos(theta_w_b)];
        J3 = zeros(3,3);
        J3(1:2,3) = -R2d_w_b*[-tvec_b_m(2); tvec_b_m(1)];
        J3(1:2,1:2) = -eye(2);
        matJacobian(3*i-2:3*i, idxStOdo+3*lpOdo-2:idxStOdo+3*lpOdo) = inv(mat_std)*J3;
    end
    
    % calculate J of odometry
    row_st = 3*mk.num;
    mat_std = mats_std_odo{1};
    matJacobian(row_st+1:row_st+3, idxStOdo+1:idxStOdo+3) = inv(mat_std);
    for i = 2:odo.num
        ps_w_b1 = vec_ps2d_w_b(i-1,:);
        ps_w_b2 = vec_ps2d_w_b(i,:);
        [J1, J2] = FunJacobianRelPs2d(ps_w_b1, ps_w_b2);
        mat_std = mats_std_odo{i};
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-5:idxStOdo+3*i-3) = inv(mat_std)*J1;
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-2:idxStOdo+3*i) = inv(mat_std)*J2;
        
        % odometric part, TODO!
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
    
%     % J of mk distance
%     mkid_1 = map_pre(1,1);
%     mkid_2 = map_pre(2,1);    
%     mkid_ord_1 = find(mk.vecMkId(:,1) == mkid_1, 1);
%     tvec_w_m1_model = vec_tvec_w_m(mkid_ord_1,:).';
%     mkid_ord_2 = find(mk.vecMkId(:,1) == mkid_2, 1);
%     tvec_w_m2_model = vec_tvec_w_m(mkid_ord_2,:).';   
%     
%     mat_std = 1;
%     J_mk_1 = 2*(tvec_w_m1_model - tvec_w_m2_model).';
%     J_mk_2 = -2*(tvec_w_m1_model - tvec_w_m2_model).';
%     
%     matJacobian(end, 3*mkid_ord_1+idxStMk-2:3*mkid_ord_1+idxStMk) = inv(mat_std) * J_mk_1;
%     matJacobian(end, 3*mkid_ord_2+idxStMk-2:3*mkid_ord_2+idxStMk) = inv(mat_std) * J_mk_2;
    
    
end

