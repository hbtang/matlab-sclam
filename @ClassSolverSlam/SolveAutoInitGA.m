function vec_out = SolveAutoInitGA(this, measure, calib)
%SOLVEGAODOROT 此处显示有关此函数的摘要
%   此处显示详细说明

%% Read data and group constraints
disp('Init: calibrate odo_mat rotational term');

odo = measure.odo;
mk = measure.mk;

if isfield(this.setting.solver, 'init_thresh_locallp')
    threshLpLocal = this.setting.solver.init_thresh_locallp;
else
    threshLpLocal = 10;
end

cnstrRows = {};
for i = 1:numel(mk.vecMkId)
    mkId_i = mk.vecMkId(i);
    
    vecMkFind = find(mk.id == mkId_i);
    vecMkLpFind = mk.lp(vecMkFind);
    
    for j = 2:numel(vecMkLpFind)
        
        lp1 = vecMkLpFind(j-1);
        lp2 = vecMkLpFind(j);
        
        if abs(lp2-lp1) > threshLpLocal
            continue;
        end
        
        rowOdo1 = find(odo.lp == lp1);
        rowOdo2 = find(odo.lp == lp2);
        
        rowMk1 = vecMkFind(j-1);
        rowMk2 = vecMkFind(j);
        
        cnstrRow.rowOdo1 = rowOdo1;
        cnstrRow.rowOdo2 = rowOdo2;
        cnstrRow.rowMk1 = rowMk1;
        cnstrRow.rowMk2 = rowMk2;
        
        cnstrRows{numel(cnstrRows)+1} = cnstrRow;
    end
end


%% calibrate rotational terms and alpha_23
num_cnstr = numel(cnstrRows);
A = zeros(num_cnstr, 2);
b = zeros(num_cnstr, 1);
% define a reference axiz z in camera frame
tvec_z_c = [0;0;1];

vec_alpha_2 = [];
vec_alpha_3 = [];
for i = 1:num_cnstr
    rowOdo1 = cnstrRows{i}.rowOdo1;
    rowOdo2 = cnstrRows{i}.rowOdo2;
    rowMk1 = cnstrRows{i}.rowMk1;
    rowMk2 = cnstrRows{i}.rowMk2;
    
    enc_l_1 = odo.enc_l(rowOdo1);
    enc_r_1 = odo.enc_r(rowOdo1);
    rvec_c1_m = mk.rvec(rowMk1,:).';
    R3d_c1_m = rodrigues(rvec_c1_m);
    
    enc_l_2 = odo.enc_l(rowOdo2);
    enc_r_2 = odo.enc_r(rowOdo2);
    rvec_c2_m = mk.rvec(rowMk2,:).';
    R3d_c2_m = rodrigues(rvec_c2_m);
    
    R3d_c1_c2 = R3d_c1_m * R3d_c2_m.';
    rvec_c1_c2 = rodrigues(R3d_c1_c2);
    
    % step 1: build LES for mat_odo_rot
    % as in the reference paper 2010-ISR A non-iterative and effective procedure for simultaneous odometry and camera calibration for a differential drive mobile robot based on the singular value decomposition.pdf
    theta_ij = norm(rvec_c1_c2) * sign(tvec_z_c.'*rvec_c1_c2);
    d_enc_l = enc_l_2 - enc_l_1;
    d_enc_r = enc_r_2 - enc_r_1;
    A(i,:) = [d_enc_l, d_enc_r];
    b(i) = theta_ij;
    
    % step 2: compute alpha 23
    % bug when theta_ij == 0
    if theta_ij ~= 0
        R = R3d_c1_c2;
        r = 1/(2*sin(theta_ij)) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
        alpha_2_ij = atan2(norm(r(1:2)), r(3));
        alpha_3_ij = atan2(r(2), -r(1));
        vec_alpha_2 = [vec_alpha_2; alpha_2_ij];
        vec_alpha_3 = [vec_alpha_3; alpha_3_ij];
    end
end

mat_odo_rot = pinv(A)*b;
alpha_2 = mean(vec_alpha_2);
alpha_3 = mean(vec_alpha_3);

%% calibrate the other terms
% [b, x, y, c, s]
beta_l = mat_odo_rot(1);
beta_r = mat_odo_rot(2);

Psi = zeros(2*num_cnstr, 5);

for i = 1:num_cnstr
    rowOdo1 = cnstrRows{i}.rowOdo1;
    rowOdo2 = cnstrRows{i}.rowOdo2;
    rowMk1 = cnstrRows{i}.rowMk1;
    rowMk2 = cnstrRows{i}.rowMk2;
    
    enc_l_1 = odo.enc_l(rowOdo1);
    enc_r_1 = odo.enc_r(rowOdo1);
    rvec_c1_m = mk.rvec(rowMk1,:).';
    tvec_c1_m = mk.tvec(rowMk1,:).';
    [T3d_c1_m, R3d_c1_m] = FunVec2Trans3d(rvec_c1_m, tvec_c1_m);
    
    enc_l_2 = odo.enc_l(rowOdo2);
    enc_r_2 = odo.enc_r(rowOdo2);
    rvec_c2_m = mk.rvec(rowMk2,:).';
    tvec_c1_m = mk.tvec(rowMk2,:).';
    [T3d_c2_m, R3d_c2_m] = FunVec2Trans3d(rvec_c2_m, tvec_c1_m);
    
    T3d_c1_c2 = T3d_c1_m * inv(T3d_c2_m);
    [rvec_c1_c2, tvec_c1_c2] = FunTrans2Vec3d(T3d_c1_c2);
    [rvec_c2_c1, tvec_c2_c1] = FunTrans2Vec3d(inv(T3d_c1_c2));
    
    d_enc_l = enc_l_2 - enc_l_1;
    d_enc_r = enc_r_2 - enc_r_1;
    
    % step 1: build LES for mat_odo_rot
    % as in the reference paper 2010-ISR A non-iterative and effective procedure for simultaneous odometry and camera calibration for a differential drive mobile robot based on the singular value decomposition.pdf
    theta_ij = norm(rvec_c1_c2) * sign(tvec_z_c.'*rvec_c1_c2);
    theta_ji = - theta_ij;
    R_theta_l = [cos(theta_ji), -sin(theta_ji); sin(theta_ji), cos(theta_ji)];
    T_theta_l = eye(2) - R_theta_l;
    a_l = cos(alpha_2)*cos(alpha_3)*tvec_c2_c1(1) ...
        - cos(alpha_2)*sin(alpha_3)*tvec_c2_c1(2) ...
        + sin(alpha_2)*tvec_c2_c1(3);
    b_l = sin(alpha_3)*tvec_c2_c1(1) + cos(alpha_3)*tvec_c2_c1(2);
    T_alpha_l = [a_l, -b_l; b_l a_l];
    
    beta = (-beta_l*d_enc_l/2 + beta_r*d_enc_r/2) * ...
        [cos(theta_ij/2); sin(theta_ij/2)];
    
    Psi_block = [R_theta_l*beta, T_theta_l, T_alpha_l];
    Psi(2*i-1:2*i,:) = Psi_block;
end

[~, ~, V] = svd(Psi);
rmse_min = inf;
v_min = V(:,1);
for i = 1:5
    v = V(:,i);
    v = v / norm(v(4:5));
    rmse = rms(Psi*v);
    if rmse < rmse_min
        rmse_min = rmse;
        v_min = v;
    end
end
if v_min(1) < 0
    v_min = -v_min;
end


b = v_min(1); x = v_min(2); y = v_min(3); c = v_min(4); s = v_min(5);
alpha_1 = atan2(s, c);
rvec_alpha_1 = [0;0;alpha_1];
rvec_alpha_2 = [0;alpha_2;0];
rvec_alpha_3 = [0;0;alpha_3];
R_b_c = rodrigues(rvec_alpha_1) * rodrigues(rvec_alpha_2) * rodrigues(rvec_alpha_3);
rvec_b_c = rodrigues(R_b_c);
tvec_b_c = [x;y;0];
mat_odo = [-b*beta_l/2, b*beta_r/2; beta_l, beta_r];

calib.SetVecbc(rvec_b_c, tvec_b_c);
calib.mat_odo = mat_odo;

end

