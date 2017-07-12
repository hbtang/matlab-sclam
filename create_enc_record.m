clear;

%% read raw data
path_fold = 'C:\Workspace\Data\sim\exp-agv-enc-ceiling-mk211\';
file_mk = 'mk.rec';
file_odo = 'odo_raw.rec';
file_odo_new = 'odo.rec';
measure = ClassMeasure(path_fold, file_mk, file_odo);
measure.ReadRecData;

% thresh_lin_odo = 200;
% thresh_rot_odo = 3*pi/180;
% measure.PruneData(thresh_lin_odo, thresh_rot_odo, false);

%% set configure
wheel_diameter = 100;
thread_per_round = 16*64;
base_len = 750;
alpha_l = wheel_diameter*pi/thread_per_round;
alpha_r = alpha_l;

mat_odo = [alpha_l/2, alpha_r/2; -alpha_l/base_len, alpha_r/base_len];
mat_odo_inv = inv(mat_odo);

%% compute odo configure
% T2d_odo1_enc1: odo is the raw odometry frame, enc is the differential
% odometry frame according to the encoders
% mat_odo: 2x2 matrix from d_enc to d_l and d_theta

odo = measure.odo;

A = zeros(odo.num-1, 3);

for i = 2:odo.num
    x_odo_1 = odo.x(i-1);
    y_odo_1 = odo.y(i-1);
    theta_odo_1 = odo.theta(i-1);
    
    x_odo_2 = odo.x(i);
    y_odo_2 = odo.y(i);
    theta_odo_2 = odo.theta(i);
    
    ps2d_odo_1 = [x_odo_1; y_odo_1; theta_odo_1];
    ps2d_odo_2 = [x_odo_2; y_odo_2; theta_odo_2];
    
    d_ps2d_odo = FunRelPos2d(ps2d_odo_1, ps2d_odo_2);
    dx = d_ps2d_odo(1);
    dy = d_ps2d_odo(2);
    d_theta = d_ps2d_odo(3);
    
    A(i,:) = [dx, dy, -d_theta];
end

[~, ~, V] = svd(A);
rmse_min = inf;
v_min = V(:,1);
for i = 1:3
    v = V(:,i);
    if v(3) < 0
        v = -v;
    end
    v = v / norm(v(1:2));
    rmse = rms(A*v);
    if rmse < rmse_min
        rmse_min = rmse;
        v_min = v;
    end
end
sin_alpha = v_min(1);
cos_alpha = v_min(2);
d = v_min(3);
alpha = atan2(sin_alpha, cos_alpha);

ps2d_enc_odo = [d; 0; alpha];
T2d_enc_odo = FunVec2Trans2d(ps2d_enc_odo);
T2d_odo_enc = inv(T2d_enc_odo);


%% create new odo with encoder measurements
odo_new = odo;
odo_new.enc_l = zeros(odo.num, 1);
odo_new.enc_r = zeros(odo.num, 1);

ps2d_w_enc = [odo_new.x(1); odo_new.y(1); odo_new.theta(1)];

for i = 2:odo.num
    x_odo_1 = odo.x(i-1);
    y_odo_1 = odo.y(i-1);
    theta_odo_1 = odo.theta(i-1);
    x_odo_2 = odo.x(i);
    y_odo_2 = odo.y(i);
    theta_odo_2 = odo.theta(i);
    ps2d_odo_1 = [x_odo_1; y_odo_1; theta_odo_1];
    ps2d_odo_2 = [x_odo_2; y_odo_2; theta_odo_2];
    
    d_ps2d_odo = FunRelPos2d(ps2d_odo_1, ps2d_odo_2);
    T2d_odo1_odo2 = FunVec2Trans2d(d_ps2d_odo);
    T2d_enc1_enc2 = T2d_enc_odo*T2d_odo1_odo2*T2d_odo_enc;
    d_ps2d_enc = FunTrans2Vec2d(T2d_enc1_enc2);
    ps2d_w_enc = FunMove2d(ps2d_w_enc, d_ps2d_enc);
    
    odo_new.x(i) = ps2d_w_enc(1);
    odo_new.y(i) = ps2d_w_enc(2);
    odo_new.theta(i) = ps2d_w_enc(3);
    
    d_x = d_ps2d_enc(1);
    d_y = d_ps2d_enc(2);
    
    d_l = norm([d_x; d_y]) * sign(d_x);
    d_theta = d_ps2d_enc(3);
    
    d_enc = mat_odo_inv * [d_l; d_theta];
    d_enc_l = d_enc(1);
    d_enc_r = d_enc(2);
    
    odo_new.enc_l(i) = odo_new.enc_l(i-1) + d_enc_l;
    odo_new.enc_r(i) = odo_new.enc_r(i-1) + d_enc_r;  
end

odo_new.enc_l = floor(odo_new.enc_l);
odo_new.enc_r = floor(odo_new.enc_r);

%% record file
% open file
file_out_odo = fopen([path_fold, 'record/', file_odo_new],'w+');
fprintf(file_out_odo, '# odometry info\n');
fprintf(file_out_odo, '# format: lp time_odo time_cam x y theta enc_l enc_r\n');

% record data
odo_rec = odo_new;
timestep = 30;
for i = 1:numel(odo_rec.lp)
    strOdo = [num2str(odo_rec.lp(i)), ' '];
    strOdo = [strOdo, ...
        num2str(odo_rec.lp(i)*timestep), ' ', ...
        num2str(odo_rec.lp(i)*timestep), ' ', ...
        ];
    strOdo = [strOdo, ...
        num2str(odo_rec.x(i)), ' ', ...
        num2str(odo_rec.y(i)), ' ', ...
        num2str(odo_rec.theta(i)), ' ', ...
        ];
    strOdo = [strOdo, ...
        num2str(odo_rec.enc_l(i)), ' ', ...
        num2str(odo_rec.enc_r(i)), ' ', ...
        ];
    strOdo = [strOdo, '\n'];
    fprintf(file_out_odo, strOdo);
end

% close file
fclose(file_out_odo);









