function measure_new = Enc2Odo(this, measure, calib)
%ENC2ODO 此处显示有关此函数的摘要
%   此处显示详细说明
measure_new = ClassMeasure();
measure.CopyTo(measure_new);
odo = measure_new.odo;
mat_odo = calib.mat_odo;
odo_new = odo;

for i = 2:odo.num
    enc_l_1 = odo_new.enc_l(i-1);
    enc_r_1 = odo_new.enc_r(i-1);
    x_1 = odo_new.x(i-1);
    y_1 = odo_new.y(i-1);
    theta_1 = odo_new.theta(i-1);
    ps2d_b1 = [x_1; y_1; theta_1];
    
    enc_l_2 = odo_new.enc_l(i);
    enc_r_2 = odo_new.enc_r(i);    
    d_enc_l = enc_l_2 - enc_l_1;
    d_enc_r = enc_r_2 - enc_r_1;
    
    vec = mat_odo * [d_enc_l; d_enc_r];
    d_l = vec(1);
    d_theta = vec(2);
    d_x = d_l * cos(d_theta/2);
    d_y = d_l * sin(d_theta/2);
    ps2d_b1_b2 = [d_x; d_y; d_theta];   
    
    ps2d_b2 = FunMove2d(ps2d_b1, ps2d_b1_b2);
    
    odo_new.x(i) = ps2d_b2(1);
    odo_new.y(i) = ps2d_b2(2);
    odo_new.theta(i) = ps2d_b2(3);
end

measure_new.odo = odo_new;

end

