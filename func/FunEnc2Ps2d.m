function [ ps2d ] = FunEnc2Ps2d( d_enc_l, d_enc_r, mat_odo )
%FUNENC2PS2D 此处显示有关此函数的摘要
%   此处显示详细说明

vec = mat_odo * [d_enc_l; d_enc_r];
d_l = vec(1);
d_theta = vec(2);
d_x = d_l * cos(d_theta/2);
d_y = d_l * sin(d_theta/2);
ps2d = [d_x; d_y; d_theta];

end

