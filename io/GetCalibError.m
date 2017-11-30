function [ error ] = GetCalibError( result, truth )
%GETCALIBERROR compute calibration error

%% extrinsic error
error.tvec_b_c = result.tvec_b_c - truth.tvec_b_c;
error.rvec_b_c = drvec(result.rvec_b_c, truth.rvec_b_c);
error.norm_tvec = norm(error.tvec_b_c);
error.norm_rvec = norm(error.rvec_b_c);

%% odometric error
beta_l_result = result.mat_odo(2,1);
beta_r_result = result.mat_odo(2,2);
base_len_result = 2*result.mat_odo(1,2)/result.mat_odo(2,2);

beta_l_truth = truth.mat_odo(2,1);
beta_r_truth = truth.mat_odo(2,2);
base_len_truth = 2*truth.mat_odo(1,2)/truth.mat_odo(2,2);

error.beta_l = beta_l_result - beta_l_truth;
error.beta_r = beta_r_result - beta_r_truth;
error.base_len = base_len_result - base_len_truth;

%% set outlier
error.isoutlier = false;
if base_len_result < 10
    error.isoutlier = true;
end

end

