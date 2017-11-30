function [ vec_out ] = RemoveOutliers( vec_in, ratio_thresh, val_min, val_max )
%REMOVEOUTLIERS 此处显示有关此函数的摘要
%   此处显示详细说明

size_in = size(vec_in);
row_in = size_in(1);
col_in = size_in(2);

if nargin < 2
    ratio_thresh = 1;
end

if nargin < 3
    std_in = std(vec_in);
    mean_in = mean(vec_in);
    val_min = mean_in - ratio_thresh*std_in;
    val_max = mean_in + ratio_thresh*std_in;
else
    val_min = ones(1,col_in) * val_min;
    val_max = ones(1,col_in) * val_max;
end



vec_out = [];
for i = 1:row_in
    bBadRow = false;
    for j = 1:col_in
        if val_min(j) > vec_in(i,j) || vec_in(i,j) > val_max(j)
            bBadRow = true;
            break;
        end
    end
    if ~bBadRow
        vec_out = [vec_out; vec_in(i,:)];
    end
end

end

