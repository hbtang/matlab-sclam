function PruneData( this, threshDistOdo, threshAngleOdo, bUseEnc, bKeepAllMk )
%PRUNEDATA prune raw input data

%% init

if nargin == 1
    threshDistOdo = 500;
    threshAngleOdo = 5*pi/180;
end

if nargin < 4
    bUseEnc = true;
end
if nargin < 5
    bKeepAllMk = false;
end


%% prune odometry data
x_ref = this.odo.x(1);
y_ref = this.odo.y(1);
theta_ref = this.odo.theta(1);

if bUseEnc
    odo_new = struct( ...
        'lp',this.odo.lp(1), ...
        'x', this.odo.x(1), ...
        'y', this.odo.y(1), ...
        'theta', this.odo.theta(1), ...
        'enc_l', this.odo.enc_l(1), ...
        'enc_r', this.odo.enc_r(1));
else
    odo_new = struct( ...
        'lp',this.odo.lp(1), ...
        'x', this.odo.x(1), ...
        'y', this.odo.y(1), ...
        'theta', this.odo.theta(1));
end

for i = 2:this.odo.num
    x_now = this.odo.x(i);
    y_now = this.odo.y(i);
    theta_now = this.odo.theta(i);
    lp_now = this.odo.lp(i);
    
    if bUseEnc
        enc_l_now = this.odo.enc_l(i);
        enc_r_now = this.odo.enc_r(i);
    end
    
    dx = x_now - x_ref;
    dy = y_now - y_ref;
    dl = norm([dx;dy]);
    dtheta = FunPrdCnst(theta_now - theta_ref, pi, -pi);
    
    
    % if has mk this loop
    if bKeepAllMk && ~isempty(find(this.mk.lp == lp_now, 1))
        bHasMkNow = true;
    else
        bHasMkNow = false;
    end
    
    
    bKeepThisOdo = false;
    if ~bHasMkNow
        if (dl > threshDistOdo) || (abs(dtheta) > threshAngleOdo)
            bKeepThisOdo = true;
        end
    else
        if (dl > 0.3*threshDistOdo) || (abs(dtheta) > 0.3*threshAngleOdo)
            bKeepThisOdo = true;
        end
    end
    
    
    if bKeepThisOdo
        odo_new.lp = [odo_new.lp; lp_now];
        odo_new.x = [odo_new.x; x_now];
        odo_new.y = [odo_new.y; y_now];
        odo_new.theta = [odo_new.theta; theta_now];
        
        if bUseEnc
            odo_new.enc_l = [odo_new.enc_l; enc_l_now];
            odo_new.enc_r = [odo_new.enc_r; enc_r_now];
        end
        
        x_ref = x_now;
        y_ref = y_now;
        theta_ref = theta_now;
    end
end
odo_new.num = numel(odo_new.lp);
this.odo = odo_new;
disp(['Odometry info prunning is done, ', num2str(odo_new.num), ' records remained.']);

%% prune mark data
mk_new = struct('lp', [], 'id', [], ...
    'rvec', [], 'tvec', [], 'num', [], ...
    'numMkId', [], 'vecMkId', [], ...
    'pt1', [], 'pt2', [], 'pt3', [], 'pt4', []);
bRecPtExist = ~isempty(this.mk.pt1);
for i = 1:this.mk.num
    tmp = find(odo_new.lp == this.mk.lp(i), 1);
    if numel(tmp) ~= 0
        mk_new.lp = [mk_new.lp; this.mk.lp(i)];
        mk_new.id = [mk_new.id; this.mk.id(i)];
        mk_new.rvec = [mk_new.rvec; this.mk.rvec(i,:)];
        mk_new.tvec = [mk_new.tvec; this.mk.tvec(i,:)];
        
        if bRecPtExist
            mk_new.pt1 = [mk_new.pt1; this.mk.pt1(i,:)];
            mk_new.pt2 = [mk_new.pt2; this.mk.pt2(i,:)];
            mk_new.pt3 = [mk_new.pt3; this.mk.pt3(i,:)];
            mk_new.pt4 = [mk_new.pt4; this.mk.pt4(i,:)];
        end
    end
end
mk_new.num = numel(mk_new.lp);
mk_new.vecMkId = unique(mk_new.id);
mk_new.numMkId = numel(mk_new.vecMkId);
this.mk = mk_new;
disp(['Marker observation info prunning is done, ', num2str(mk_new.num), ' records remained.'])
disp(' ');

%% prune time data
time_new = struct('lp',[],'t_odo',[],'t_mk',[]);
for i = 1:numel(this.time.lp)
    tmp = find(odo_new.lp == this.time.lp(i), 1);
    if ~isempty(tmp)
        time_new.lp = [time_new.lp; this.time.lp(i)];
        time_new.t_odo = [time_new.t_odo; this.time.t_odo(i)];
        time_new.t_mk = [time_new.t_mk; this.time.t_mk(i)];
    end
end
this.time = time_new;
disp(['Time info prunning is done, ', num2str(numel(this.time.lp)), ' records remained.'])
disp(' ');

end

