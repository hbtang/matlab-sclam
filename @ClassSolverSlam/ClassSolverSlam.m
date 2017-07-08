classdef ClassSolverSlam
    %CLASSSOLVER class of solver for calibration and mapping
    
    properties
        % figure handle for display estimation resolt
        hdFigSolver;
        % error configuration
        setting;
    end
    
    methods
        
        % constructor function
        function this = ClassSolverSlam(settingInput)
            this.setting = settingInput;
        end
        
        %% calibration autoinit from encoders
        function DoAutoInitEnc(this, measure, calib)
            SolveGrndPlaneLin(this, measure, calib);
            ProjMk(this, measure, calib);
            
            
            SolveMatOdoExtSe2(this, measure, calib);
            
            
        end
        
        % outputs of ProjMk():
        % 1. estimate ground in each mark: pvec_g_m dist_g_m
        % 2. refined mk measure: mk.rvec_refine mk.tvec_refine
        % 3. project mk measure: mk.se2_cg_mg
        ProjMk(this, measure, calib);
        
        % SolveMatOdoExtSe2: solve mat_odo and the remaining 3 dof
        % extrinsic parameter se2_b_cg
        SolveMatOdoExtSe2(this, measure, calib);
        
        
        % create odo measure from enc and mat_odo
        % saved in measure.odo.enc_x/enc_y/enc_theta
        measure_new = Enc2Odo(this, measure, calib);       
        
        
        
        % solver joint opt
        SolveJointOptMSlamEnc(this, measure, calib, map, options);
        SolveJointOptVSlamEnc(this, measure, calib, map, options);
        RefineScale(this, calib, map);
        
        
        
        %% calibration init. linear solution
        function DoAutoInitOdo(this, measure, calib)
            % step 1.1: init. estimate ground
            this.SolveGrndPlaneLin(measure, calib);
            % step 1.2: init. estimate yaw and XY
            this.SolveYawXY(measure, calib);
            % print calibration results
            calib.DispCalib;
        end
        % solve ground plane by linear constraints
        SolveGrndPlaneLin(this, measure, calib);
        % solve YawXY by linear constraints
        SolveYawXY(this, measure, calib);
        
        %% calibration init. Guo's linear solution
        SolveInitGuo(this, measure, calib);
        % solve YawXY by Guo's
        SolveYawXYGuo(this, measure, calib);
        
        %% calibration init. non-linear solution
        %         % solve ground plane by non-linear constraints
        %         SolveGrndPlane(this, measure, calib);
        %         cost = CostGrndPlane(this, q, mk);
        %         % solve YawXY by non-linear constraints
        %         SolveLocalOpt(this, measure, calib);
        %         [ vecCost, matJacobian ] = CostLocalOpt(this, q, measure, calib);
        %         % solve YawXY by non-linear constraints, consider loop close
        %         SolveLocalLoopOpt(this, measure, calib);
        %         [ vecCost, matJacobian ] = CostLocalLoopOpt(this, q, measure, calib);
        
        %% calibration with joint optimization
        % consider mark observation, do mark slam based calibration
        SolveJointOptMSlam(this, measure, calib, map, options);
        % consider image feature only, do visual slam based calibration,
        SolveJointOptVSlam(this, measure, calib, map, options);
        
        %         [vecCost, matJacobian] = CostJointOptVSlam(this, q, mk, odo, time, calib, setting, options);
        
        %         % consider 3 dof extrinsic in ground plane ps2d_b_cg
        %         SolveJointOpt(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt(this, q, mk, odo, calib);
        %
        %         % consider 5 dof extrinsic except z_b_c = 0
        %         SolveJointOpt2(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt2(this, q, mk, odo, calib);
        %
        %         % consider 5 dof extrinsic and 1 dof time delay
        %         SolveJointOpt3(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt3(this, q, mk, odo, time, calib);
        %
        %         % consider 5 dof extrinsic, 1 dof time delay, 2 dof odometric
        %         SolveJointOpt4(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt4(this, q, mk, odo, time, calib);
        
        
        %% solve SLAM only, fixed on given calib
        function err_slam = DoSlam(this, measure, calib, map, options_drawmap)
            % step 1.3: v-slam after init.
            % init. map from odometry and mark observation
            map.InitMap(measure, calib);
            % do vslam with current calibration results
            options_slam = struct(...
                'bCalibExtRot', false, 'bCalibExtLin', false,...
                'bCalibTmp', false, 'bCalibOdo', false);
            this.SolveJointOptVSlam(measure, calib, map, options_slam);
            % display init-calib results
            calib.DispCalib;
            % draw v-slam results
            map.DrawMap(measure, calib, this.setting, options_drawmap);
            % compute vslam error
            options_errvslam = struct('bCalibTmp', true, 'bCalibOdo', true);
            err_slam = Err_vSlam(measure, calib, map, this.setting, options_errvslam );
        end
        %         % solve mark-SLAM problem, consider mark observation tvec_c_m
        %         SolveMSlam(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostMSlam(this, q, mk, odo, time, calib);
        
        %% io
        % draw results gound plane
        %         DrawResGrndPlane(this, measure, calib, vec_ground);
        
    end
    
end

