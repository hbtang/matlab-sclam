classdef ClassSimulator < handle
    %CLASSSIMULATOR class of simulator
    % to generate record file according to a predefined map
    
    %%
    properties
        %% configuration
        % io configures
        files = struct( ...
                'str_simfolder', [], ...
                'file_out_odo', [], ...
                'file_out_mk', [], ...
                'str_path_setting', []);
        % handles of figure and figure objects
        hds = struct( ...
            'hdFigSim', [], ...
            'hdObjBaseX', [], 'hdObjBaseY', [], 'hdObjBaseZ', [], ...
            'hdObjMap', [], 'hdObjRange', []);
        % status
        flag = struct( ...
            'bQuit', false);
        % setting configuration read from yaml file
        setting;
        
        %% simulation environment
        % map: ClassMap
        map;
        % calib: ClassCalib
        calib;
        % frequency configure
        timestep;
        
        %% robot status
        % current loop id
        lp = 0;
        % current robot pose in w frame, single variable
        ps2d_w_b = [0; 0; 0];
        % current robot velocity
        vel_lin = 0; 
        vel_rot = 0;
        % current encoder
        enc_l = 0; 
        enc_r = 0;
        
        %% raw data record
        % odometry measurements, with error ratio (proportional to measure)
        odo         = struct('lp',[],'x',[],'y',[],'theta',[],'enc_l',[],'enc_r',[]);
        odo_true    = struct('lp',[],'x',[],'y',[],'theta',[],'enc_l',[],'enc_r',[]);
        odo_rec     = struct('lp',[],'x',[],'y',[],'theta',[],'enc_l',[],'enc_r',[]);
        % current mark measure, vector possible
        mk          = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);        
        mk_true     = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);        
        mk_rec      = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);
        
    end
    
    %%
    methods
        
        function this = ClassSimulator(name_sim_fold)
            if nargin < 1
                name_sim_fold = './sim/';
            end
            this.map = ClassMap;
            this.calib = ClassCalib;
            this.timestep = 0.1;            
            % load configures            
             Init(this, name_sim_fold);
        end
        
        % init function, load map and calib info, set output path, ...
        Init(this, name_sim_fold);
        % main function of simulator
        Run(this);
        % stop function
        Stop(this);
        % draw or refresh current status
        Draw(this);
        % renew mark observation
        Observe(this);
        
        % create odo_out from odo_true with noise
        odo_out = NoiseOdo(this, odo_true, calib, setting);
        mk_out = NoiseMk(this, mk_true, calib, setting);
        
        % write odo and mk into record file
        Record(this, options)
        AddNoiseRecord(this, options)
        
        % call back function when key pressed
        OnKeyPressed(this, hdFig, callBackData);
        
        % add noise into rec file
        AddNoise2Rec(this, noiseConfig, foldPath, setId);
        
        % compute encoder measurement from odometric parameters
        enc_out = CmpEncMsr(this, se2_b1_b2, mat_odo);
    end
end



