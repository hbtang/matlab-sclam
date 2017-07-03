function AddNoiseRecord(this, options)
%ADDNOISERECORD 此处显示有关此函数的摘要
%   此处显示详细说明
if ~isfield(options, 'flag')
    error('Error in options!')
end

switch options.flag
    case 'odo'
        % add noise and record odo.rec
        if ~isfield(options, 'stdratio_lin')
            options.stdratio_lin = 0;
        end
        if ~isfield(options, 'stdratio_rot')
            options.stdratio_rot = 0;
        end
        if ~isfield(options, 'set')
            options.set = 0;
        end
        stdratio_lin = options.stdratio_lin;
        stdratio_rot = options.stdratio_rot;
        set = options.set;        
        
        this.setting.error.odo.stdratio_lin = stdratio_lin;
        this.setting.error.odo.stdratio_rot = stdratio_rot;      
        this.odo_rec = NoiseOdo(this, this.odo_true, this.calib, this.setting);
        
        str_suffix_odo = [ ...
            '-el', num2str(stdratio_lin), ...
            '-er', num2str(stdratio_rot), ...
            '-s', num2str(set), ...
            ];        
        options_rec = struct( ...
            'str_suffix_odo', str_suffix_odo, ...
            'b_record_odo', true );
        this.Record(options_rec);
        
        disp([...
            'set: ', num2str(set), ...
            ', stdratio_lin: ' , num2str(stdratio_lin), ...
            ', stdratio_rot: ' , num2str(stdratio_rot)]);        
    
    case 'mk'
        % add noise and record mk.rec
        if ~isfield(options, 'std_img')
            options.std_img = 0;
        end
        if ~isfield(options, 'set')
            options.set = 0;
        end
        std_img = options.std_img;
        set = options.set;
        
        this.setting.error.mk.std_imgu = std_img;
        this.setting.error.mk.std_imgv = std_img;       
        this.mk_rec = this.NoiseMk(this.mk_true, this.calib, this.setting);
        
        str_suffix_mk = [ ...
            '-euv', num2str(std_img), ...
            '-s', num2str(set), ...
            ];
        
        options_rec = struct( ...
            'str_suffix_mk', str_suffix_mk, ...
            'b_record_mk', true );
        this.Record(options_rec);
        
        disp(['set: ', num2str(set), ', std_img: ' , num2str(std_img)]);        
        
    otherwise
        
end





end

