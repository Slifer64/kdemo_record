function convertToMat(filename)

if (nargin<1), filename = 'data.bin'; end

addpath('utils/');

TOOL_POS = 0;
TOOL_ORIENT = 1;
TOOL_FORCE = 2;
TOOL_TORQUE = 3;
JOINT_POS = 4;
JOINT_TORQUE = 5;
JACOBIAN = 6;
Data = struct('TOOL_POS',[], 'TOOL_ORIENT',[], 'TOOL_FORCE',[], 'TOOL_TORQUE',[], 'JOINT_POS',[], 'JOINT_TORQUE',[], 'JACOBIAN',[]);

fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end

while ~feof(fid)
    
    dat_id = read_scalar(fid, true, 'int32');
    
    if (dat_id == TOOL_POS), Data.TOOL_POS = read_mat(fid, true);
    elseif (dat_id == TOOL_ORIENT), Data.TOOL_ORIENT = read_mat(fid, true);
    elseif (dat_id == TOOL_FORCE), Data.TOOL_FORCE = read_mat(fid, true);
    elseif (dat_id == TOOL_TORQUE), Data.TOOL_TORQUE = read_mat(fid, true);
    elseif (dat_id == JOINT_POS), Data.JOINT_POS = read_mat(fid, true);
    elseif (dat_id == JOINT_TORQUE), Data.JOINT_TORQUE = read_mat(fid, true);
    elseif (dat_id == JACOBIAN), Data.JACOBIAN = read_mat(fid, true);
    end
    
end

fclose(fid);

mat_filename = strrep(filename,'.bin','.mat');
save(mat_filename,'Data');

end
