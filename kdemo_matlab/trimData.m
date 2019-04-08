function Data = trimData(Data, vel_thres)

if (isempty(Data.TOOL_POS))
    warning('Cannot apply data trimming: Position data are empty...');
    return;
end

Pos = Data.TOOL_POS;
Vel = zeros(size(Pos));
for i=1:size(Vel,1), Vel(i,:)=[0 diff(Pos(i,:))]; end

vel_norm = zeros(size(Vel,2));
for j=1:length(vel_norm), vel_norm(j)=norm(Vel(:,j)); end

i1 = 1;
for j=1:length(vel_norm)
   if vel_norm(j)>=vel_thres, break; end
   i1 = i1 + 1;
end

i2 = length(vel_norm)+1;
for j=length(vel_norm):-1:1
   if vel_norm(j)>=vel_thres, break; end
   i2 = i2 - 1;
end

if (~isempty(Data.TOOL_POS)), Data.TOOL_POS = Data.TOOL_POS(:, i1:i2); end
if (~isempty(Data.TOOL_ORIENT)), Data.TOOL_ORIENT = Data.TOOL_ORIENT(:, i1:i2); end
if (~isempty(Data.TOOL_FORCE)), Data.TOOL_FORCE = Data.TOOL_FORCE(:, i1:i2); end
if (~isempty(Data.TOOL_TORQUE)), Data.TOOL_TORQUE = Data.TOOL_TORQUE(:, i1:i2); end
if (~isempty(Data.JOINT_POS)), Data.JOINT_POS = Data.JOINT_POS(:, i1:i2); end
if (~isempty(Data.JOINT_TORQUE)), Data.JOINT_TORQUE = Data.JOINT_TORQUE(:, i1:i2); end
if (~isempty(Data.JACOBIAN)), Data.JACOBIAN = Data.JACOBIAN(:, i1:i2); end

size(Data.TOOL_POS)

end

