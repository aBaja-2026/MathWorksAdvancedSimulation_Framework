function [initialPos, initialRot, details] = convertRRHDPoseToSim3DInitialPose(rrhdPoint, rrhdYawDeg)
% convertRRHDPoseToSim3DInitialPose Convert an RRHD/RoadRunner pose to Sim3D initial pose.
%
% RRHD follows the RoadRunner/OpenDRIVE convention used for route selection.
% The Advanced Simulation 3D scene mirrors the RoadRunner Y axis and uses a
% different yaw zero-reference for the vehicle block. InitialRot is returned
% in radians, matching the Simulation 3D Physics Vehicle block.

validateattributes(rrhdPoint, {'numeric'}, {'real', 'vector', 'numel', 3}, ...
    mfilename, 'rrhdPoint', 1);
validateattributes(rrhdYawDeg, {'numeric'}, {'real', 'scalar', 'finite'}, ...
    mfilename, 'rrhdYawDeg', 2);

rrhdPoint = reshape(double(rrhdPoint), 1, 3);
rrhdYawDeg = double(rrhdYawDeg);

initialPos = [rrhdPoint(1), -rrhdPoint(2), rrhdPoint(3)];
rrhdYawRad = deg2rad(rrhdYawDeg);
initialYawRad = wrapTo2Pi(3*pi/2 - rrhdYawRad);
initialRot = [0, 0, initialYawRad];

details = struct;
details.SourceCoordinateSystem = "RRHD/RoadRunner";
details.TargetCoordinateSystem = "Simulation3D";
details.RRHDPoint = rrhdPoint;
details.RRHDYawDeg = rrhdYawDeg;
details.RRHDYawRad = rrhdYawRad;
details.InitialPos = initialPos;
details.InitialRot = initialRot;
details.PositionRule = "[x, y, z] -> [x, -y, z]";
details.YawRule = "yawDeg -> wrapTo2Pi(3*pi/2 - deg2rad(yawDeg))";
details.InitialRotUnits = "radians";
end
