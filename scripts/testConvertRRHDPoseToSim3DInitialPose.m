function testConvertRRHDPoseToSim3DInitialPose()
% testConvertRRHDPoseToSim3DInitialPose Basic checks for RRHD to Sim3D conversion.

assertConverted([10, 20, 0], 0, [10, -20, 0], [0, 0, 3*pi/2]);
assertConverted([10, 20, 0], 90, [10, -20, 0], [0, 0, pi]);
assertConverted([10, -20, 0], 270, [10, 20, 0], [0, 0, 0]);

fprintf("RRHD to Sim3D initial pose conversion checks passed.\n");
end

function assertConverted(rrhdPoint, rrhdYawDeg, expectedPos, expectedRot)
[initialPos, initialRot] = convertRRHDPoseToSim3DInitialPose(rrhdPoint, rrhdYawDeg);

assert(all(abs(initialPos - expectedPos) < 1e-12), ...
    "InitialPos mismatch for RRHD point [%s], yaw %.3f.", ...
    num2str(rrhdPoint), rrhdYawDeg);
assert(all(abs(initialRot - expectedRot) < 1e-12), ...
    "InitialRot mismatch for RRHD point [%s], yaw %.3f.", ...
    num2str(rrhdPoint), rrhdYawDeg);
end
