function testComputeLaneDeviationFromRRHD()
% testComputeLaneDeviationFromRRHD Unit checks for RRHD lane deviation logic.

fprintf("Running lane deviation tests...\n");

testForwardLaneSign();
testBackwardLaneSign();
testSim3DCoordinateConversion();
testNearestLaneSelection();
testVectorizedInput();

fprintf("Lane deviation tests passed.\n");
end

function testForwardLaneSign()
lane = makeLane("L_forward", [0 0 0; 100 0 0], "Forward");

[dRight, detailsRight] = computeLaneDeviationFromRRHD([10 -2 0], lane, CoordinateFrame="RRHD");
[dLeft, detailsLeft] = computeLaneDeviationFromRRHD([10 2 0], lane, CoordinateFrame="RRHD");

assertClose(dRight, 2);
assertClose(dLeft, -2);
assert(detailsRight.LaneID == "L_forward");
assertClose(detailsRight.Projection, [10 0 0]);
assertClose(detailsLeft.TravelDirection_xy, [1 0]);
end

function testBackwardLaneSign()
lane = makeLane("L_backward", [0 0 0; 100 0 0], "Backward");

[dRight, detailsRight] = computeLaneDeviationFromRRHD([10 2 0], lane, CoordinateFrame="RRHD");
[dLeft, detailsLeft] = computeLaneDeviationFromRRHD([10 -2 0], lane, CoordinateFrame="RRHD");

assertClose(dRight, 2);
assertClose(dLeft, -2);
assertClose(detailsRight.TravelDirection_xy, [-1 0]);
end

function testSim3DCoordinateConversion()
lane = makeLane("L_map", [0 0 0; 100 0 0], "Forward");

% Sim3D y=+3 converts to RRHD/map y=-3, which is right of eastbound lane.
[d, details] = computeLaneDeviationFromRRHD([10 3 0], lane);

assertClose(d, 3);
assertClose(details.VehiclePositionMap, [10 -3 0]);
end

function testNearestLaneSelection()
lanes = [
    makeLane("L_far", [0 10 0; 100 10 0], "Forward")
    makeLane("L_near", [0 0 0; 100 0 0], "Forward")
    ];

[d, details] = computeLaneDeviationFromRRHD([50 -1 0], lanes, CoordinateFrame="RRHD");

assertClose(d, 1);
assert(details.LaneID == "L_near");
assert(details.LaneIndex == 2);
end

function testVectorizedInput()
lane = makeLane("L_vector", [0 0 0; 100 0 0], "Forward");
positions = [
    10 -1 0
    20 0 0
    30 4 0
    ];

[d, details] = computeLaneDeviationFromRRHD(positions, lane, CoordinateFrame="RRHD");

assertClose(d, [1; 0; -4]);
assert(all(details.LaneID == "L_vector"));
assertClose(details.Station_m, [10; 20; 30]);
end

function lane = makeLane(id, geometry, travelDirection)
lane = struct;
lane.ID = id;
lane.LaneType = "Driving";
lane.TravelDirection = travelDirection;
lane.Geometry = geometry;
end

function assertClose(actual, expected)
tol = 1e-9;
assert(isequal(size(actual), size(expected)), ...
    "Size mismatch: expected %s, got %s.", mat2str(size(expected)), mat2str(size(actual)));
assert(all(abs(actual - expected) <= tol, "all"), ...
    "Expected %s, got %s.", mat2str(expected), mat2str(actual));
end
