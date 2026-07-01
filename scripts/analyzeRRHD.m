function summary = analyzeRRHD(rrhdFile)
% analyzeRRHD Analyze and plot a RoadRunner HD Map file.

if nargin < 1 || strlength(string(rrhdFile)) == 0
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    rrhdFile = fullfile(repoRoot, "localFolder", "IndianCityBlock.rrhd");
else
    rrhdFile = char(rrhdFile);
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
end

if ~isfile(rrhdFile)
    error("analyzeRRHD:FileNotFound", "RRHD file not found: %s", rrhdFile);
end

outDir = fullfile(repoRoot, "analysis_outputs");
if ~isfolder(outDir)
    mkdir(outDir);
end

rrMap = roadrunnerHDMap;
read(rrMap, rrhdFile);

summary = collectSummary(rrMap, rrhdFile, outDir);
printSummary(summary);

fig = figure("Name", "RRHD Overview", "Color", "w");
layout = tiledlayout(fig, 1, 1, "Padding", "compact", "TileSpacing", "compact");
ax = nexttile(layout);
plotMapGeometry(ax, rrMap, summary);
overviewPng = fullfile(outDir, "IndianCityBlock_rrhd_overview.png");
exportgraphics(fig, overviewPng, "Resolution", 200);
close(fig);

fig = figure("Name", "RRHD Detail", "Color", "w");
layout = tiledlayout(fig, 2, 2, "Padding", "compact", "TileSpacing", "compact");
plotMapGeometry(nexttile(layout), rrMap, summary, "Lanes");
plotMapGeometry(nexttile(layout), rrMap, summary, "LaneBoundaries");
plotMapGeometry(nexttile(layout), rrMap, summary, "Junctions");
plotMapGeometry(nexttile(layout), rrMap, summary, "Objects");
detailPng = fullfile(outDir, "IndianCityBlock_rrhd_detail.png");
exportgraphics(fig, detailPng, "Resolution", 200);
close(fig);

summary.OverviewPlot = string(overviewPng);
summary.DetailPlot = string(detailPng);
summaryFile = fullfile(outDir, "IndianCityBlock_rrhd_summary.mat");
save(summaryFile, "summary");
fprintf("Overview plot: %s\n", overviewPng);
fprintf("Detail plot:   %s\n", detailPng);
fprintf("Summary MAT:   %s\n", summaryFile);

end

function summary = collectSummary(rrMap, rrhdFile, outDir)
summary = struct;
summary.File = string(rrhdFile);
summary.OutputDirectory = string(outDir);
summary.GeoReference = getPropOrEmpty(rrMap, "GeoReference");

entityNames = [
    "Lanes"
    "LaneBoundaries"
    "LaneMarkings"
    "LaneGroups"
    "Junctions"
    "Barriers"
    "Signs"
    "Signals"
    "StaticObjects"
    "SpeedLimits"
    "ParkingSpaces"
    "ParkingSpaceMarkings"
    "LogicalLaneBoundaries"
    "Intersections"
    ];

counts = struct;
for i = 1:numel(entityNames)
    name = entityNames(i);
    value = getPropOrEmpty(rrMap, name);
    counts.(name) = numel(value);
end
summary.Counts = counts;

[allPts, sourceCounts] = collectAllGeometry(rrMap);
summary.GeometrySourceCounts = sourceCounts;
if isempty(allPts)
    summary.Bounds = [];
    summary.WidthMeters = NaN;
    summary.HeightMeters = NaN;
else
    minXYZ = min(allPts(:, 1:3), [], 1);
    maxXYZ = max(allPts(:, 1:3), [], 1);
    axisNames = ["X"; "Y"; "Z"];
    minVals = reshape(minXYZ(1:3), [], 1);
    maxVals = reshape(maxXYZ(1:3), [], 1);
    spanVals = maxVals - minVals;
    summary.Bounds = table(axisNames, minVals, maxVals, spanVals, ...
        'VariableNames', {'Axis', 'Min', 'Max', 'Span'});
    summary.WidthMeters = maxXYZ(1) - minXYZ(1);
    summary.HeightMeters = maxXYZ(2) - minXYZ(2);
end

lanes = getPropOrEmpty(rrMap, "Lanes");
summary.LaneTypes = countStringProperty(lanes, "LaneType");
summary.TravelDirections = countStringProperty(lanes, "TravelDirection");
summary.LaneGeometryPointCount = countGeometryPoints(lanes);
summary.LaneBoundaryGeometryPointCount = countGeometryPoints(getPropOrEmpty(rrMap, "LaneBoundaries"));
end

function value = getPropOrEmpty(obj, propName)
if isprop(obj, propName)
    value = obj.(propName);
else
    value = [];
end
end

function counts = countStringProperty(items, propName)
counts = table(strings(0, 1), zeros(0, 1), ...
    'VariableNames', {'Value', 'Count'});
if isempty(items)
    return
end
propNames = properties(items(1));
if ~any(strcmp(propName, propNames))
    return
end

values = strings(numel(items), 1);
for i = 1:numel(items)
    values(i) = string(items(i).(propName));
end
values(values == "") = "<empty>";
uniqueValues = unique(values);
counts = table(reshape(uniqueValues, [], 1), zeros(numel(uniqueValues), 1), ...
    'VariableNames', {'Value', 'Count'});
for i = 1:numel(uniqueValues)
    counts.Count(i) = nnz(values == uniqueValues(i));
end
end

function n = countGeometryPoints(items)
n = 0;
if isempty(items)
    return
end
propNames = properties(items(1));
if ~any(strcmp("Geometry", propNames))
    return
end

for i = 1:numel(items)
    geom = items(i).Geometry;
    if isnumeric(geom)
        n = n + size(geom, 1);
    end
end
end

function [allPts, sourceCounts] = collectAllGeometry(rrMap)
sources = ["Lanes", "LaneBoundaries", "Junctions", "Barriers", "Signs", "Signals", "StaticObjects", "ParkingSpaces"];
allPts = zeros(0, 3);
sourceCounts = struct;

for i = 1:numel(sources)
    source = sources(i);
    items = getPropOrEmpty(rrMap, source);
    pts = collectGeometryFromArray(items);
    allPts = [allPts; pts]; %#ok<AGROW>
    sourceCounts.(source) = size(pts, 1);
end
end

function pts = collectGeometryFromArray(items)
pts = zeros(0, 3);
if isempty(items)
    return
end

for i = 1:numel(items)
    pts = [pts; collectGeometry(items(i))]; %#ok<AGROW>
end
end

function pts = collectGeometry(item)
pts = zeros(0, 3);
if ~isprop(item, "Geometry")
    return
end

geom = item.Geometry;
pts = collectGeometryValue(geom);
end

function pts = collectGeometryValue(geom)
pts = zeros(0, 3);

if isnumeric(geom)
    if size(geom, 2) >= 2
        if size(geom, 2) == 2
            geom = [geom, zeros(size(geom, 1), 1)];
        end
        pts = geom(:, 1:3);
    end
    return
end

if isobject(geom)
    propNames = properties(geom);
    for k = 1:numel(propNames)
        value = geom.(propNames{k});
        if isnumeric(value) && size(value, 2) >= 2
            if size(value, 2) == 2
                value = [value, zeros(size(value, 1), 1)];
            end
            pts = [pts; value(:, 1:3)]; %#ok<AGROW>
        elseif isobject(value)
            pts = [pts; collectGeometryValue(value)]; %#ok<AGROW>
        end
    end
end
end

function printSummary(summary)
fprintf("\nRRHD file: %s\n", summary.File);
fprintf("GeoReference: %s\n", mat2str(summary.GeoReference));
fprintf("\nEntity counts:\n");
countNames = string(fieldnames(summary.Counts));
for i = 1:numel(countNames)
    fprintf("  %-26s %d\n", countNames(i), summary.Counts.(countNames(i)));
end

if ~isempty(summary.Bounds)
    fprintf("\nGeometry bounds in map coordinates:\n");
    disp(summary.Bounds);
    fprintf("Approximate XY footprint: %.1f m x %.1f m\n", ...
        summary.WidthMeters, summary.HeightMeters);
end

if ~isempty(summary.LaneTypes)
    fprintf("\nLane types:\n");
    disp(summary.LaneTypes);
end

if ~isempty(summary.TravelDirections)
    fprintf("\nTravel directions:\n");
    disp(summary.TravelDirections);
end

fprintf("Lane geometry points: %d\n", summary.LaneGeometryPointCount);
fprintf("Lane boundary geometry points: %d\n", summary.LaneBoundaryGeometryPointCount);
end

function plotMapGeometry(ax, rrMap, summary, mode)
if nargin < 4
    mode = "All";
end
mode = string(mode);
hold(ax, "on");
axis(ax, "equal");
grid(ax, "on");
box(ax, "on");
title(ax, mode);
xlabel(ax, "X (m)");
ylabel(ax, "Y (m)");

switch mode
    case "All"
        plotEntityArray(ax, getPropOrEmpty(rrMap, "LaneBoundaries"), [0.45 0.45 0.45], 0.5);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Junctions"), [0.85 0.55 0.15], 0.8);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Barriers"), [0.65 0.25 0.25], 0.8);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "StaticObjects"), [0.30 0.30 0.30], 0.8);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Lanes"), [0.00 0.35 0.80], 1.0);
        legend(ax, ["Lane boundaries", "Junctions", "Barriers/objects", "Lanes"], ...
            "Location", "bestoutside");
    case "Lanes"
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Lanes"), [0.00 0.35 0.80], 0.8);
    case "LaneBoundaries"
        plotEntityArray(ax, getPropOrEmpty(rrMap, "LaneBoundaries"), [0.45 0.45 0.45], 0.5);
    case "Junctions"
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Junctions"), [0.85 0.55 0.15], 0.8);
    case "Objects"
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Barriers"), [0.65 0.25 0.25], 0.8);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Signs"), [0.35 0.65 0.35], 0.8);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "Signals"), [0.85 0.15 0.15], 0.8);
        plotEntityArray(ax, getPropOrEmpty(rrMap, "StaticObjects"), [0.30 0.30 0.30], 0.8);
end

if isfield(summary, "Bounds") && ~isempty(summary.Bounds)
    xlim(ax, [summary.Bounds.Min(1), summary.Bounds.Max(1)]);
    ylim(ax, [summary.Bounds.Min(2), summary.Bounds.Max(2)]);
end
end

function plotEntityArray(ax, items, color, lineWidth)
if isempty(items)
    return
end

for i = 1:numel(items)
    pts = collectGeometry(items(i));
    plotPointSet(ax, pts, color, lineWidth);
end
end

function plotPointSet(ax, pts, color, lineWidth)
if isempty(pts)
    return
end

if size(pts, 1) == 1
    plot(ax, pts(:, 1), pts(:, 2), ".", "Color", color, "MarkerSize", 8);
else
    plot(ax, pts(:, 1), pts(:, 2), "-", "Color", color, "LineWidth", lineWidth);
end
end
