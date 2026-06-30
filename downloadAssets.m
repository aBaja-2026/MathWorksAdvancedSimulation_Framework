% downloadAssets
% Downloads the local 3D simulation assets required by ControlVehicle.slx.

driveFolderId = "1KBWsERstzlyzwUZIvQli-Kxdr_SiJZV3";

repoRoot = fileparts(mfilename("fullpath"));
assetFolder = fullfile(repoRoot, "localFolder");
requiredFiles = {
    fullfile(assetFolder, "Windows", "AutoVrtlEnv", "Binaries", "Win64", "AutoVrtlEnv.exe")
    fullfile(assetFolder, "IndianCityBlock.xodr")
};

if requiredAssetsExist(requiredFiles)
    fprintf("Required simulation assets are already available in %s\n", assetFolder);
    listFolder(assetFolder, 0);
    return
end

fprintf("Downloading Google Drive folder to %s\n", assetFolder);
if ~isfolder(assetFolder)
    mkdir(assetFolder);
end
downloadGoogleDriveFolder(driveFolderId, assetFolder);

fprintf("\nDownloaded asset folder contents:\n");
listFolder(assetFolder, 0);

function tf = requiredAssetsExist(requiredFiles)
    tf = true;
    for i = 1:numel(requiredFiles)
        tf = tf && isfile(requiredFiles{i});
    end
end

function downloadGoogleDriveFolder(folderId, outputFolder)
    items = listGoogleDriveFolder(folderId);

    if isempty(items)
        error("No downloadable files were found. Make sure the Google Drive folder is shared publicly.");
    end

    for i = 1:numel(items)
        item = items(i);
        targetPath = fullfile(outputFolder, item.name);

        if item.isFolder
            if ~isfolder(targetPath)
                mkdir(targetPath);
            end
            fprintf("Entering folder: %s\n", item.name);
            downloadGoogleDriveFolder(item.id, targetPath);
        else
            fprintf("Downloading file: %s\n", item.name);
            downloadGoogleDriveFile(item.id, targetPath);
        end
    end
end

function items = listGoogleDriveFolder(folderId)
    folderUrl = "https://drive.google.com/embeddedfolderview?id=" + folderId;
    html = webread(folderUrl, weboptions("Timeout", 60));
    html = string(html);

    pattern = '<a[^>]+href="https://drive\.google\.com/(file/d|drive/folders)/([^"/?]+)[^"]*"[^>]*>(.*?)</a>';
    matches = regexp(html, pattern, "tokens");

    items = struct("id", {}, "name", {}, "isFolder", {});
    seenIds = strings(0);

    for i = 1:numel(matches)
        match = matches{i};
        type = string(match{1});
        id = string(match{2});
        name = cleanHtmlText(match{3});

        if id == folderId || any(seenIds == id) || strlength(name) == 0
            continue
        end

        seenIds(end + 1) = id;
        items(end + 1).id = id;
        items(end).name = makeSafeFileName(name);
        items(end).isFolder = type == "drive/folders";
    end
end

function downloadGoogleDriveFile(fileId, targetPath)
    downloadUrl = "https://drive.google.com/uc?export=download&id=" + fileId;
    tempPath = targetPath + ".download";

    websave(tempPath, downloadUrl, weboptions("Timeout", Inf));

    fileText = "";
    if getFileSize(tempPath) < 1024 * 1024
        try
            fileText = string(fileread(tempPath));
        catch
            fileText = "";
        end
    end

    if contains(fileText, "download-form")
        confirmUrl = getGoogleDriveConfirmUrl(fileText);
        websave(tempPath, confirmUrl, weboptions("Timeout", Inf));
    end

    if getFileSize(tempPath) < 1024 * 1024
        try
            fileText = string(fileread(tempPath));
        catch
            fileText = "";
        end

        if contains(fileText, "<html") && contains(fileText, "Google Drive")
            error("Google Drive returned an HTML page instead of file data for file ID %s.", fileId);
        end
    end

    if isfile(targetPath)
        delete(targetPath);
    end
    movefile(tempPath, targetPath);

    if endsWith(lower(targetPath), ".zip")
        unzipFolder = erase(targetPath, ".zip");
        if ~isfolder(unzipFolder)
            mkdir(unzipFolder);
        end
        unzip(targetPath, unzipFolder);
    end
end

function confirmUrl = getGoogleDriveConfirmUrl(html)
    action = regexp(html, '<form[^>]+id="download-form"[^>]+action="([^"]+)"', "tokens", "once");
    if isempty(action)
        error("Could not find the Google Drive confirmation form.");
    end

    inputPattern = '<input[^>]+type="hidden"[^>]+name="([^"]+)"[^>]+value="([^"]*)"';
    inputs = regexp(html, inputPattern, "tokens");
    if isempty(inputs)
        error("Could not find the Google Drive confirmation fields.");
    end

    queryParts = strings(1, numel(inputs));
    for i = 1:numel(inputs)
        name = string(inputs{i}{1});
        value = string(inputs{i}{2});
        queryParts(i) = urlencode(name) + "=" + urlencode(value);
    end

    confirmUrl = string(action{1}) + "?" + strjoin(queryParts, "&");
end

function text = cleanHtmlText(htmlText)
    text = regexprep(string(htmlText), "<[^>]*>", "");
    text = replace(text, "&amp;", "&");
    text = replace(text, "&lt;", "<");
    text = replace(text, "&gt;", ">");
    text = replace(text, "&quot;", '"');
    text = replace(text, "&#39;", "'");
    text = strtrim(text);
end

function name = makeSafeFileName(name)
    name = regexprep(name, '[<>:"/\\|?*]', "_");
    name = strtrim(name);
end

function bytes = getFileSize(filePath)
    info = dir(filePath);
    bytes = info.bytes;
end

function listFolder(folderPath, depth)
    maxDepth = 2;
    items = dir(folderPath);
    items = items(~ismember({items.name}, {'.', '..'}));

    indent = repmat('  ', 1, depth);
    for i = 1:numel(items)
        item = items(i);
        if item.isdir
            fprintf("%s%s/\n", indent, item.name);
            if depth < maxDepth
                listFolder(fullfile(item.folder, item.name), depth + 1);
            end
        else
            fprintf("%s%s (%s)\n", indent, item.name, formatBytes(item.bytes));
        end
    end
end

function text = formatBytes(bytes)
    units = ["B", "KB", "MB", "GB"];
    value = double(bytes);
    unitIndex = 1;

    while value >= 1024 && unitIndex < numel(units)
        value = value / 1024;
        unitIndex = unitIndex + 1;
    end

    if unitIndex == 1
        text = sprintf("%d %s", bytes, units(unitIndex));
    else
        text = sprintf("%.1f %s", value, units(unitIndex));
    end
end
