
% extractPointCloudsFromBag: Convert the organized sensor_msgs/PointCloud2
% stream of a ROS1 bag into the MAT file of organized frames consumed by the
% perception module. Every frame is stored as a struct with [H x W] fields
% x, y, z, intensity, reflectivity, ambient, range, and a timestamp; the XYZ
% coordinates are rotated by the LiDAR extrinsic into the vehicle frame
% (x forward, y left, z up). Set bagPath and outputMatPath before running,
% or accept the defaults below. Requires ROS Toolbox.
%
% Input:
%   bagPath, outputMatPath, topicName, chunkSize, maxFrames, overwriteOutput
%       (optional workspace variables)
%
% Output:
%   MAT file with a struct array pointClouds, one struct per frame
if ~exist("bagPath", "var") || strlength(string(bagPath)) == 0
    bagPath = fullfile("data", "raw", "Missisipi", "raw_data_2024-06-07-12-09-31_0.bag");
end
if ~exist("outputMatPath", "var") || strlength(string(outputMatPath)) == 0
    outputMatPath = fullfile("data", "raw", "MissisipiPointClouds.mat");
end
if ~exist("topicName", "var")
    topicName = "";
end

% --- Validation ---
assert(exist("rosbag", "file") == 2, ...
    "MATLAB Robotics System Toolbox is required (missing function 'rosbag').");
assert(isfile(bagPath), "Bag file not found: %s", bagPath);

% --- Rotation Matrix (LiDAR Extrinsic) ---
baseRotationMatrix = [ ...
    0.925216, -0.367871, 0.093195; ...
    0.368647, 0.929524, 0.009468; ...
    -0.090110, 0.025595, 0.995625];

additionalRotationMatrix = [ ...
    0.931395, 0.364011, 0; ...
    -0.364011, 0.931395, 0; ...
    0, 0, 1];

rotationMatrix = additionalRotationMatrix * baseRotationMatrix;

% --- Main Processing ---
bag = rosbag(bagPath);

if strlength(string(topicName)) == 0
    topicName = inferPointCloud2Topic(bag);
end

ensureTopicIsPointCloud2(bag, topicName);

% Create reader
sel = select(bag, "Topic", char(topicName));
numMsgs = sel.NumMessages;
assert(numMsgs > 0, "No messages found for topic: %s", topicName);        

% --- Core processing (chunked to avoid OOM) ---
rotationMatrixSingle = single(rotationMatrix);
if ~exist("chunkSize", "var") || isempty(chunkSize)
    chunkSize = 50;
end
if ~exist("overwriteOutput", "var")
    overwriteOutput = true;
end
if ~exist("maxFrames", "var") || isempty(maxFrames)
    numToProcess = numMsgs;
else
    numToProcess = min(numMsgs, double(maxFrames));
end

if overwriteOutput && isfile(outputMatPath)
    warning("Deleting existing output file: %s", outputMatPath);
    delete(outputMatPath);
end

fprintf("Extracting %d/%d frames from topic %s\n", numToProcess, numMsgs, string(topicName));

frameTemplate = struct( ...
    "x", [], ...
    "y", [], ...
    "z", [], ...
    "intensity", [], ...
    "reflectivity", [], ...
    "ambient", [], ...
    "range", [], ...
    "timestamp", []);

pointClouds(1, numToProcess) = frameTemplate;

organizedHeight = [];
organizedWidth = [];

for startIdx = 1:chunkSize:numToProcess
    endIdx = min(startIdx + chunkSize - 1, numToProcess);
    idxRange = startIdx:endIdx;

    msgs = readMessages(sel, idxRange, "DataFormat", "struct");
    numChunk = numel(msgs);

    expectedChunk = numel(idxRange);
    if numChunk ~= expectedChunk
        warning("readMessages returned %d messages for an index range of %d; missing frames will be left empty.", numChunk, expectedChunk);
    end
    framesChunk = repmat(frameTemplate, 1, expectedChunk);
    loopCount = min(numChunk, expectedChunk);

    for j = 1:loopCount
        msg = msgs{j};

        H = double(getProp(msg, "Height", "height", NaN));
        W = double(getProp(msg, "Width", "width", NaN));
        assert(~isnan(H) && ~isnan(W), "Missing Height/Width in PointCloud2 message.");
        assert(H > 1 && W > 1, "PointCloud2 is not organized (Height=%g, Width=%g).", H, W);

        if isempty(organizedHeight)
            organizedHeight = H;
            organizedWidth = W;
        else
            assert(H == organizedHeight && W == organizedWidth, ...       
                "Organized shape changed across frames: expected %dx%d, got %dx%d.", ...
                organizedHeight, organizedWidth, H, W);
        end

        frame = frameTemplate;
        frame.timestamp = getMsgTimestampSec(msg);

        % XYZ (float32) -> rotate -> organized HxW
        % Note: ROS organized PointCloud2 is row-major; MATLAB reshape is column-major.
        xyz = single(rosReadXYZ(msg));
        assert(size(xyz, 1) == H * W && size(xyz, 2) == 3, ...
            "Unexpected xyz shape: got %dx%d, expected %dx3.", size(xyz, 1), size(xyz, 2), H * W);

        xyzRot = xyz * rotationMatrixSingle.';
        frame.x = reshapeRosRowMajorToHW(xyzRot(:, 1), H, W);
        frame.y = reshapeRosRowMajorToHW(xyzRot(:, 2), H, W);
        frame.z = reshapeRosRowMajorToHW(xyzRot(:, 3), H, W);

        intensity = tryRosReadField(msg, "intensity");
        if isempty(intensity)
            frame.intensity = NaN(H, W, "single");
        else
            frame.intensity = reshapeRosRowMajorToHW(single(intensity), H, W);
        end

        reflectivity = tryRosReadField(msg, "reflectivity");
        if isempty(reflectivity)
            frame.reflectivity = zeros(H, W, "uint16");
        else
            frame.reflectivity = reshapeRosRowMajorToHW(uint16(reflectivity), H, W);
        end

        ambient = tryRosReadField(msg, "ambient");
        if isempty(ambient)
            frame.ambient = zeros(H, W, "uint16");
        else
            frame.ambient = reshapeRosRowMajorToHW(uint16(ambient), H, W);
        end

        rangeVal = tryRosReadField(msg, "range");
        if isempty(rangeVal)
            frame.range = NaN(H, W, "single");
        else
            if isinteger(rangeVal)
                rangeVal = single(rangeVal) * single(0.001);
            else
                rangeVal = single(rangeVal);
            end
            frame.range = reshapeRosRowMajorToHW(rangeVal, H, W);
        end

        framesChunk(j) = frame;
    end

    pointClouds(1, idxRange) = framesChunk;
    fprintf("Processed %d/%d frames (%.1f%%)\n", endIdx, numToProcess, 100.0 * endIdx / numToProcess);
end

save(outputMatPath, "pointClouds", "-v7.3");
fprintf("Done. Saved %d frames to %s\n", numToProcess, outputMatPath);
% Core Logic Functions
function topicName = inferPointCloud2Topic(bag)
    % inferPointCloud2Topic: Select a PointCloud2 topic from a rosbag,
    % preferring the first available point cloud stream.
    %
    % Input:
    %   bag: rosbag object
    %
    % Output:
    %   topicName: string topic name for a PointCloud2 stream
    topics = bag.AvailableTopics;
    msgTypes = string(topics.MessageType);
    isPc2 = contains(msgTypes, "PointCloud2", "IgnoreCase", true);        
    pcTopics = topics.Properties.RowNames(isPc2);
    if isempty(pcTopics)
        disp("Available topics in bag:");
        disp(topics(:, intersect(["MessageType","NumMessages"], string(topics.Properties.VariableNames), "stable")));
        error("No PointCloud2 topics found in bag.");
    end
    topicName = string(pcTopics{1});
    if numel(pcTopics) > 1
        fprintf("Multiple PointCloud2 topics found; using %s\n", topicName);
    end
end

function ensureTopicIsPointCloud2(bag, topicName)
    % ensureTopicIsPointCloud2: Verify that a named topic exists and is a
    % PointCloud2 message stream in the rosbag.
    %
    % Input:
    %   bag: rosbag object
    %   topicName: char/string topic name to validate
    %
    % Output:
    %   none
    topics = bag.AvailableTopics;
    topicName = string(topicName);
    if ~any(string(topics.Properties.RowNames) == topicName)
        disp("Available topics in bag:");
        disp(topics(:, intersect(["MessageType","NumMessages"], string(topics.Properties.VariableNames), "stable")));
        error("Topic not found: %s", topicName);
    end
    msgType = string(topics{char(topicName), "MessageType"});
    if ~contains(msgType, "PointCloud2", "IgnoreCase", true)
        error("Selected topic is not PointCloud2: %s (MessageType=%s)", topicName, msgType);
    end
end

function val = getProp(obj, upperName, lowerName, defaultVal)
% getProp: Read a property or struct field with fallback names, returning
% a default value when the field/property is missing.
%
% Input:
%   obj: struct or object to query
%   upperName: char/string primary property/field name
%   lowerName: char/string alternate property/field name
%   defaultVal: value to return when the field/property is missing
%
% Output:
%   val: property/field value or defaultVal when missing
if nargin < 4
    defaultVal = [];
end
if isstruct(obj)
    if isfield(obj, lowerName)
        val = obj.(lowerName);
        return;
    end
    if isfield(obj, upperName)
        val = obj.(upperName);
        return;
    end
else
    if isprop(obj, upperName)
        val = obj.(upperName);
        return;
    end
    if isprop(obj, lowerName)
        val = obj.(lowerName);
        return;
    end
end
val = defaultVal;
end

function val = tryRosReadField(msg, fieldName)
% tryRosReadField: Read a PointCloud2 field from a message, returning []
% when the field is absent.
%
% Input:
%   msg: ROS PointCloud2 message (struct or object)
%   fieldName: char/string field name
%
% Output:
%   val: field data or [] if the field is missing
val = [];
try
    val = rosReadField(msg, char(fieldName));
catch
    val = [];
end
end

function out = reshapeRosRowMajorToHW(vec, height, width)
% reshapeRosRowMajorToHW: Convert row-major PointCloud2 data into an
% organized [H x W] array matching MATLAB indexing.
%
% Input:
%   vec: [H*W x 1] vector in row-major order
%   height: scalar height H
%   width: scalar width W
%
% Output:
%   out: [H x W] array with MATLAB column-major ordering
out = reshape(vec, [width, height]).';
end

function stamp = getMsgTimestampSec(msg)
% getMsgTimestampSec: Convert a ROS header timestamp into seconds with
% fractional nanoseconds, returning NaN when missing.
%
% Input:
%   msg: ROS message (struct or object) with Header/Stamp fields
%
% Output:
%   stamp: scalar seconds (double), NaN if timestamp is missing
stamp = NaN;
hdr = getProp(msg, "Header", "header", []);
if isempty(hdr), return; end
st = getProp(hdr, "Stamp", "stamp", []);
if isempty(st), return; end
sec = double(getProp(st, "Sec", "sec", 0));
nsec = double(getProp(st, "Nsec", "nsec", 0));
stamp = sec + nsec * 1e-9;
end
