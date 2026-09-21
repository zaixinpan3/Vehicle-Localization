function clock = loadReceiverClock(inspvaPath)
% loadReceiverClock Read the shared Python-fitted clock and verify its source.
% Prepare with: python scripts/receiverClock.py <native_inspva.csv>
% No independent fit or piecewise-header fallback is performed in MATLAB.
    arguments
        inspvaPath (1,1) string
    end
    [folder,name]=fileparts(inspvaPath);
    path=fullfile(folder,name+".clock.json");
    assert(isfile(path),'VehicleLocalization:MissingReceiverClock', ...
        'Prepare the shared clock first: python scripts/receiverClock.py "%s"',inspvaPath);
    clock=jsondecode(fileread(path));
    fid=fopen(inspvaPath,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    bytes=fread(fid,Inf,'*uint8');
    digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
    hash=reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]);
    assert(isfield(clock,'sourceSha256') && strcmpi(hash,clock.sourceSha256), ...
        'VehicleLocalization:StaleReceiverClock','Clock source changed; regenerate the shared model.');
    assert(isfield(clock,'modelId') && strlength(string(clock.modelId))==64 && ...
        clock.offlineUsesFutureSamples && ~clock.absoluteLatencyCalibrated, ...
        'VehicleLocalization:InvalidReceiverClock','Missing clock identity or invalid calibration claim.');
    receiverClockTime(clock,clock.sourceOriginSeconds);
    coefficients=[clock.sourceOriginSeconds,clock.scale,clock.offsetSeconds, ...
        clock.sourceSpanSeconds,clock.maximumExtrapolationSeconds, ...
        clock.receiverOriginGpsWeek,clock.receiverOriginGpsSeconds];
    [~,~,endian]=computer;
    if endian=='B',coefficients=swapbytes(coefficients);end
    digest=java.security.MessageDigest.getInstance('SHA-256');
    digest.update(typecast(coefficients,'uint8'));
    hash=reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]);
    assert(isfield(clock,'coefficientSha256') && strcmpi(hash,clock.coefficientSha256), ...
        'VehicleLocalization:InvalidReceiverClock','Clock coefficients changed; regenerate the model.');
end
