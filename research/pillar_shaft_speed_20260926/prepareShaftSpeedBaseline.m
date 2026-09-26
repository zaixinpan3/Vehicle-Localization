function targetDir = prepareShaftSpeedBaseline(targetDir)
% prepareShaftSpeedBaseline: Rebuild the frozen shaft baseline from Git objects.
% A supported MEX C++ compiler and the immutable commit below are required.
% Only ignored study outputs are written. Existing unmarked or incomplete
% folders are preserved; pass a fresh folder name to rebuild separately.
    root = setupVehicleLocalization();
    outputRoot = fullfile(root,'output','pillar_shaft_speed_20260926');
    if nargin<1 || isempty(targetDir), targetDir=fullfile(outputRoot,'baseline'); end
    outputRoot = canonicalPath(outputRoot);
    targetDir = canonicalPath(targetDir);
    assert(startsWith(string(targetDir),string(outputRoot)+filesep), ...
        'Baseline targets must be inside output/pillar_shaft_speed_20260926.');
    assert(~isfile(targetDir),'The target path is already a file.');
    commit = '35cdb88388300ba1b8bb215905435bde670dff03';
    repositoryFiles = {
        'config/structuralPillarConfig.m'
        'config/pillarShaftConfig.m'
        'perception/offGroundFeatures/analyzeStructuralPillars.m'
        'perception/offGroundFeatures/detectPolePillars.m'
        'perception/offGroundFeatures/findPillarShaftModes.m'
        'perception/offGroundFeatures/assignPillarShaftSupport.m'
        'perception/perceptionNativeAvailable.m'
        'perception/native/perceptionKernelsMex.cpp'
        'perception/native/poleSubsetKernel.hpp'
        'perception/native/pillarShaftKernel.hpp'};
    localNames = cell(size(repositoryFiles));
    for k=1:numel(repositoryFiles)
        [~,name,extension]=fileparts(repositoryFiles{k});
        localNames{k}=[name extension];
    end
    binaryName = ['perceptionKernelsMex.' mexext];
    manifestName = 'baseline_manifest.json';
    if isfolder(targetDir)
        manifestFile=fullfile(targetDir,manifestName);
        assert(isfile(manifestFile), ...
            'Existing baseline folder has no manifest; preserve it and choose a fresh target.');
        manifest=jsondecode(fileread(manifestFile));
        assert(strcmp(manifest.commit,commit) && manifest.nativeVersion==5 && ...
            strcmp(manifest.binary,binaryName), ...
            'Existing baseline manifest is incompatible; choose a fresh target.');
        required=[localNames;{binaryName}];
        assert(all(cellfun(@(name)isfile(fullfile(targetDir,name)),required)), ...
            'Existing baseline is incomplete; preserve it and choose a fresh target.');
        fprintf('Reusing complete frozen baseline: %s\n',targetDir);
        return;
    end
    if ~isfolder(outputRoot), mkdir(outputRoot); end
    temporary=tempname(outputRoot); mkdir(temporary);
    originalDirectory=pwd; originalPath=path;
    temporaryCleanup=onCleanup(@()removeTemporary(temporary));
    environmentCleanup=onCleanup(@()restoreEnvironment( ...
        originalDirectory,originalPath,temporary));
    cd(root);
    for k=1:numel(repositoryFiles)
        % Both the revision and repository path are fixed trusted literals;
        % no user-provided path is interpolated into a shell command.
        [status,contents]=system(['git --no-pager -c color.ui=false show ' commit ':' repositoryFiles{k}]);
        assert(status==0,'Cannot export frozen source %s: %s',repositoryFiles{k},contents);
        writeBytes(fullfile(temporary,localNames{k}),unicode2native(contents,'UTF-8'));
    end
    options={};
    if isunix, options={'CXXFLAGS=$CXXFLAGS -std=c++17'}; end
    if ispc, options={'COMPFLAGS=$COMPFLAGS /std:c++17'}; end
    postBuildInspection=false;
    try
        mex('-R2018a','-O',options{:},'-outdir',temporary, ...
            fullfile(temporary,'perceptionKernelsMex.cpp'));
    catch exception
        % Accept only the known inspection error, and only after executing
        % the freshly linked binary successfully below.
        if ~contains(exception.message,'is not a MEX file'), rethrow(exception); end
        postBuildInspection=true;
    end
    binary=fullfile(temporary,binaryName);
    assert(isfile(binary),'The frozen baseline build produced no MEX binary.');
    shaftSpeedUseBackend(originalPath,temporary,true);
    assert(strcmp(which('perceptionKernelsMex'),binary), ...
        'Smoke validation must load the newly built frozen binary.');
    assert(isequal(perceptionKernelsMex('version'),5),'Frozen native version mismatch.');
    mask=perceptionKernelsMex('growRoad',logical([1 1;0 1]),1, ...
        [0 0.1;0 0.2],[0 0.15 inf]);
    assert(isequal(mask,logical([1 1;0 1])),'Frozen native smoke validation failed.');
    shaftSpeedUseBackend(originalPath,temporary,false);
    manifest=struct('commit',commit,'nativeVersion',5,'binary',binaryName, ...
        'repositoryFiles',{repositoryFiles},'localFiles',{localNames}, ...
        'compilerOptions',{options},'mexOptions',{{'-R2018a','-O'}}, ...
        'postBuildInspectionError',postBuildInspection, ...
        'smokeValidation','Exact version 5 and growRoad output passed', ...
        'preparedAtUtc',char(datetime('now','TimeZone','UTC', ...
            'Format',"yyyy-MM-dd'T'HH:mm:ss'Z'")));
    writeBytes(fullfile(temporary,manifestName), ...
        unicode2native([jsonencode(manifest,PrettyPrint=true) newline],'UTF-8'));
    parent=fileparts(targetDir);
    if ~isfolder(parent), mkdir(parent); end
    assert(~isfolder(targetDir) && ~isfile(targetDir), ...
        'Target appeared during preparation; refusing to overwrite it.');
    [ok,message]=movefile(temporary,targetDir);
    assert(ok,'Could not admit the validated frozen baseline: %s',message);
    fprintf('Prepared frozen baseline %s at %s\n',commit,targetDir);
    if postBuildInspection
        warning('perception:FrozenMexPostBuildInspection', ...
            'mex inspection reported an error; the fresh baseline passed load-and-execute validation.');
    end
end

function resolved=canonicalPath(folder)
    file=java.io.File(char(folder));
    resolved=char(file.getCanonicalPath());
end

function writeBytes(filename,bytes)
    file=fopen(filename,'wb');
    assert(file>=0,'Could not create baseline artifact: %s',filename);
    cleanup=onCleanup(@()fclose(file));
    assert(fwrite(file,bytes,'uint8')==numel(bytes),'Incomplete write: %s',filename);
end

function restoreEnvironment(directory,originalPath,temporary)
    cd(directory);
    shaftSpeedUseBackend(originalPath,temporary,false);
end

function removeTemporary(folder)
    if isfolder(folder), rmdir(folder,'s'); end
end
