function analyzeClosedLoop()
% analyzeClosedLoop Summarize the closed-loop alias-hypothesis replays.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    files=dir('output/alias_hypotheses_20260924/closed_loop*.mat');R=struct();
    for f=files.',L=load(fullfile(f.folder,f.name),'results');for name=string(fieldnames(L.results)).',R.(name)=L.results.(name);end,end
    names=string(fieldnames(R)).';
    base=R.current;segments=[158 214;421 426;841 853;875 879;955 959];
    rows=cell(0,12);
    for name=names
        T=R.(name);a=logical(T.accepted);e=T.errorM;
        rows(end+1,:)={name,nnz(a),100*rms(e(a)),100*median(e(a)),100*prctile(e(a),95),100*max(e(a)),nnz(e(a)>.3),nnz(e(a)>.5), ...
            rms(T.yawErrorDeg(a)),nnz(e>base.errorM+.10),nnz(base.errorM>.3 & e<.15),1e3*mean(T.seconds)}; %#ok<AGROW>
    end
    S=cell2table(rows,VariableNames={'mode','accepted','rmseCm','medianCm','p95Cm','maxCm','above30cm','above50cm','yawRmseDeg','worseThanCurrentBy10cm','rescuedFrames','msPerFrame'});
    disp(S);writetable(S,fullfile(dest,'closed_loop_summary.csv'));
    fprintf('\nsegment (frames)      ');fprintf('%14s',names);fprintf('\n');
    for s=1:size(segments,1)
        k=segments(s,1):segments(s,2);fprintf('%4d-%4d max/median ',segments(s,1),segments(s,2));
        for name=names,e=R.(name).errorM(k);fprintf('  %5.1f /%5.1f ',100*max(e),100*median(e));end;fprintf('\n');
    end
    fprintf('\nframes above 30 cm:\n');
    for name=names,fprintf('%-13s %s\n',name,mat2str(find(R.(name).errorM>.3 & logical(R.(name).accepted)).'));end
    fig=figure('Visible','off','Color','w','Position',[100 100 1250 700]);theme(fig,'light');tiledlayout(2,1,TileSpacing='compact');
    t=(1:height(base))/10;
    nexttile;hold on;
    for name=names,plot(t,100*R.(name).errorM,DisplayName=name);end
    ylabel('Matching position error (cm)');grid on;legend(Interpreter='none');ylim([0 80]);title('LiDAR-only recursive matching, closed loop');
    nexttile;hold on;
    for name=names(2:end),plot(t,R.(name).candidates,DisplayName=name+" candidates");end
    ylabel('Solves per frame');xlabel('Time since first scan (s)');grid on;legend(Interpreter='none');
    exportgraphics(fig,fullfile(dest,'closed_loop_comparison.png'),Resolution=140);close(fig);
end
