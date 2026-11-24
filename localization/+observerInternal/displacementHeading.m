function yaw=displacementHeading(t,fix,measuredHeadingValid,twist,motionValid,offset,positionStdM)
% Align measured and body-integrated antenna displacements at the current fix.
% Fit one planar rotation to resolved chords during a receiver-heading gap.
    yaw=nan(size(t));
    validateattributes(positionStdM,{'numeric'},{'scalar','real','finite','nonnegative'});
    twist(~motionValid,:)=0;
    [odom,segment]=integrateRecordedPlanarMotion(t,twist,t,motionValid);
    relativeYaw=odom(:,3);
    c=cos(relativeYaw);s=sin(relativeYaw);
    bodyAntenna=odom(:,1:2)+[c*offset(1)-s*offset(2),s*offset(1)+c*offset(2)];
    mapAntenna=fix.values(:,1:2);
    % The two endpoint position errors can add in a displacement.
    resolution=2*positionStdM;
    anchor=0;next=1;alignment=zeros(2,1);
    for k=1:numel(t)
        if measuredHeadingValid(k),alignment(:)=0;end
        % A synchronized fix may depend on a later native interpolation endpoint.
        while next<=k && fix.availableAt(next)<=t(k)
            j=next;next=next+1;
            if ~fix.valid(j) || ~motionValid(j),continue;end
            if anchor==0 || segment(j)~=segment(anchor)
                anchor=j;alignment(:)=0;
                continue
            end
            measured=mapAntenna(j,:)-mapAntenna(anchor,:);
            integrated=bodyAntenna(j,:)-bodyAntenna(anchor,:);
            if min(norm(measured),norm(integrated))<=resolution,continue;end
            anchor=j;
            if fix.valid(k) && ~measuredHeadingValid(k) && segment(k)==segment(j)
                % Cosine and sine coefficients of the least-squares chord fit.
                perpendicular=[-integrated(2),integrated(1)];
                alignment=alignment+[dot(measured,integrated);dot(measured,perpendicular)];
                if any(alignment)
                    direction=[c(k),-s(k);s(k),c(k)]*alignment;
                    yaw(k)=atan2(direction(2),direction(1));
                end
            end
        end
    end
end
