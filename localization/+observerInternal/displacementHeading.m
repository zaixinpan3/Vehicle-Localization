function yaw=displacementHeading(t,fix,measuredHeadingValid,twist,motionValid,offset,positionStdM)
% Align measured and body-integrated antenna displacements at the current fix.
% Fit one planar rotation to resolved chords during a receiver-heading gap.
    yaw=nan(size(t));
    validateattributes(positionStdM,{'numeric'},{'scalar','real','finite','nonnegative'});
    twist(~motionValid,:)=0;
    [odom,segment]=integrateRecordedPlanarMotion(t,twist,t,motionValid);
    relativeYaw=odom(:,3);
    bodyAntenna=complex(odom(:,1),odom(:,2))+exp(1i*relativeYaw)*complex(offset(1),offset(2));
    mapAntenna=complex(fix.values(:,1),fix.values(:,2));
    % The two endpoint position errors can add in a displacement.
    resolution=2*positionStdM;
    anchor=0;next=1;crossMoment=0;
    for k=1:numel(t)
        if measuredHeadingValid(k),crossMoment=0;end
        % A synchronized fix may depend on a later native interpolation endpoint.
        while next<=k && fix.availableAt(next)<=t(k)
            j=next;next=next+1;
            if ~fix.valid(j) || ~motionValid(j),continue;end
            if anchor==0 || segment(j)~=segment(anchor)
                anchor=j;crossMoment=0;
                continue
            end
            measured=mapAntenna(j)-mapAntenna(anchor);
            integrated=bodyAntenna(j)-bodyAntenna(anchor);
            if min(abs(measured),abs(integrated))<=resolution,continue;end
            anchor=j;
            if fix.valid(k) && ~measuredHeadingValid(k) && segment(k)==segment(j)
                % arg(sum(measured .* conj(integrated))) minimizes the sum
                % of squared chord residuals over rotations in the plane.
                crossMoment=crossMoment+measured*conj(integrated);
                if abs(crossMoment)>0
                    yaw(k)=angle(crossMoment*exp(1i*relativeYaw(k)));
                end
            end
        end
    end
end
