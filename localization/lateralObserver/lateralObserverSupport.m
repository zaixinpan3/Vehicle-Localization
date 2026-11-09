classdef lateralObserverSupport
% lateralObserverSupport Bicycle model and gain scheduling of the lateral observer.
% Static methods build the LPV bicycle matrices, evaluate the model, map speed
% to scheduling coordinates and triangle vertices, blend vertex gains and
% verify the vehicle parameters of a stored design.
% Example: K = lateralObserverSupport.scheduleLateralObserverGain(design,speed).

    methods (Static)
        function model = lateralBicycleModel(vehicle)
        % lateralBicycleModel Build the nominal bicycle model with virtual output Vx*ay.
        % State x = [vy; r], steering delta, and y = Vx*ay = C*x + D*delta.
        % A(rho) = A0 + Vx*A1 + (1/Vx)*A2; C is constant, D = Vx*D1.
        % An optional copied nonlinearity adds Vx*f_1(x) to this output.
            requiredFields = ["mass", "yawInertia", "lf", "lr", "frontCorneringStiffness", "rearCorneringStiffness"];
            for fieldName = requiredFields
                assert(isfield(vehicle, fieldName) && isscalar(vehicle.(fieldName)) && ...
                    isfinite(vehicle.(fieldName)) && vehicle.(fieldName) > 0, ...
                    "vehicle.%s must be a positive finite scalar.", fieldName);
            end

            mass = double(vehicle.mass);
            yawInertia = double(vehicle.yawInertia);
            lf = double(vehicle.lf);
            lr = double(vehicle.lr);
            frontStiffness = double(vehicle.frontCorneringStiffness);
            rearStiffness = double(vehicle.rearCorneringStiffness);

            stiffnessSum = frontStiffness + rearStiffness;
            stiffnessMoment = (lr .* rearStiffness) - (lf .* frontStiffness);
            stiffnessSecondMoment = (lf.^2 .* frontStiffness) + (lr.^2 .* rearStiffness);

            model = struct();
            model.vehicle = vehicle;
            model.stateNames = ["lateralVelocity", "yawRate"];
            model.outputNames = "speedTimesLateralAcceleration";

            % A(rho) = A0 + Vx A1 + (1/Vx) A2
            model.A0 = zeros(2, 2);
            model.A1 = [0.0, -1.0; 0.0, 0.0];
            model.A2 = [-stiffnessSum ./ mass, stiffnessMoment ./ mass; ...
                stiffnessMoment ./ yawInertia, -stiffnessSecondMoment ./ yawInertia];

            model.C = model.A2(1, :);
            model.B = [frontStiffness ./ mass; (lf .* frontStiffness) ./ yawInertia];
            model.D1 = frontStiffness/mass;
        end

        function [A, C, D] = evaluateLateralModel(model, rho)
        % evaluateLateralModel: Evaluate the LPV lateral model at one scheduling
        % point. The two components of rho are treated as independent coordinates,
        % because the polytope vertices used by the synthesis do not all lie on the
        % physical curve rho = [Vx; 1/Vx].
        %
        % Input:
        %   model: struct from lateralObserverSupport.lateralBicycleModel
        %   rho: [2 x 1] scheduling point [rho1; rho2]
        %
        % Output:
        %   A: [2 x 2] state matrix at rho
        %   C: [1 x 2] constant virtual-output matrix
        %   D: scalar known steering feedthrough coefficient
            rho = double(rho(:));
            assert(numel(rho) == 2 && all(isfinite(rho)), "rho must be a finite [2 x 1] scheduling point.");
            A = model.A0 + (rho(1) .* model.A1) + (rho(2) .* model.A2);
            C = model.C;
            D = rho(1)*model.D1;
        end

        function alpha = schedulingCoordinates(polytope, longitudinalSpeed)
        % schedulingCoordinates Barycentric coordinates of [Vx;1/Vx].
        % The certificate also covers the derivative of these coordinates.
            longitudinalSpeed = double(longitudinalSpeed);
            assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed) && longitudinalSpeed > 0, ...
                "longitudinalSpeed must be a positive finite scalar.");
            alpha = polytope.S \ [longitudinalSpeed; 1.0 ./ longitudinalSpeed; 1.0];
        end

        function polytope = buildSchedulingPolytope(speedRange)
        % buildSchedulingPolytope: Build the triangle that covers the scheduling
        % curve rho(Vx) = [Vx; 1/Vx] over Vx in [Vmin, Vmax]. Because 1/Vx is
        % convex in Vx, the curve lies below its endpoint chord and above both
        % endpoint tangents. The intersection of those two tangents supplies
        % the third vertex of the triangle enclosing the whole arc:
        %
        %   rho1 = [Vmin; 1/Vmin]
        %   rho2 = [Vmax; 1/Vmax]
        %   rho3 = [2 Vmin Vmax/(Vmin+Vmax); 2/(Vmin+Vmax)]
        %
        % The third vertex takes the harmonic mean speed with the reciprocal of the
        % arithmetic mean, which is strictly below the curve because 4 Vmin Vmax
        % < (Vmin+Vmax)^2. The barycentric coordinates of any rho then follow from
        % the vertex matrix S, and they are nonnegative exactly because the arc is
        % inside the triangle.
        %
        % Input:
        %   speedRange: [1 x 2] positive longitudinal speed bounds [Vmin, Vmax]
        %
        % Output:
        %   polytope: struct with vertices [2 x 3], S [3 x 3], speedRange, and the
        %       vertex speeds used for reporting
            speedRange = double(speedRange(:).');
            assert(numel(speedRange) == 2 && all(isfinite(speedRange)) && all(speedRange > 0) && ...
                speedRange(1) < speedRange(2), ...
                "speedRange must be [Vmin, Vmax] with 0 < Vmin < Vmax.");

            minimumSpeed = speedRange(1);
            maximumSpeed = speedRange(2);
            harmonicMeanSpeed = (2.0 .* minimumSpeed .* maximumSpeed) ./ (minimumSpeed + maximumSpeed);

            vertices = [minimumSpeed, maximumSpeed, harmonicMeanSpeed; ...
                1.0 ./ minimumSpeed, 1.0 ./ maximumSpeed, 2.0 ./ (minimumSpeed + maximumSpeed)];
            S = [vertices; ones(1, 3)];
            assert(abs(det(S)) > eps(max(abs(S(:)))), "The scheduling polytope is degenerate for the requested speed range.");

            polytope = struct();
            polytope.vertices = vertices;
            polytope.S = S;
            polytope.speedRange = speedRange;
            polytope.vertexSpeeds = vertices(1, :);
        end

        function rate = schedulingCoordinateRates(polytope, longitudinalSpeed, speedRate)
        % schedulingCoordinateRates Differentiate [Vx; 1/Vx; 1] exactly.
            assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed) && longitudinalSpeed > 0, ...
                "longitudinalSpeed must be positive and finite.");
            assert(isscalar(speedRate) && isfinite(speedRate), "speedRate must be finite.");
            rate = polytope.S\[speedRate; -speedRate/longitudinalSpeed^2; 0];
        end

        function domain = buildCertificateDomain(model, polytope, speedRateRange)
        % buildCertificateDomain Position triangle and a four-corner rate hull.
        % The rate hull encloses all a*[1; -1/Vx^2; 0] after applying S^-1.
        % It deliberately permits independent position and rate vertices.
            speedRateRange = double(speedRateRange(:).');
            assert(numel(speedRateRange) == 2 && all(isfinite(speedRateRange)) && ...
                speedRateRange(1) <= 0 && speedRateRange(2) >= 0 && ...
                speedRateRange(1) < speedRateRange(2), ...
                "speedRateRange must be finite [amin,amax] with amin <= 0 <= amax and amin < amax.");
            domain = struct('numVertices', 3, 'vertexA', zeros(2, 2, 3), ...
                'speedRateRange', speedRateRange, 'coordinateRateVertices', zeros(3, 4));
            for idx = 1:3
                domain.vertexA(:, :, idx) = ...
                    lateralObserverSupport.evaluateLateralModel(model, polytope.vertices(:, idx));
            end
            for rateIdx = 1:2
                for speedIdx = 1:2
                    idx = 2*(rateIdx - 1) + speedIdx;
                    domain.coordinateRateVertices(:, idx) = lateralObserverSupport.schedulingCoordinateRates( ...
                        polytope, polytope.speedRange(speedIdx), speedRateRange(rateIdx));
                end
            end
            domain.numRateVertices = 4;
            domain.numVertexPairs = 6;
        end

        function [L, P, Pdot] = scheduleLateralObserverGain(design, longitudinalSpeed, speedRate)
        % scheduleLateralObserverGain Schedule P and Y together, then solve P*L=Y.
        % The gain is generally rational in rho, not a blend of vertex gains.
        % Endpoint clamping is for evaluation only; runtime withdraws aiding
        % outside the positive-speed interval. Pdot needs d(Vx)/dt, not ax.
            longitudinalSpeed = double(longitudinalSpeed);
            assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed), ...
                "The scheduling speed must be a finite scalar.");
            speedRange = design.polytope.speedRange;
            clampedSpeed = min(max(longitudinalSpeed, speedRange(1)), speedRange(2));
            alpha = lateralObserverSupport.schedulingCoordinates(design.polytope, clampedSpeed);
            P = sum(design.vertexLyapunovMatrices.*reshape(alpha, 1, 1, 3), 3);
            Y = sum(design.vertexY.*reshape(alpha, 1, 1, 3), 3);
            L = P\Y;
            if nargout > 2
                assert(nargin >= 3, "Supply the scheduling speed derivative to evaluate Pdot.");
                rates = lateralObserverSupport.schedulingCoordinateRates(design.polytope, clampedSpeed, speedRate);
                Pdot = sum(design.vertexLyapunovMatrices.*reshape(rates, 1, 1, 3), 3);
                if longitudinalSpeed < speedRange(1) || longitudinalSpeed > speedRange(2)
                    Pdot(:) = 0;
                end
            end
        end

        function assertLateralVehicleMatches(design,cfg)
        % assertLateralVehicleMatches Reject gains synthesized for another vehicle.
        % Passing an explicit sensitivity configuration is supported, but
        % a saved certificate cannot silently be relabeled with a different model.
            assert(isfield(design,'model') && isfield(design.model,'vehicle') && ...
                isfield(cfg,'vehicle') && isequaln(design.model.vehicle,cfg.vehicle), ...
                'VehicleLocalization:VehicleParameterMismatch', ...
                'Saved lateral gains do not match the requested vehicle. Re-synthesize using lateralObserverConfig, or pass an explicit sensitivity configuration with matching gains.');
        end
    end
end
