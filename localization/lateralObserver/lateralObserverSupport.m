classdef lateralObserverSupport
% lateralObserverSupport Bicycle model and gain scheduling of the lateral observer.
% Static methods build the LPV bicycle matrices, evaluate the model, map speed
% to scheduling coordinates and triangle vertices, blend vertex gains and
% verify the vehicle parameters of a stored design.
% Example: K = lateralObserverSupport.scheduleLateralObserverGain(design,speed).

    methods (Static)
        function model = lateralBicycleModel(vehicle)
        % lateralBicycleModel Build the two-state bicycle model with yaw output.
        % State x = [vy; r], steering input delta and measured output y = r.
        % A(rho) = A0 + Vx*A1 + (1/Vx)*A2, with constant C = [0,1].
        % Axle-force mismatch w = [dFyf;dFyr]/m enters only the state
        % equation through E. No lateral accelerometer is used.
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
            model.outputNames = "yawRate";

            % A(rho) = A0 + Vx A1 + (1/Vx) A2
            model.A0 = zeros(2, 2);
            model.A1 = [0.0, -1.0; 0.0, 0.0];
            model.A2 = [-stiffnessSum ./ mass, stiffnessMoment ./ mass; ...
                stiffnessMoment ./ yawInertia, -stiffnessSecondMoment ./ yawInertia];

            model.C = [0.0, 1.0];
            model.B = [frontStiffness ./ mass; (lf .* frontStiffness) ./ yawInertia];
            model.E = [1.0, 1.0; (mass .* lf) ./ yawInertia, -(mass .* lr) ./ yawInertia];
        end

        function [A, C] = evaluateLateralModel(model, rho)
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
        %   C: [1 x 2] constant yaw-rate output matrix
            rho = double(rho(:));
            assert(numel(rho) == 2 && all(isfinite(rho)), "rho must be a finite [2 x 1] scheduling point.");
            A = model.A0 + (rho(1) .* model.A1) + (rho(2) .* model.A2);
            C = model.C;
        end

        function alpha = schedulingCoordinates(polytope, longitudinalSpeed)
        % schedulingCoordinates Barycentric coordinates of [Vx;1/Vx].
        % A common Lyapunov matrix requires no scheduling derivative.
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

        function domain = buildCertificateDomain(model, polytope)
        % buildCertificateDomain Three speed-position vertices; no rate hull.
            domain = struct('numVertices', 3, 'vertexA', zeros(2, 2, 3));
            for idx = 1:3
                domain.vertexA(:, :, idx) = ...
                    lateralObserverSupport.evaluateLateralModel(model, polytope.vertices(:, idx));
            end
        end

        function [L, P] = scheduleLateralObserverGain(design, longitudinalSpeed)
        % scheduleLateralObserverGain Blend the three 2-by-1 gain columns.
        % P is the common Lyapunov matrix at every speed. Gains clamp at
        % the endpoints; the runtime withdraws aiding outside the interval.
            longitudinalSpeed = double(longitudinalSpeed);
            assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed), ...
                "The scheduling speed must be a finite scalar.");
            speedRange = design.polytope.speedRange;
            clampedSpeed = min(max(longitudinalSpeed, speedRange(1)), speedRange(2));
            alpha = lateralObserverSupport.schedulingCoordinates(design.polytope, clampedSpeed);

            L = (alpha(1) .* design.vertexGains(:, :, 1)) + (alpha(2) .* design.vertexGains(:, :, 2)) + ...
                (alpha(3) .* design.vertexGains(:, :, 3));
            P = design.lyapunovMatrix;
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
