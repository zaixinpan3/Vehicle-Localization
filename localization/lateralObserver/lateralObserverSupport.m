classdef lateralObserverSupport
% lateralObserverSupport Bicycle model and gain scheduling of the lateral observer.
% Static methods build the LPV bicycle matrices, evaluate the model, map speed
% to scheduling coordinates and triangle vertices, blend vertex gains and
% verify the vehicle parameters of a stored design.
% Example: K = lateralObserverSupport.scheduleLateralObserverGain(design,speed).

    methods (Static)
        function model = lateralBicycleModel(vehicle)
        % lateralBicycleModel: Build the 2-DOF lateral bicycle model of the ego
        % vehicle in the affine-in-scheduling form used by the LPV observer. With
        % the state x = [vy; r], the input u = delta, and the output y = [ay; r],
        % linear tire forces give
        %
        %   vydot = -(Cf+Cr)/(m Vx) vy + ((lr Cr - lf Cf)/(m Vx) - Vx) r + Cf/m delta
        %   rdot  = (lr Cr - lf Cf)/(Iz Vx) vy - (lf^2 Cf + lr^2 Cr)/(Iz Vx) r + lf Cf/Iz delta
        %   ay    = -(Cf+Cr)/(m Vx) vy + (lr Cr - lf Cf)/(m Vx) r + Cf/m delta
        %
        % Every speed-dependent entry is affine in the scheduling parameter
        % rho = [Vx; 1/Vx], so A(rho) = A0 + rho(1) A1 + rho(2) A2 and
        % C(rho) = C0 + rho(2) C2 with constant B and D. That affine structure is
        % what makes the polytopic representation exact.
        %
        % Input:
        %   vehicle: struct with mass, yawInertia, lf, lr, frontCorneringStiffness,
        %       and rearCorneringStiffness
        %
        % Output:
        %   model: struct with the affine basis matrices A0, A1, A2, C0, C2, the
        %       constant B and D, the axle-force mismatch channel E and F, and the
        %       source vehicle parameters
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
            model.outputNames = ["lateralAcceleration", "yawRate"];

            % A(rho) = A0 + Vx A1 + (1/Vx) A2
            model.A0 = zeros(2, 2);
            model.A1 = [0.0, -1.0; 0.0, 0.0];
            model.A2 = [-stiffnessSum ./ mass, stiffnessMoment ./ mass; ...
                stiffnessMoment ./ yawInertia, -stiffnessSecondMoment ./ yawInertia];

            % C(rho) = C0 + (1/Vx) C2; the yaw rate is measured directly
            model.C0 = [0.0, 0.0; 0.0, 1.0];
            model.C2 = [-stiffnessSum ./ mass, stiffnessMoment ./ mass; 0.0, 0.0];

            model.B = [frontStiffness ./ mass; (lf .* frontStiffness) ./ yawInertia];
            model.D = [frontStiffness ./ mass; 0.0];

            % Axle-force mismatch channel. With w = [dFyf; dFyr] / m the error of the
            % modeled front and rear axle lateral forces per unit vehicle mass, the
            % true vehicle obeys xdot = A x + B delta + E w and y = C x + D delta +
            % F w exactly, whatever makes the tire forces differ from the model
            model.E = [1.0, 1.0; (mass .* lf) ./ yawInertia, -(mass .* lr) ./ yawInertia];
            model.F = [1.0, 1.0; 0.0, 0.0];
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
        %   C: [2 x 2] output matrix at rho
            rho = double(rho(:));
            assert(numel(rho) == 2 && all(isfinite(rho)), "rho must be a finite [2 x 1] scheduling point.");
            A = model.A0 + (rho(1) .* model.A1) + (rho(2) .* model.A2);
            C = model.C0 + (rho(2) .* model.C2);
        end

        function [alpha, alphaRate] = schedulingCoordinates(polytope, longitudinalSpeed, longitudinalAcceleration)
        % schedulingCoordinates: Express one scheduling point in the barycentric
        % coordinates of the polytope, together with their time derivative. Solving
        % S alpha = [rho; 1] gives coordinates that sum to one and reproduce rho,
        % so any matrix affine in rho, including the observer gain, equals the same
        % convex combination of its vertex values. Differentiating along the speed
        % trajectory with rhodot = [ax; -ax/Vx^2] gives S alphaRate = [rhodot; 0],
        % which is the Pdot term of the parameter-dependent Lyapunov function.
        %
        % Input:
        %   polytope: struct from lateralObserverSupport.buildSchedulingPolytope
        %   longitudinalSpeed: scalar Vx in meters per second, strictly positive
        %   longitudinalAcceleration: optional scalar ax in meters per second
        %       squared; defaults to zero when only alpha is needed
        %
        % Output:
        %   alpha: [3 x 1] barycentric coordinates of rho(Vx) = [Vx; 1/Vx]
        %   alphaRate: [3 x 1] time derivative of alpha at the given acceleration
            if nargin < 3
                longitudinalAcceleration = 0.0;
            end
            longitudinalSpeed = double(longitudinalSpeed);
            longitudinalAcceleration = double(longitudinalAcceleration);
            assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed) && longitudinalSpeed > 0, ...
                "longitudinalSpeed must be a positive finite scalar.");
            assert(isscalar(longitudinalAcceleration) && isfinite(longitudinalAcceleration), ...
                "longitudinalAcceleration must be a finite scalar.");

            alpha = polytope.S \ [longitudinalSpeed; 1.0 ./ longitudinalSpeed; 1.0];
            alphaRate = polytope.S \ (longitudinalAcceleration .* [1.0; -1.0 ./ (longitudinalSpeed.^2); 0.0]);
        end

        function polytope = buildSchedulingPolytope(speedRange)
        % buildSchedulingPolytope: Build the triangle that covers the scheduling
        % curve rho(Vx) = [Vx; 1/Vx] over Vx in [Vmin, Vmax]. Because 1/Vx is
        % convex in Vx, the curve lies below the chord joining its endpoints, so a
        % triangle whose first two vertices are those endpoints and whose third
        % vertex lies below the curve contains the whole arc:
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

        function [L, P] = scheduleLateralObserverGain(design, longitudinalSpeed)
        % scheduleLateralObserverGain: Evaluate the observer gain of one operating
        % point as the barycentric blend of the three vertex gains. The gain and
        % the Lyapunov matrix are affine in the barycentric coordinates alpha of
        % rho = [Vx; 1/Vx] on the scheduling triangle, so L(rho) = sum_i alpha_i L_i
        % and P(rho) = sum_i alpha_i P_i with the same alpha that reproduces rho. A
        % speed outside the designed range is clamped to the nearer end, where
        % alpha is a unit vector and the gain is that vertex gain itself.
        %
        % Input:
        %   design: struct with polytope, vertexGains, and lyapunovBasis from
        %       designLateralObserverGains
        %   longitudinalSpeed: scalar Vx in meters per second
        %
        % Output:
        %   L: [2 x 2] observer gain at the operating point
        %   P: [2 x 2] Lyapunov matrix of the certificate at the same point
            longitudinalSpeed = double(longitudinalSpeed);
            assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed), ...
                "The scheduling speed must be a finite scalar.");
            speedRange = design.polytope.speedRange;
            clampedSpeed = min(max(longitudinalSpeed, speedRange(1)), speedRange(2));
            alpha = lateralObserverSupport.schedulingCoordinates(design.polytope, clampedSpeed);

            L = (alpha(1) .* design.vertexGains(:, :, 1)) + (alpha(2) .* design.vertexGains(:, :, 2)) + ...
                (alpha(3) .* design.vertexGains(:, :, 3));
            P = (alpha(1) .* design.lyapunovBasis(:, :, 1)) + (alpha(2) .* design.lyapunovBasis(:, :, 2)) + ...
                (alpha(3) .* design.lyapunovBasis(:, :, 3));
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
