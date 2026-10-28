classdef coupledMotionSupport
% Joint polynomial enclosure of vehicle dynamics and invariant-output Grams.
% Course coordinates use v=[s;0], a=[b;s*q]. The same s,q,b occur in
% dynamics and every output. Tensor Bernstein controls certify whole boxes.
    methods (Static)
        function [controls,boxes]=controls(p,d)
            validateattributes(d.minimumCourseSpeed,{'double'}, ...
                {'real','finite','positive','scalar','<=',p.maximumSpeed});
            validateattributes([p.maximumAcceleration,p.maximumCourseRate],{'double'}, ...
                {'real','finite','nonnegative'});
            validateattributes(d.speedPartitions,{'double'},{'integer','positive','scalar'});
            validateattributes(d.coursePartitions,{'double'},{'integer','positive','scalar'});
            validateattributes(d.accelerationPartitions,{'double'},{'integer','positive','scalar'});
            assert(mod(d.coursePartitions,2)==0 && mod(d.accelerationPartitions,2)==0, ...
                'VehicleLocalization:InvalidRedundancyDomain','Signed partitions must include zero.');
            % Relative resolution matters for the low-speed squared sensitivities.
            speed=exp(linspace(log(d.minimumCourseSpeed),log(p.maximumSpeed),d.speedPartitions+1));
            speed([1,end])=[d.minimumCourseSpeed,p.maximumSpeed];
            count=d.speedPartitions*d.coursePartitions*d.accelerationPartitions;
            controls=zeros(27*count,8);boxes=zeros(count,6);index=0;
            for i=1:d.speedPartitions
                % Every feasible point in this speed slab satisfies this q bound.
                qmax=min(p.maximumCourseRate,p.maximumAcceleration/speed(i));
                course=linspace(-qmax,qmax,d.coursePartitions+1);
                for j=1:d.coursePartitions
                    qmin=min(abs(course(j:j+1)));
                    if course(j)<=0 && course(j+1)>=0,qmin=0;end
                    % b^2+s^2*q^2<=A^2. This outer box retains every feasible
                    % point; it does not assert that every box corner is physical.
                    bmax=sqrt(max(0,p.maximumAcceleration^2-(speed(i)*qmin)^2));
                    acceleration=linspace(-bmax,bmax,d.accelerationPartitions+1);
                    for k=1:d.accelerationPartitions
                        box=[speed(i:i+1),course(j:j+1),acceleration(k:k+1)];
                        index=index+1;boxes(index,:)=box;
                        controls(27*index-26:27*index,:)=coupledMotionSupport.boxControls(box);
                    end
                end
            end
        end

        function c=boxControls(box)
        % Exact degree-(2,2,2) coefficients, not values sampled as vertices.
        % For z in [l,r], z has controls [l,(l+r)/2,r] and z^2 has
        % controls [l^2,l*r,r^2]. Tensor products give mixed monomials.
            s=[box(1),mean(box(1:2)),box(2)];s2=[box(1)^2,prod(box(1:2)),box(2)^2];
            q=[box(3),mean(box(3:4)),box(4)];q2=[box(3)^2,prod(box(3:4)),box(4)^2];
            b=[box(5),mean(box(5:6)),box(6)];b2=[box(5)^2,prod(box(5:6)),box(6)^2];
            c=zeros(27,8);n=0;
            for i=1:3
                for j=1:3
                    for k=1:3
                        n=n+1;c(n,:)=[q(j),q2(j),b(k),s(i),s2(i), ...
                            b2(k)+s2(i)*q2(j),s(i)*b(k),s2(i)*q(j)];
                    end
                end
            end
        end

        function f=features(s,q,b)
            f=[q,q^2,b,s,s^2,b^2+s^2*q^2,s*b,s^2*q];
        end

        function [A,speed,pair,direction]=matrices(p,f)
        % At a physical point, pair is EXACTLY H_dot'*H_dot+H_cross'*H_cross.
        % A Bernstein control need not itself be a PSD Gram or a physical state.
            t=p.theta;V=p.maximumSpeed;ell=p.headingLengthM;
            E=eye(8);e2=E(:,2);e3=E(:,3);e5=E(:,5);e6=E(:,6);e7=E(:,7);
            A=zeros(8);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;A(7,8)=1;
            A([3,6],[2,5])=f(2)/t^2*eye(2);
            A([3,6],[3,6])=2*f(1)/t*[0,-1;1,0];
            A(6,8)=2*f(3)/(t^2*ell);A(8,8)=-p.gyroGainNormalized;
            speed=f(5)/V^2*(e2*e2.');
            T=e2*e3.'-e5*e6.';N=e2*e6.'+e5*e3.';
            pair=f(6)/(t*V)^2*(e2*e2.'+e5*e5.') ...
                +f(5)/V^2*(e3*e3.'+e6*e6.') ...
                +f(7)/(t*V^2)*(T+T.')+f(8)/(t*V^2)*(N+N.');
            direction=e5*e5.'-f(4)/(t*ell)*(e5*e7.'+e7*e5.') ...
                +f(5)/(t*ell)^2*(e7*e7.');
        end
    end
end
