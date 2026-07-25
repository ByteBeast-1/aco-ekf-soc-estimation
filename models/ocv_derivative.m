function dOCV = ocv_derivative(SoC)
% OCV_DERIVATIVE  d(OCV)/d(SoC), the slope of the OCV curve.
%
%   dOCV = ocv_derivative(SoC)
%
%   Derivative of Eq. (5): OCV = 0.5214*SoC^3 - 0.9611*SoC^2 + 0.66*SoC + 3.13
%   Not needed for Week 1's plant simulation, but required later for the
%   EKF's linearized measurement matrix C = [dOCV/dSoC - 1, -1].

    SoC = min(max(SoC, 0), 1);
    dOCV = 3*0.5214*SoC.^2 - 2*0.9611*SoC + 0.66;
end
