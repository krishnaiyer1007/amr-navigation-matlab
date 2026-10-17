function [u, st] = wheelPID(target, actual, st, dt, cfg)
%WHEELPID Discrete PI(D) wheel-speed controller with feedforward and anti-windup.
%   u = target + Kp e + Ki integral(e) + Kd de/dt, per wheel.
%   The integral term is clamped to +/- iMax and frozen while the output
%   saturates (conditional integration) to prevent windup.
    g = cfg.wheelPID;
    e = target - actual;
    integCand = st.integ + e * dt;
    integCand = min(max(integCand, -g.iMax / max(g.Ki, eps)), g.iMax / max(g.Ki, eps));
    d = (e - st.prevErr) / dt;
    uCand = target + g.Kp * e + g.Ki * integCand + g.Kd * d;
    uMax = cfg.robot.wheelSpeedMax;
    sat = abs(uCand) > uMax;
    integ = integCand;
    integ(sat) = st.integ(sat);                 % conditional integration
    u = target + g.Kp * e + g.Ki * integ + g.Kd * d;
    u = min(max(u, -uMax), uMax);
    st.integ = integ;
    st.prevErr = e;
end
