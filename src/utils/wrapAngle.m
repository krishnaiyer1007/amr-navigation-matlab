function a = wrapAngle(a)
%WRAPANGLE Wrap angle(s) to [-pi, pi). Core MATLAB replacement for wrapToPi.
    a = mod(a + pi, 2*pi) - pi;
end
