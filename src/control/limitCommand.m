function cmd = limitCommand(cmd, prev, cfg)
%LIMITCOMMAND Enforce speed and acceleration limits on a body command [v w].
    dt = cfg.dt; R = cfg.robot;
    v = min(max(cmd(1), R.vMin), R.vMax);
    w = min(max(cmd(2), -R.wMax), R.wMax);
    v = min(max(v, prev(1) - R.aMax * dt), prev(1) + R.aMax * dt);
    w = min(max(w, prev(2) - R.alphaMax * dt), prev(2) + R.alphaMax * dt);
    cmd = [v w];
end
