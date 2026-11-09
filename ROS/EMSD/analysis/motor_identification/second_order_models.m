% Exploratory second-order current-to-output models from motor_data.xlsx.
% Requires MATLAB Control System Toolbox (or Octave: pkg load control).
% Input: current deviations [A]. Outputs: thrust deviations [N], torque deviations [N*m].
% Fast poles are unresolved by the ~1.35 s sampling; these are not validated motor dynamics.
% For absolute output add the fitted offset to the response to absolute current;
% reproduce the fitted record using equilibrium initial state at the first current.

G_thrust = tf([22156.787879081035], [1 20.189264411569066 38080.121872305477]);
thrust_offset = 1.9523346378761559;
% wn = 195.14129 rad/s; zeta = 0.051729864; R^2 = 0.96181516

G_torque = tf([329.66803774478126], [1 20.106223878617545 38302.537265702136]);
torque_offset = 0.026381252776717403;
% wn = 195.71034 rad/s; zeta = 0.051367301; R^2 = 0.96097034
