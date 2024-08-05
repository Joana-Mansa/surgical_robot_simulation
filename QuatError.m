function eo = QuatError(ed, e)
% computes the error quaternion between a desired quaternion
% and the actual quaternion
% error quaternion
% eo = QuatError(ed, e)
% 
eo = e(4)*ed(1:3) - ed(4)*e(1:3) - cross(ed(1:3),e(1:3));

eta_tilde = e(4)*ed(4) + e(1:3)'*ed(1:3);
if eta_tilde < 0
    eo = - eo;
end

end
