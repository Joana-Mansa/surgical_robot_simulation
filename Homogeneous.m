function T = Homogeneous(DH_row)

% T = Homogeneous_dh(DH_i)
% takes  and loops over an input of a row on the DH table
% outputs an HTM
    a = DH_row(1);
    alpha = DH_row(2);
    d = DH_row(3);
    theta = DH_row(4);

    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    T = [ct -st*ca st*sa a*ct;
        st ct*ca -ct*sa a*st;
        0 sa ca d;
        0 0 0 1];
end
