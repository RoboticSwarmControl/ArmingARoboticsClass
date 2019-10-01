function result = Htrans(DH)%DH = (r,alpha,d,theta)
    %MAKE TRANSFORMATION MATRIX 
    R_z_theta   = makehgtform('zrotate',DH(4));
    T_z_d       = makehgtform('translate',0,0,DH(3));
    T_x_r       = makehgtform('translate',DH(1),0,0);
    R_x_alpha   = makehgtform('xrotate',DH(2));    
    result = R_z_theta * T_z_d * T_x_r * R_x_alpha;
end
