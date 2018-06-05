function  [A] = GenerateTransformationMatrices(theta, DH_params)
    a_dh = DH_params(1);
    d_dh = DH_params(3);
    arpha_dh = DH_params(2);    
    theta_dh = DH_params(4);
    carpha = cos(arpha_dh);
    sarpha = sin(arpha_dh);
    ctheta = cos(theta_dh + theta);
    stheta = sin(theta_dh + theta);
    
    A =    [ctheta   -stheta*carpha  stheta*sarpha    a_dh*ctheta;
            stheta   ctheta*carpha   -ctheta*sarpha   a_dh*stheta;
            0        sarpha          carpha           d_dh       ;      
            0        0               0                1          ];
    
end