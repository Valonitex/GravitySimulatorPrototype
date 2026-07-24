void compute_D(double dx, double dy, double D_out[NCOEFF]) {
    double r2 = dx*dx + dy*dy + eps*eps;
    double inv_r = 1.0 / std::sqrt(r2);
    double inv_r2 = inv_r * inv_r;
    double x = dx, y = dy;
    // Precompute powers of inv_r and x, y if helpful, or just let the compiler optimize
    double x0 = pow(x, 2);
    double x1 = pow(eps, 2) + x0 + pow(y, 2);
    double x2 = pow(x1, -1.0/2.0);
    double x3 = pow(x1, -3.0/2.0);
    double x4 = -x*x3;
    double x5 = pow(x1, -5.0/2.0);
    double x6 = 3*x0*x5 - x3;
    double x7 = pow(x, 3);
    double x8 = pow(x1, -7.0/2.0);
    double x9 = 9*x*x5 - 15*x7*x8;
    double x10 = pow(x, 4);
    double x11 = pow(x1, -9.0/2.0);
    double x12 = -90*x0*x8 + 105*x10*x11 + 9*x5;
    double x13 = 225*x8;
    double x14 = pow(x, 5);
    double x15 = pow(x1, -11.0/2.0);
    double x16 = -x*x13 + 1050*x11*x7 - 945*x14*x15;
    double x17 = pow(x, 6);
    double x18 = pow(x1, -13.0/2.0);
    double x19 = 4725*x0*x11 - 14175*x10*x15 - x13 + 10395*x17*x18;
    double x20 = pow(x1, -15.0/2.0);
    double x21 = -135135*pow(x, 7)*x20 + 11025*x*x11 + 218295*x14*x18 - 99225*x15*x7;
    D_out[0] = x2; // p=0, q=0
    D_out[1] = x2; // p=0, q=1
    D_out[2] = x4; // p=1, q=0
    D_out[3] = x2; // p=0, q=2
    D_out[4] = x4; // p=1, q=1
    D_out[5] = x6; // p=2, q=0
    D_out[6] = x2; // p=0, q=3
    D_out[7] = x4; // p=1, q=2
    D_out[8] = x6; // p=2, q=1
    D_out[9] = x9; // p=3, q=0
    D_out[10] = x2; // p=0, q=4
    D_out[11] = x4; // p=1, q=3
    D_out[12] = x6; // p=2, q=2
    D_out[13] = x9; // p=3, q=1
    D_out[14] = x12; // p=4, q=0
    D_out[15] = x2; // p=0, q=5
    D_out[16] = x4; // p=1, q=4
    D_out[17] = x6; // p=2, q=3
    D_out[18] = x9; // p=3, q=2
    D_out[19] = x12; // p=4, q=1
    D_out[20] = x16; // p=5, q=0
    D_out[21] = x2; // p=0, q=6
    D_out[22] = x4; // p=1, q=5
    D_out[23] = x6; // p=2, q=4
    D_out[24] = x9; // p=3, q=3
    D_out[25] = x12; // p=4, q=2
    D_out[26] = x16; // p=5, q=1
    D_out[27] = x19; // p=6, q=0
    D_out[28] = x2; // p=0, q=7
    D_out[29] = x4; // p=1, q=6
    D_out[30] = x6; // p=2, q=5
    D_out[31] = x9; // p=3, q=4
    D_out[32] = x12; // p=4, q=3
    D_out[33] = x16; // p=5, q=2
    D_out[34] = x19; // p=6, q=1
    D_out[35] = x21; // p=7, q=0
    D_out[36] = x2; // p=0, q=8
    D_out[37] = x4; // p=1, q=7
    D_out[38] = x6; // p=2, q=6
    D_out[39] = x9; // p=3, q=5
    D_out[40] = x12; // p=4, q=4
    D_out[41] = x16; // p=5, q=3
    D_out[42] = x19; // p=6, q=2
    D_out[43] = x21; // p=7, q=1
    D_out[44] = 2027025*pow(x, 8)/pow(x1, 17.0/2.0) - 396900*x0*x15 + 2182950*x10*x18 + 11025*x11 - 3783780*x17*x20; // p=8, q=0
}
