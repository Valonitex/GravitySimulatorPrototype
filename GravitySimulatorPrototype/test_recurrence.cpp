#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

int main() {
    int P = 8;
    double X = 2.0, Y = 1.0, eps = 0.1;
    double R2 = X*X + Y*Y + eps*eps;
    double R = std::sqrt(R2);
    
    std::vector<std::vector<double>> C(P+1, std::vector<double>(P+1, 0.0));
    
    C[0][0] = 1.0 / R;
    
    for (int q = 0; q <= P; ++q) {
        if (q > 0) {
            double term1 = Y * (2*q - 1) * C[0][q-1];
            double term2 = (q >= 2) ? (q - 1) * C[0][q-2] : 0.0;
            C[0][q] = (term1 - term2) / (q * R2);
        }
        
        for (int p = 1; p <= P - q; ++p) {
            double term1 = X * (2*p - 1) * C[p-1][q];
            double term2 = (q >= 1) ? 2 * Y * p * C[p][q-1] : 0.0;
            double term3 = (p >= 2) ? (p - 1) * C[p-2][q] : 0.0;
            double term4 = (q >= 2) ? p * C[p][q-2] : 0.0;
            
            C[p][q] = (term1 + term2 - term3 - term4) / (p * R2);
        }
    }
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "C[0][0] = " << C[0][0] << "\n";
    std::cout << "C[1][0] = " << C[1][0] << " (Expected: " << X / (R2*R) << ")\n";
    std::cout << "C[0][1] = " << C[0][1] << " (Expected: " << Y / (R2*R) << ")\n";
    
    double exp_C20 = (3*X*X - R2) / (2 * R2*R2*R);
    std::cout << "C[2][0] = " << C[2][0] << " (Expected: " << exp_C20 << ")\n";
    
    double exp_C11 = (3*X*Y) / (R2*R2*R);
    std::cout << "C[1][1] = " << C[1][1] << " (Expected: " << exp_C11 << ")\n";
    
    return 0;
}
