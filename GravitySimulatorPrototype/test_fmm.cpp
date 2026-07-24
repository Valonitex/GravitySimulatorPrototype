#include <iostream>
#include <vector>
#include <cmath>
#include <complex>

struct vectorP {
    double icap, jcap;
    vectorP(double i=0, double j=0) : icap(i), jcap(j) {}
    vectorP operator-(const vectorP& o) const { return vectorP(icap - o.icap, jcap - o.jcap); }
    vectorP operator+(const vectorP& o) const { return vectorP(icap + o.icap, jcap + o.jcap); }
    vectorP operator*(double s) const { return vectorP(icap * s, jcap * s); }
    vectorP& operator+=(const vectorP& o) { icap+=o.icap; jcap+=o.jcap; return *this; }
    double magSq() const { return icap*icap + jcap*jcap; }
};

namespace fmm {
    constexpr int P = 8;
    constexpr int NCOEFF = (P+1)*(P+2)/2;
    constexpr double G = 6.67430e-11;
    constexpr double eps = 0.1;

    inline int idx(int p, int q) {
        return p + q*(P+1) - q*(q-1)/2; // Linearized 2D index for p+q <= P
    }

    struct PascalTable {
        double C[P + 2][P + 2];
        PascalTable() {
            for (int i = 0; i <= P + 1; ++i) {
                for (int j = 0; j <= i; ++j) {
                    if (j == 0 || j == i) C[i][j] = 1.0;
                    else C[i][j] = C[i - 1][j - 1] + C[i - 1][j];
                }
            }
        }
    };
    static const PascalTable pascal;

    struct Node {
        double cx, cy;
        double half;
        int level, parent;
        int ch[4];
        bool is_leaf;
        std::vector<int> bodies;
        double M[NCOEFF] = {0};
        double L[NCOEFF] = {0};
        double com_vx = 0, com_vy = 0;
        
        Node() : cx(0), cy(0), half(0), level(0), parent(-1), is_leaf(true) {
            for(int i=0; i<4; ++i) ch[i] = -1;
        }
    };
    
    // Will complete this
}

int main() {
    std::cout << "Test script\n";
    return 0;
}
