#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <algorithm>

constexpr double G = 6.67430e-11;
constexpr double eps = 0.1;

class vectorP {
public:
    double icap, jcap;
    vectorP(double i=0, double j=0) : icap(i), jcap(j) {}
    vectorP operator+(const vectorP& o) const { return vectorP(icap + o.icap, jcap + o.jcap); }
    vectorP operator-(const vectorP& o) const { return vectorP(icap - o.icap, jcap - o.jcap); }
    vectorP operator*(double s) const { return vectorP(icap * s, jcap * s); }
    vectorP& operator+=(const vectorP& o) { icap+=o.icap; jcap+=o.jcap; return *this; }
    double magSq() const { return icap*icap + jcap*jcap; }
    double mag() const { return std::sqrt(magSq()); }
};

class Body {
public:
    bool dead = false;
    bool movability = true;
    vectorP m_posVec, m_velVec, m_accVec, m_jerkVec, m_forVec;
    double m_Mass, m_radius;
    Body(double m, vectorP pos, vectorP vel = vectorP(0,0)) : m_Mass(m), m_posVec(pos), m_velVec(vel) {}
};

namespace fmm {
    constexpr int P = 8;
    constexpr int NCOEFF = (P+1)*(P+2)/2;

    inline int idx(int p, int q) {
        return p + q*(P+1) - q*(q-1)/2;
    }

    struct BinomialTable {
        double C[P + 2][P + 2];
        BinomialTable() {
            for (int i = 0; i <= P + 1; ++i) {
                for (int j = 0; j <= i; ++j) {
                    if (j == 0 || j == i) C[i][j] = 1.0;
                    else C[i][j] = C[i - 1][j - 1] + C[i - 1][j];
                }
            }
        }
    };
    static const BinomialTable binom;

    struct Node {
        double cx, cy;
        double half;
        int level, parent;
        int ch[4];
        bool is_leaf;
        std::vector<int> bodies;
        double M[NCOEFF] = {0};
        double L[NCOEFF] = {0};
        double mass = 0;
        double com_vx = 0, com_vy = 0;
        
        Node() : cx(0), cy(0), half(0), level(0), parent(-1), is_leaf(true) {
            for(int i=0; i<4; ++i) ch[i] = -1;
        }
    };
    
    static std::vector<Node> tree;
    static int next_node = 0;

    static int alloc_node(double cx, double cy, double half, int level, int parent) {
        int idx = next_node++;
        if (idx >= (int)tree.size()) tree.resize(std::max((int)tree.size() * 2, 256));
        tree[idx] = Node();
        tree[idx].cx = cx; tree[idx].cy = cy;
        tree[idx].half = half; tree[idx].level = level; tree[idx].parent = parent;
        return idx;
    }

    inline bool is_neighbor(const Node& a, const Node& b) {
        double dx = std::abs(a.cx - b.cx);
        double dy = std::abs(a.cy - b.cy);
        double max_h = a.half + b.half + 1e-9;
        return (dx <= max_h) && (dy <= max_h);
    }

    static void build_sub(int node_idx, const std::vector<vectorP>& pos, int max_depth) {
        Node& nd = tree[node_idx];
        if ((int)nd.bodies.size() <= 16 || nd.level >= max_depth) {
            nd.is_leaf = true;
            return;
        }
        nd.is_leaf = false;
        double h2 = nd.half * 0.5;
        std::vector<int> quad_bodies[4];
        for (int b_idx : nd.bodies) {
            int q = (pos[b_idx].icap >= nd.cx ? 1 : 0) | (pos[b_idx].jcap >= nd.cy ? 2 : 0);
            quad_bodies[q].push_back(b_idx);
        }
        nd.bodies.clear();
        for (int q = 0; q < 4; ++q) {
            if (quad_bodies[q].empty()) continue;
            double child_cx = nd.cx + ((q & 1) ? h2 : -h2);
            double child_cy = nd.cy + ((q & 2) ? h2 : -h2);
            int child_idx = alloc_node(child_cx, child_cy, h2, nd.level + 1, node_idx);
            tree[child_idx].bodies = std::move(quad_bodies[q]);
            tree[node_idx].ch[q] = child_idx;
            build_sub(child_idx, pos, max_depth);
        }
    }

    static void p2m(int node_idx, const std::vector<vectorP>& pos, const std::vector<double>& masses, const std::vector<vectorP>* vel_ptr) {
        Node& nd = tree[node_idx];
        for (int b_idx : nd.bodies) {
            double dx = pos[b_idx].icap - nd.cx;
            double dy = pos[b_idx].jcap - nd.cy;
            double m = masses[b_idx];
            nd.mass += m;
            if (vel_ptr) {
                nd.com_vx += m * (*vel_ptr)[b_idx].icap;
                nd.com_vy += m * (*vel_ptr)[b_idx].jcap;
            }
            
            double xp[P+1], yp[P+1];
            xp[0] = 1.0; yp[0] = 1.0;
            for(int i=1; i<=P; ++i) { xp[i] = xp[i-1]*dx; yp[i] = yp[i-1]*dy; }
            
            for (int q = 0; q <= P; ++q) {
                for (int p = 0; p <= P - q; ++p) {
                    nd.M[idx(p,q)] += (G * m) * xp[p] * yp[q];
                }
            }
        }
        if (nd.mass > 0) {
            nd.com_vx /= nd.mass;
            nd.com_vy /= nd.mass;
        }
    }

    static void m2m(int parent_idx, int child_idx) {
        Node& p = tree[parent_idx];
        const Node& c = tree[child_idx];
        double dx = c.cx - p.cx;
        double dy = c.cy - p.cy;
        
        double xp[P+1], yp[P+1];
        xp[0] = 1.0; yp[0] = 1.0;
        for(int i=1; i<=P; ++i) { xp[i] = xp[i-1]*dx; yp[i] = yp[i-1]*dy; }

        p.mass += c.mass;
        p.com_vx += c.mass * c.com_vx;
        p.com_vy += c.mass * c.com_vy;
        
        for (int q = 0; q <= P; ++q) {
            for (int p_idx = 0; p_idx <= P - q; ++p_idx) {
                double sum = 0;
                for (int j = 0; j <= q; ++j) {
                    for (int i = 0; i <= p_idx; ++i) {
                        sum += binom.C[p_idx][i] * binom.C[q][j] * xp[p_idx-i] * yp[q-j] * c.M[idx(i,j)];
                    }
                }
                p.M[idx(p_idx,q)] += sum;
            }
        }
    }

    static void m2l(int target_idx, int source_idx) {
        Node& target = tree[target_idx];
        const Node& source = tree[source_idx];
        double dx = target.cx - source.cx;
        double dy = target.cy - source.cy;
        double r2 = dx*dx + dy*dy + eps*eps;
        
        double C[P+1][P+1] = {0};
        C[0][0] = 1.0 / std::sqrt(r2);
        
        for (int q = 0; q <= P; ++q) {
            if (q > 0) {
                double t1 = dy * (2*q - 1) * C[0][q-1];
                double t2 = (q >= 2) ? (q - 1) * C[0][q-2] : 0.0;
                C[0][q] = (t1 - t2) / (q * r2);
            }
            for (int p_idx = 1; p_idx <= P - q; ++p_idx) {
                double t1 = dx * (2*p_idx - 1) * C[p_idx-1][q];
                double t2 = (q >= 1) ? 2 * dy * p_idx * C[p_idx][q-1] : 0.0;
                double t3 = (p_idx >= 2) ? (p_idx - 1) * C[p_idx-2][q] : 0.0;
                double t4 = (q >= 2) ? p_idx * C[p_idx][q-2] : 0.0;
                C[p_idx][q] = (t1 + t2 - t3 - t4) / (p_idx * r2);
            }
        }
        
        for (int b = 0; b <= P; ++b) {
            for (int a = 0; a <= P - b; ++a) {
                double sum = 0;
                for (int v = 0; v <= P - a - b; ++v) {
                    for (int u = 0; u <= P - a - b - v; ++u) {
                        double sign = ((u+v)%2 == 1) ? -1.0 : 1.0;
                        sum += C[a+u][b+v] * binom.C[a+u][a] * binom.C[b+v][b] * sign * source.M[idx(u,v)];
                    }
                }
                target.L[idx(a,b)] += sum;
            }
        }
    }

    static void l2l(int parent_idx, int child_idx) {
        const Node& p = tree[parent_idx];
        Node& c = tree[child_idx];
        double dx = c.cx - p.cx;
        double dy = c.cy - p.cy;
        
        double xp[P+1], yp[P+1];
        xp[0] = 1.0; yp[0] = 1.0;
        for(int i=1; i<=P; ++i) { xp[i] = xp[i-1]*dx; yp[i] = yp[i-1]*dy; }

        for (int j = 0; j <= P; ++j) {
            for (int i = 0; i <= P - j; ++i) {
                double sum = 0;
                for (int b = j; b <= P - i; ++b) {
                    for (int a = i; a <= P - b; ++a) {
                        sum += binom.C[a][i] * binom.C[b][j] * xp[a-i] * yp[b-j] * p.L[idx(a,b)];
                    }
                }
                c.L[idx(i,j)] += sum;
            }
        }
    }

    static void l2p(int leaf_idx, double px, double py, double vx, double vy, vectorP& acc_out, vectorP* jerk_out) {
        const Node& nd = tree[leaf_idx];
        double dx = px - nd.cx;
        double dy = py - nd.cy;
        
        double xp[P+1], yp[P+1];
        xp[0] = 1.0; yp[0] = 1.0;
        for(int i=1; i<=P; ++i) { xp[i] = xp[i-1]*dx; yp[i] = yp[i-1]*dy; }

        double ax = 0, ay = 0;
        for (int b = 0; b <= P; ++b) {
            for (int a = 1; a <= P - b; ++a) {
                ax += a * nd.L[idx(a,b)] * xp[a-1] * yp[b];
            }
        }
        for (int b = 1; b <= P; ++b) {
            for (int a = 0; a <= P - b; ++a) {
                ay += b * nd.L[idx(a,b)] * xp[a] * yp[b-1];
            }
        }
        
        acc_out.icap += ax;
        acc_out.jcap += ay;

        if (jerk_out) {
            double jx = 0, jy = 0;
            double vrel_x = vx;
            double vrel_y = vy;
            
            double dax_dx = 0, dax_dy = 0, day_dy = 0;
            for (int b = 0; b <= P; ++b) {
                for (int a = 2; a <= P - b; ++a) {
                    dax_dx += a * (a-1) * nd.L[idx(a,b)] * xp[a-2] * yp[b];
                }
            }
            for (int b = 1; b <= P; ++b) {
                for (int a = 1; a <= P - b; ++a) {
                    double term = a * b * nd.L[idx(a,b)] * xp[a-1] * yp[b-1];
                    dax_dy += term;
                }
            }
            for (int b = 2; b <= P; ++b) {
                for (int a = 0; a <= P - b; ++a) {
                    day_dy += b * (b-1) * nd.L[idx(a,b)] * xp[a] * yp[b-2];
                }
            }
            
            jx = dax_dx * vrel_x + dax_dy * vrel_y;
            jy = dax_dy * vrel_x + day_dy * vrel_y;
            
            jerk_out->icap += jx;
            jerk_out->jcap += jy;
        }
    }

    static void run_fmm(const std::vector<vectorP>& pos,
                        const std::vector<double>& masses,
                        const std::vector<vectorP>* vel_ptr,
                        std::vector<vectorP>& acc_out,
                        std::vector<vectorP>* jerk_out,
                        const std::vector<std::unique_ptr<Body>>& bodies)
    {
        const int n = (int)pos.size();
        for (int i = 0; i < n; ++i) {
            acc_out[i] = vectorP(0, 0);
            if (jerk_out) (*jerk_out)[i] = vectorP(0, 0);
        }
        if (n < 2) return;

        double xmin = pos[0].icap, xmax = pos[0].icap;
        double ymin = pos[0].jcap, ymax = pos[0].jcap;
        for (int i = 1; i < n; ++i) {
            xmin = std::min(xmin, pos[i].icap); xmax = std::max(xmax, pos[i].icap);
            ymin = std::min(ymin, pos[i].jcap); ymax = std::max(ymax, pos[i].jcap);
        }
        double cx = (xmin + xmax) * 0.5;
        double cy = (ymin + ymax) * 0.5;
        double half = std::max({ (xmax - xmin) * 0.5, (ymax - ymin) * 0.5, 1e-6 }) * 1.001;

        next_node = 0;
        int max_depth = std::min(8, (int)std::ceil(std::log2(n) / 2.0));
        int root = alloc_node(cx, cy, half, 0, -1);

        tree[root].bodies.resize(n);
        for (int i = 0; i < n; ++i) tree[root].bodies[i] = i;

        build_sub(root, pos, max_depth);

        std::vector<std::vector<int>> levels(max_depth + 1);
        std::vector<int> leaves;
        for (int i = 0; i < next_node; ++i) {
            levels[tree[i].level].push_back(i);
            if (tree[i].is_leaf) leaves.push_back(i);
        }

        for (int leaf_idx : leaves) p2m(leaf_idx, pos, masses, vel_ptr);

        for (int l = max_depth - 1; l >= 0; --l) {
            for (int idx : levels[l]) {
                if (!tree[idx].is_leaf) {
                    for (int q = 0; q < 4; ++q) {
                        int ch = tree[idx].ch[q];
                        if (ch != -1) m2m(idx, ch);
                    }
                    if (tree[idx].mass > 0) {
                        tree[idx].com_vx /= tree[idx].mass;
                        tree[idx].com_vy /= tree[idx].mass;
                    }
                }
            }
        }

        for (int l = 2; l <= max_depth; ++l) {
            for (int u_idx : levels[l]) {
                int parent_u = tree[u_idx].parent;
                for (int p_neg : levels[l - 1]) {
                    if (is_neighbor(tree[parent_u], tree[p_neg])) {
                        if (tree[p_neg].is_leaf) {
                            if (!is_neighbor(tree[u_idx], tree[p_neg])) m2l(u_idx, p_neg);
                        } else {
                            for (int q = 0; q < 4; ++q) {
                                int ch = tree[p_neg].ch[q];
                                if (ch != -1 && !is_neighbor(tree[u_idx], tree[ch])) {
                                    m2l(u_idx, ch);
                                }
                            }
                        }
                    }
                }
            }
        }

        for (int l = 0; l < max_depth; ++l) {
            for (int idx : levels[l]) {
                if (!tree[idx].is_leaf) {
                    for (int q = 0; q < 4; ++q) {
                        int ch = tree[idx].ch[q];
                        if (ch != -1) l2l(idx, ch);
                    }
                }
            }
        }

        for (int u_leaf : leaves) {
            Node& u_nd = tree[u_leaf];
            for (int bi : u_nd.bodies) {
                if (!bodies[bi]->movability) continue;
                double vx = vel_ptr ? (*vel_ptr)[bi].icap : 0.0;
                double vy = vel_ptr ? (*vel_ptr)[bi].jcap : 0.0;
                vectorP* j_ptr = jerk_out ? &((*jerk_out)[bi]) : nullptr;
                l2p(u_leaf, pos[bi].icap, pos[bi].jcap, vx, vy, acc_out[bi], j_ptr);
            }

            for (int v_leaf : leaves) {
                if (!is_neighbor(u_nd, tree[v_leaf])) continue;
                for (int i : u_nd.bodies) {
                    if (!bodies[i]->movability) continue;
                    for (int j : tree[v_leaf].bodies) {
                        if (i == j) continue;

                        vectorP r   = pos[j] - pos[i];
                        double  r2  = r.magSq() + eps*eps;
                        double  inv_r2 = 1.0 / r2;
                        double  inv_r = std::sqrt(inv_r2);
                        double  inv_r3 = inv_r2 * inv_r;

                        acc_out[i] += r * (G * masses[j] * inv_r3);

                        if (jerk_out && vel_ptr) {
                            vectorP v       = (*vel_ptr)[j] - (*vel_ptr)[i];
                            double  v_dot_r = v.icap * r.icap + v.jcap * r.jcap;
                            double  inv_r5  = inv_r3 * inv_r2;
                            (*jerk_out)[i] += (v * inv_r3 - r * (3.0 * v_dot_r * inv_r5)) * (G * masses[j]);
                        }
                    }
                }
            }
        }
    }
}

int main() {
    std::cout << "Running test..." << std::endl;
    // Set up a simple 2-body test
    std::vector<std::unique_ptr<Body>> bodies;
    bodies.push_back(std::make_unique<Body>(1000000000000.0, vectorP(10.0, 0.0), vectorP(0.0, 1.29173)));
    bodies.push_back(std::make_unique<Body>(1000000000000.0, vectorP(-10.0, 0.0), vectorP(0.0, -1.29173)));

    std::vector<vectorP> pos(2), vel(2), acc(2), acc_fmm(2), jerk(2), jerk_fmm(2);
    std::vector<double> masses(2);
    for (int i = 0; i < 2; ++i) {
        pos[i] = bodies[i]->m_posVec;
        vel[i] = bodies[i]->m_velVec;
        masses[i] = bodies[i]->m_Mass;
    }

    // Set max_depth manually or let it be 1 (so no M2L, only P2P)
    // Wait, to test FMM, we need enough bodies to trigger M2L!
    // So let's make 20 bodies.
    std::vector<std::unique_ptr<Body>> bodies20;
    for (int i = 0; i < 20; ++i) {
        bodies20.push_back(std::make_unique<Body>(
            1e12, 
            vectorP(i * 5.0, (i%3)*2.0), 
            vectorP((i%2)*0.5, -(i%4)*0.1)
        ));
    }
    
    std::vector<vectorP> pos20(20), vel20(20), acc20(20), acc_fmm20(20), jerk20(20), jerk_fmm20(20);
    std::vector<double> masses20(20);
    for (int i = 0; i < 20; ++i) {
        pos20[i] = bodies20[i]->m_posVec;
        vel20[i] = bodies20[i]->m_velVec;
        masses20[i] = bodies20[i]->m_Mass;
    }

    fmm::run_fmm(pos20, masses20, &vel20, acc_fmm20, &jerk_fmm20, bodies20);

    // Exact O(N^2) for comparison
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            if (i == j) continue;
            vectorP r = pos20[j] - pos20[i];
            double r2 = r.magSq() + eps*eps;
            double inv_r = std::sqrt(1.0/r2);
            double inv_r3 = inv_r * inv_r * inv_r;
            acc20[i] += r * (G * masses20[j] * inv_r3);
            
            vectorP v = vel20[j] - vel20[i];
            double v_dot_r = v.icap * r.icap + v.jcap * r.jcap;
            double inv_r5 = inv_r3 * (1.0/r2);
            jerk20[i] += (v * inv_r3 - r * (3.0 * v_dot_r * inv_r5)) * (G * masses20[j]);
        }
    }

    for (int i = 0; i < 2; ++i) { // Print first 2
        std::cout << "Body " << i << ":\n";
        std::cout << "  Acc Exact: " << acc20[i].icap << ", " << acc20[i].jcap << "\n";
        std::cout << "  Acc FMM:   " << acc_fmm20[i].icap << ", " << acc_fmm20[i].jcap << "\n";
        std::cout << "  Jerk Exact: " << jerk20[i].icap << ", " << jerk20[i].jcap << "\n";
        std::cout << "  Jerk FMM:   " << jerk_fmm20[i].icap << ", " << jerk_fmm20[i].jcap << "\n";
    }

    return 0;
}
