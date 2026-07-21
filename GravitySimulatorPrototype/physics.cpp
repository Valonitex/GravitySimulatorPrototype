//hi

#include <filesystem>

#include "EndBrace.h"

double dt = (1.0f / 120.0f);
constexpr double G = 6.67430e-11;

class Body;
class vectorP;
void eos(double& KE , double& PE , double& E , std::vector<std::unique_ptr<Body>>& bodys);
void linearP(vectorP& lP , std::vector<std::unique_ptr<Body>>& bodys);
void angularP(double& aP , std::vector<std::unique_ptr<Body>>& bodys);

void drawGrid(const std::vector<std::vector<char>> livyud)
{
	for (int i = 0; i < livyud.size(); i++)
	{
		for (int j = 0; j < livyud[i].size(); j++)
		{
			std::cout << livyud[i][j] << " ";
		}
		LOG("");
	};
}

class vectorP
{
public:
	double icap;
	double jcap;
	//double inclineT;
	//double inclineC;

public:
	vectorP(double i=0, double j=0)
		:icap(i), jcap(j)
	{
		//inclineT = (icap != 0) ? (jcap / icap) : 0;
		//inclineC = (icap != 0) ? (icap / mag) : 0;

	}
	void getInfo()
	{
		LOG(icap << "\n" << jcap << "\n");
	}
	void updateValues()
	{
		//inclineT = (icap != 0) ? (jcap / icap) : 0;
		//inclineC = (icap != 0) ? (icap / mag) : 0;
	}
	double magSq() const
	{
		return icap*icap + jcap*jcap;
	}
	double mag() const
	{
		return sqrt(magSq());
	}

	vectorP operator+=(const vectorP& other)
	{

		icap += other.icap;
		jcap += other.jcap;
		//updateValues();

		return (*this);
	}

	vectorP operator-=(const vectorP& other)
	{
		icap -= other.icap;
		jcap -= other.jcap;
		//updateValues();

		return (*this);
	}

	vectorP operator+(const vectorP& other)
	{
		return (vectorP(icap + other.icap, jcap + other.jcap));
	}

	vectorP operator-(const vectorP& other) const //const coz matrix mul told me 
	{
		return (vectorP(icap - other.icap, jcap - other.jcap));
	}

	vectorP operator*(double t)
	{
		return (vectorP(icap * t, jcap * t));
	}

	vectorP operator/(double t)
	{
		return (vectorP(icap / t, jcap / t));
	}

	vectorP operator/=(double t)
	{
		icap /= t;
		jcap /= t;
		//updateValues();

		return (*this);
	}

	vectorP& operator=(const vectorP& other)
	{
		icap = other.icap;
		jcap = other.jcap;
		//updateValues();

		return *this;
	}

	double cross(const vectorP& other)
	{
		return (icap * other.jcap - jcap * other.icap);
	}

	bool operator!=(const vectorP& other)
	{
		if (icap != other.icap || jcap != other.jcap)
		{
			return true;
		}
		return false;
	}
	bool operator==(const vectorP& other)
	{
		if (icap == other.icap && jcap == other.jcap)
		{
			return true;
		}
		return false;
	}

	vectorP negate()
	{
		return vectorP(-(icap), -(jcap));
	}

	vectorP round()
	{
		return vectorP(std::round(icap), std::round(jcap));
	}

	void negate_inp()
	{
		icap *= -1;
		jcap *= -1;
	}
};

std::ostream& operator<<(std::ostream& stream, const vectorP& other)
{
	stream << other.icap << "," << other.jcap ;
	return stream;
}

class Body
{
public:
	bool dead;
	bool movability;
	vectorP m_posVec;
	vectorP m_velVec;
	vectorP m_accVec;
	vectorP m_forVec;
	vectorP m_forRes;
	vectorP m_jerkVec;
	double m_radius;
	double m_Mass;
	int clusterIndex = 0;

public:
	Body(double m, double r, bool stat = true, vectorP pos = { 0,0 }, vectorP vel = { 0,0 }, vectorP f = { 0,0 }) //stat = static
		:dead(false),movability(stat), m_posVec(pos), m_velVec(vel), m_forVec(f), m_accVec(0.0f, 0.0f)
	{
		if (m <= 0 || r<=0)
			throw std::invalid_argument("Mass/Radius be positive");
		m_Mass = m;
		m_radius = r;

		m_accVec = m_forVec / m_Mass;
	}

	void updateVal()
	{
		m_forVec = m_forRes;
		m_accVec = m_forVec / m_Mass;
		m_forRes = vectorP(0.0f, 0.0f);
	}

	void forsum(vectorP force)
	{
		m_forRes += force;
	}

	void GetVal()
	{
		LOG(m_Mass <<"\n" << m_radius << "\n" << m_posVec << "\n" << m_velVec << "\n" << m_accVec << "\n" << m_forVec << "\n" << "---------");
	}

	std::unique_ptr<Body> clone() const
	{
		return std::make_unique<Body>(m_Mass, m_radius, movability, m_posVec, m_velVec, m_forVec);
	}

	vectorP lP()
	{
		return (m_velVec * m_Mass);
	}

	double aP()
	{
		return (m_posVec.cross(lP()));
	}

};

struct CollisionResult {
	std::vector<std::unique_ptr<Body>> deadBodies;
	std::vector<std::vector<Body>> clusters;
};

// ═══════════════════════════════════════════════════════════════════════════
//  BARNES-HUT  O(N log N) gravity — paste inside namespace physics { },
//  right before the closing } at the end of the namespace (after moveHermite).
//
//  Provides three drop-in replacements:
//    resolveBarnesHut   → replace  resolve()          in moveVerlet / moveYoshida
//    resolveWithJerkBH  → replace  resolveWithJerk()  in moveHermite
//    computeAccelBH     → replace  computeAccel()     in moveRK45
//
//  Theta (opening-angle criterion):
//    0.0 = exact O(N²)   0.5 = standard   1.0 = fast/coarse
// ═══════════════════════════════════════════════════════════════════════════

// ──────────────────────────────────────────────────────────────────────────
//  Internal quadtree implementation
// ──────────────────────────────────────────────────────────────────────────


// ═══════════════════════════════════════════════════════════════════════════
//  FAST MULTIPOLE METHOD (FMM)  O(N) 2D Gravity Engine
// ═══════════════════════════════════════════════════════════════════════════

namespace fmm {

    constexpr int P = 10; // Order of expansion (P=10 gives ~10-12 digits of precision)
    using Cplx = std::complex<double>;

    // Precalculated Binomial Coefficients (Pascal's Triangle)
    struct PascalTable {
        double C[2 * P + 2][2 * P + 2];
        PascalTable() {
            for (int i = 0; i <= 2 * P + 1; ++i) {
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
        int level;
        int parent;
        int ch[4];
        bool is_leaf;
        std::vector<int> bodies;

        Cplx M[P + 1]; // Multipole expansion
        Cplx L[P + 1]; // Local expansion

        double com_vx, com_vy; // Centre-of-mass velocity (for Jerk computation)

        Node() : cx(0), cy(0), half(0), level(0), parent(-1), is_leaf(true), com_vx(0), com_vy(0) {
            for (int i = 0; i < 4; ++i) ch[i] = -1;
            for (int k = 0; k <= P; ++k) { M[k] = 0.0; L[k] = 0.0; }
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

    // ── Recursive Tree Build ────────────────────────────────────────────────
    static void build_sub(int node_idx, const std::vector<vectorP>& pos, const std::vector<vectorP>* vel_ptr, int max_depth) {
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

            build_sub(child_idx, pos, vel_ptr, max_depth);
        }
    }

    // ── P2M: Particle to Multipole ──────────────────────────────────────────
    static void p2m(int node_idx, const std::vector<vectorP>& pos, const std::vector<double>& masses) {
        Node& nd = tree[node_idx];
        Cplx z_C(nd.cx, nd.cy);
        for (int k = 0; k <= P; ++k) nd.M[k] = 0.0;

        for (int idx : nd.bodies) {
            Cplx z_i(pos[idx].icap, pos[idx].jcap);
            Cplx dz = z_i - z_C;
            double m = masses[idx];

            Cplx dz_pow = 1.0;
            for (int k = 0; k <= P; ++k) {
                nd.M[k] += (G * m) * dz_pow;
                dz_pow *= dz;
            }
        }
    }

    // ── M2M: Multipole to Multipole (Child -> Parent) ───────────────────────
    static void m2m(int parent_idx, int child_idx) {
        Node& p = tree[parent_idx];
        const Node& c = tree[child_idx];
        Cplx dz = Cplx(c.cx, c.cy) - Cplx(p.cx, p.cy);

        for (int k = 0; k <= P; ++k) {
            for (int m = 0; m <= k; ++m) {
                p.M[k] += pascal.C[k][m] * c.M[m] * std::pow(dz, k - m);
            }
        }
    }

    // ── M2L: Multipole to Local (Source -> Target) ──────────────────────────
    static void m2l(int target_idx, int source_idx) {
        Node& target = tree[target_idx];
        const Node& source = tree[source_idx];

        Cplx D = Cplx(source.cx, source.cy) - Cplx(target.cx, target.cy);
        Cplx invD = 1.0 / D;

        std::vector<Cplx> invD_pow(2 * P + 2, 1.0);
        for (size_t i = 1; i < invD_pow.size(); ++i) invD_pow[i] = invD_pow[i - 1] * invD;

        for (int l = 0; l <= P; ++l) {
            Cplx sum = 0.0;
            for (int k = 0; k <= P; ++k) {
                sum += pascal.C[l + k][k] * source.M[k] * invD_pow[l + k + 1];
            }
            target.L[l] += sum;
        }
    }

    // ── L2L: Local to Local (Parent -> Child) ───────────────────────────────
    static void l2l(int parent_idx, int child_idx) {
        const Node& p = tree[parent_idx];
        Node& c = tree[child_idx];
        Cplx dz = Cplx(c.cx, c.cy) - Cplx(p.cx, p.cy);

        for (int l = 0; l <= P; ++l) {
            Cplx sum = 0.0;
            for (int m = l; m <= P; ++m) {
                sum += pascal.C[m][l] * p.L[m] * std::pow(dz, m - l);
            }
            c.L[l] += sum;
        }
    }

    // ── L2P: Local Expansion to Acceleration & Jerk ────────────────────────
    static void l2p(int leaf_idx, double px, double py, double vx, double vy, vectorP& acc_out, vectorP* jerk_out) {
        const Node& nd = tree[leaf_idx];
        Cplx dz = Cplx(px, py) - Cplx(nd.cx, nd.cy);

        Cplx accel_cplx = 0.0;
        Cplx dz_pow = 1.0;

        for (int l = 0; l <= P; ++l) {
            accel_cplx += nd.L[l] * dz_pow;
            dz_pow *= dz;
        }

        acc_out.icap += accel_cplx.real();
        acc_out.jcap += -accel_cplx.imag();

        if (jerk_out) {
            Cplx dz_rel_v = Cplx(vx - nd.com_vx, vy - nd.com_vy);
            Cplx jerk_cplx = 0.0;
            Cplx dz_pow_j = 1.0;

            for (int l = 1; l <= P; ++l) {
                jerk_cplx += (double)l * nd.L[l] * dz_pow_j * dz_rel_v;
                dz_pow_j *= dz;
            }
            jerk_out->icap += jerk_cplx.real();
            jerk_out->jcap += -jerk_cplx.imag();
        }
    }

    // ── Core Engine Execution Routine ───────────────────────────────────────
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

        // Bounding Box
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

        build_sub(root, pos, vel_ptr, max_depth);

        // Group Nodes by Level
        std::vector<std::vector<int>> levels(max_depth + 1);
        std::vector<int> leaves;
        for (int i = 0; i < next_node; ++i) {
            levels[tree[i].level].push_back(i);
            if (tree[i].is_leaf) leaves.push_back(i);
        }

        // 1. P2M Pass
        for (int leaf_idx : leaves) p2m(leaf_idx, pos, masses);

        // 2. M2M Pass (Upward)
        for (int l = max_depth - 1; l >= 0; --l) {
            for (int idx : levels[l]) {
                if (!tree[idx].is_leaf) {
                    for (int q = 0; q < 4; ++q) {
                        int ch = tree[idx].ch[q];
                        if (ch != -1) m2m(idx, ch);
                    }
                }
            }
        }

        // 3. M2L Pass (Interaction Lists)
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

        // 4. L2L Pass (Downward)
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

        // 5. Far-Field L2P + Near-Field Direct P2P Evaluation
        for (int u_leaf : leaves) {
            Node& u_nd = tree[u_leaf];

            // Evaluate Far-Field expansion for all bodies in this leaf
            for (int bi : u_nd.bodies) {
                if (!bodies[bi]->movability) continue;
                double vx = vel_ptr ? (*vel_ptr)[bi].icap : 0.0;
                double vy = vel_ptr ? (*vel_ptr)[bi].jcap : 0.0;
                vectorP* j_ptr = jerk_out ? &((*jerk_out)[bi]) : nullptr;
                l2p(u_leaf, pos[bi].icap, pos[bi].jcap, vx, vy, acc_out[bi], j_ptr);
            }

            // Direct P2P for neighboring leaves
            for (int v_leaf : leaves) {
                if (!is_neighbor(u_nd, tree[v_leaf])) continue;

                for (int i : u_nd.bodies) {
                    if (!bodies[i]->movability) continue;
                    for (int j : tree[v_leaf].bodies) {
                        if (i == j) continue; // Skip self-interaction

                        vectorP r = pos[j] - pos[i];
                        double eps = 0.1;
                        double r2 = r.magSq() + eps * eps;
                        double inv_r = 1.0 / std::sqrt(r2);
                        double inv_r3 = inv_r * inv_r * inv_r;

                        acc_out[i] += r * (G * masses[j] * inv_r3);

                        if (jerk_out && vel_ptr) {
                            vectorP v = (*vel_ptr)[j] - (*vel_ptr)[i];
                            double v_dot_r = v.icap * r.icap + v.jcap * r.jcap;
                            double inv_r5 = inv_r3 * inv_r * inv_r;
                            (*jerk_out)[i] += (v * inv_r3 - r * (3.0 * v_dot_r * inv_r5)) * (G * masses[j]);
                        }
                    }
                }
            }
        }
    }
} // namespace fmm

namespace bh {

    // Each node represents one cell of the recursive square partition.
    //
    //  Quadrant layout (bit-field: x-bit | y-bit):
    //    0 = SW  (x < cx, y < cy)
    //    1 = SE  (x ≥ cx, y < cy)
    //    2 = NW  (x < cx, y ≥ cy)
    //    3 = NE  (x ≥ cx, y ≥ cy)
    struct Node {
        double cx, cy;        // centre of the bounding square
        double half;          // half-side (square spans [cx-half,cx+half] × [cy-half,cy+half])
        double mass;          // total mass of all particles in sub-tree
        double com_x, com_y;  // centre-of-mass position
        double com_vx,com_vy; // centre-of-mass velocity  (only set when building for jerk)
        int    body;          // ≥ 0 : leaf holding this body index
                              // -1  : internal node or empty leaf
        int    ch[4];         // child node indices; -1 = child absent
    };

    // Bump allocator: the pool is pre-sized before any inserts so it never
    // reallocates during the build, keeping Node& references valid throughout.
    static std::vector<Node> pool;
    static int               next_node;

    static int alloc(double cx, double cy, double half)
    {
        int idx = next_node++;
        pool[idx] = { cx, cy, half,  0,0,0,  0,0,  -1, {-1,-1,-1,-1} };
        return idx;
    }

    // Quadrant of (px,py) relative to box centre (cx,cy)
    inline int quadrant(double cx, double cy, double px, double py)
    {
        return (px >= cx ? 1 : 0) | (py >= cy ? 2 : 0);
    }

    // Child centre for quadrant q at current centre (cx,cy) and half-size half
    inline double child_cx(double cx, double half, int q) { return cx + ((q & 1) ? half : -half); }
    inline double child_cy(double cy, double half, int q) { return cy + ((q & 2) ? half : -half); }

    // ── Insert body bi into the sub-tree rooted at node_idx ──────────────
    // All accesses use pool[idx] (index, never a cached reference) so they
    // remain correct even if this function is called recursively and triggers
    // further alloc() calls inside the same pre-allocated buffer.
    //
    // vel_ptr: pointer to velocity array; nullptr when jerk is not needed.
    static void insert(int bi, double px, double py, double bm, double bvx, double bvy,
                       const std::vector<vectorP>& pos,
                       const std::vector<double>&  masses,
                       const std::vector<vectorP>* vel_ptr,
                       int node_idx)
    {
        while (true) {
            // ── Case 1: empty leaf → place body here ──────────────────────
            bool empty = (pool[node_idx].body == -1 &&
                          pool[node_idx].ch[0] == -1 && pool[node_idx].ch[1] == -1 &&
                          pool[node_idx].ch[2] == -1 && pool[node_idx].ch[3] == -1);
            if (empty) {
                pool[node_idx].body  = bi;
                pool[node_idx].mass  = bm;
                pool[node_idx].com_x = px;  pool[node_idx].com_y = py;
                pool[node_idx].com_vx= bvx; pool[node_idx].com_vy= bvy;
                return;
            }

            // Update this node's aggregate mass and centre-of-mass
            double new_mass = pool[node_idx].mass + bm;
            pool[node_idx].com_x  = (pool[node_idx].com_x  * pool[node_idx].mass + px  * bm) / new_mass;
            pool[node_idx].com_y  = (pool[node_idx].com_y  * pool[node_idx].mass + py  * bm) / new_mass;
            pool[node_idx].com_vx = (pool[node_idx].com_vx * pool[node_idx].mass + bvx * bm) / new_mass;
            pool[node_idx].com_vy = (pool[node_idx].com_vy * pool[node_idx].mass + bvy * bm) / new_mass;
            pool[node_idx].mass   = new_mass;

            // ── Case 2: occupied leaf → subdivide ─────────────────────────
            // Promote to internal node, push the resident body to a child,
            // then fall through to insert bi (tail-call via the while loop).
            int existing = pool[node_idx].body;
            if (existing != -1) {
                pool[node_idx].body = -1;

                // Guard against infinite subdivision when two particles
                // sit at (nearly) identical positions.
                if (pool[node_idx].half >= 1e-10) {
                    double cx = pool[node_idx].cx, cy = pool[node_idx].cy;
                    double h2 = pool[node_idx].half * 0.5;

                    double ex  = pos[existing].icap, ey  = pos[existing].jcap;
                    double evx = vel_ptr ? (*vel_ptr)[existing].icap : 0.0;
                    double evy = vel_ptr ? (*vel_ptr)[existing].jcap : 0.0;
                    int q_ex   = quadrant(cx, cy, ex, ey);

                    if (pool[node_idx].ch[q_ex] == -1)
                        pool[node_idx].ch[q_ex] = alloc(child_cx(cx, h2, q_ex),
                                                        child_cy(cy, h2, q_ex), h2);

                    // Recursive call depth ≤ tree depth ≈ O(log(domain/eps)) ≈ 50 max.
                    insert(existing, ex, ey, masses[existing], evx, evy,
                           pos, masses, vel_ptr, pool[node_idx].ch[q_ex]);
                    // pool[node_idx] still valid — no realloc (pre-allocated pool)
                }
            }

            // ── Case 3: descend for bi ────────────────────────────────────
            double cx = pool[node_idx].cx, cy = pool[node_idx].cy;
            double h2 = pool[node_idx].half * 0.5;
            int q = quadrant(cx, cy, px, py);

            if (pool[node_idx].ch[q] == -1)
                pool[node_idx].ch[q] = alloc(child_cx(cx, h2, q), child_cy(cy, h2, q), h2);

            node_idx = pool[node_idx].ch[q];   // tail-call as iteration
        }
    }

    // ── Apply the contribution of one node (or leaf body) to (ax,ay,jx,jy) ─
    static inline void apply(double mass_n, double dx, double dy,
                              double dvx, double dvy,
                              double& ax, double& ay,
                              double* jx, double* jy)
    {
        // r²_softened = |r|² + ε²   (ε = 0.1 matches the value used in resolve/computeAccel)
        double r2s    = dx*dx + dy*dy + 0.01;
        double inv_r  = 1.0 / std::sqrt(r2s);
        double inv_r3 = inv_r * inv_r * inv_r;

        ax += G * mass_n * dx * inv_r3;
        ay += G * mass_n * dy * inv_r3;

        if (jx) {   // jerk: d/dt[a] = G*M*[v_rel/r³ − 3(v_rel·r)r/r⁵]
            double vdr    = dvx*dx + dvy*dy;
            double inv_r5 = inv_r3 * inv_r * inv_r;
            *jx += G * mass_n * (dvx * inv_r3 - 3.0 * vdr * dx * inv_r5);
            *jy += G * mass_n * (dvy * inv_r3 - 3.0 * vdr * dy * inv_r5);
        }
    }

    // ── Tree-walk: accumulate force (and optionally jerk) on body bi ──────
    // theta_sq = θ².  jx/jy may be nullptr when jerk is not needed.
    static void walk(int node_idx, int bi,
                     double bx, double by, double bvx, double bvy,
                     double theta_sq,
                     double& ax, double& ay,
                     double* jx, double* jy)
    {
        if (node_idx == -1) return;
        const Node& n = pool[node_idx];   // read-only walk — no alloc → reference safe
        if (n.mass == 0.0) return;

        double dx = n.com_x - bx, dy = n.com_y - by;
        double r2 = dx*dx + dy*dy;          // geometric r² (no softening, used for criterion)

        // ── Leaf: direct pair ─────────────────────────────────────────────
        if (n.body != -1) {
            if (n.body == bi) return;       // skip self-interaction
            apply(n.mass, dx, dy,
                  n.com_vx - bvx, n.com_vy - bvy,
                  ax, ay, jx, jy);
            return;
        }

        // ── Internal node: opening-angle criterion ────────────────────────
        // s = 2*half (cell width).  Accept approximation when s²/r² < θ²,
        // i.e. 4*half² < θ²*r².  Using squared form avoids a sqrt here.
        if (r2 > 0.0 && 4.0 * n.half * n.half < theta_sq * r2) {
            apply(n.mass, dx, dy,
                  n.com_vx - bvx, n.com_vy - bvy,
                  ax, ay, jx, jy);
            return;
        }

        // ── Open: recurse into children ───────────────────────────────────
        for (int q = 0; q < 4; q++)
            walk(n.ch[q], bi, bx, by, bvx, bvy, theta_sq, ax, ay, jx, jy);
    }

    // ── Build the quadtree from scratch ───────────────────────────────────
    // vel_ptr: nullptr → skip velocity tracking (jerk not needed)
    // Returns root node index.
    static int build(const std::vector<vectorP>& pos,
                     const std::vector<double>&  masses,
                     const std::vector<vectorP>* vel_ptr = nullptr)
    {
        const int s = (int)pos.size();

        // Bounding square enclosing all particles
        double xmin = pos[0].icap, xmax = pos[0].icap;
        double ymin = pos[0].jcap, ymax = pos[0].jcap;
        for (int i = 1; i < s; i++) {
            xmin = std::min(xmin, pos[i].icap); xmax = std::max(xmax, pos[i].icap);
            ymin = std::min(ymin, pos[i].jcap); ymax = std::max(ymax, pos[i].jcap);
        }
        double cx   = (xmin + xmax) * 0.5, cy = (ymin + ymax) * 0.5;
        // half must be strictly larger than any particle offset so no body
        // sits exactly on a cell boundary.
        double half = std::max({ (xmax - xmin) * 0.5,
                                 (ymax - ymin) * 0.5,
                                 1e-6 }) * 1.001;

        // Pre-allocate pool.  Worst-case nodes per body ≈ tree_depth ≈ 60
        // (double precision, domain/eps ≈ 1e14).  64*s is a safe upper bound.
        int max_nodes = std::max(64 * s, 256);
        if ((int)pool.size() < max_nodes) pool.resize(max_nodes);
        next_node = 0;

        int root = alloc(cx, cy, half);
        for (int i = 0; i < s; i++) {
            double vx = vel_ptr ? (*vel_ptr)[i].icap : 0.0;
            double vy = vel_ptr ? (*vel_ptr)[i].jcap : 0.0;
            insert(i, pos[i].icap, pos[i].jcap, masses[i], vx, vy,
                   pos, masses, vel_ptr, root);
        }
        return root;
    }

} // namespace bh

// ═══════════════════════════════════════════════════════════════════════════
//  KS / LEVI-CIVITA REGULARIZATION ENGINE (2D N-Body Universal Adapter)
//  Paste this directly above: namespace physics {
// ═══════════════════════════════════════════════════════════════════════════

namespace ks_regularization {

    // Threshold distance: pairs closer than this will dynamically transition
    // into KS regularized space. Tune this based on your scale!
    static constexpr double R_KS_THRESHOLD = 2.5;

    struct RegularizedPair {
        int idxA;
        int idxB;
        double u1, u2;       // Regularized position u
        double up1, up2;     // Regularized velocity u' = du/d(tau)
        double H;            // Keplerian binding energy
        double mu;           // Gravitational parameter G * (M1 + M2)
        double M_tot;        // Total mass M1 + M2
        double m1, m2;       // Individual masses
        vectorP com_pos;     // Center of mass position
        vectorP com_vel;     // Center of mass velocity
    };

    // ── 1. Map Physical Coordinates (x, y, vx, vy) -> KS Space (u, u') ──
    static void physicalToKS(const vectorP& rel_pos, const vectorP& rel_vel,
                             double M1, double M2, RegularizedPair& ks)
    {
        double x = rel_pos.icap;
        double y = rel_pos.jcap;
        double r = rel_pos.mag();

        // Robust LC Position Inversion: u1^2 - u2^2 = x, 2*u1*u2 = y
        if (x >= 0.0) {
            ks.u1 = std::sqrt((r + x) * 0.5);
            ks.u2 = (ks.u1 > 1e-15) ? (y / (2.0 * ks.u1)) : 0.0;
        } else {
            ks.u2 = (y >= 0.0 ? 1.0 : -1.0) * std::sqrt((r - x) * 0.5);
            ks.u1 = (std::abs(ks.u2) > 1e-15) ? (y / (2.0 * ks.u2)) : 0.0;
        }

        // LC Velocity Inversion via Sundman: u' = 1/2 * r * (u_rot * v)
        // u'_1 = 0.5 * (vx * u1 + vy * u2)
        // u'_2 = 0.5 * (vy * u1 - vx * u2)
        double vx = rel_vel.icap;
        double vy = rel_vel.jcap;
        ks.up1 = 0.5 * (vx * ks.u1 + vy * ks.u2);
        ks.up2 = 0.5 * (vy * ks.u1 - vx * ks.u2);

        // Keplerian Energy: H = 0.5 * v^2 - G*(M1+M2)/r
        ks.mu = G * (M1 + M2);
        double v_sq = rel_vel.magSq();
        ks.H = 0.5 * v_sq - (ks.mu / (r + 1e-15));
        ks.M_tot = M1 + M2;
        ks.m1 = M1;
        ks.m2 = M2;
    }

    // ── 2. Map KS Space (u, u') -> Physical Coordinates (x, y, vx, vy) ──
    static void ksToPhysical(const RegularizedPair& ks, vectorP& rel_pos, vectorP& rel_vel)
    {
        // x = u1^2 - u2^2, y = 2*u1*u2
        rel_pos.icap = ks.u1 * ks.u1 - ks.u2 * ks.u2;
        rel_pos.jcap = 2.0 * ks.u1 * ks.u2;

        double r = ks.u1 * ks.u1 + ks.u2 * ks.u2;
        if (r < 1e-15) { rel_vel = vectorP(0, 0); return; }

        // vx = 2*(u1*up1 - u2*up2)/r, vy = 2*(u2*up1 + u1*up2)/r
        double inv_r = 2.0 / r;
        rel_vel.icap = (ks.u1 * ks.up1 - ks.u2 * ks.up2) * inv_r;
        rel_vel.jcap = (ks.u2 * ks.up1 + ks.u1 * ks.up2) * inv_r;
    }

    // ── 3. Exact Analytical Solver for the KS Harmonic Oscillator ──
    // Solves u'' - (H/2)u = 0 over physical time step delta_t
    static void stepKSAnalytical(RegularizedPair& ks, double delta_t)
    {
        double r = ks.u1 * ks.u1 + ks.u2 * ks.u2;
        if (r < 1e-15) r = 1e-15;

        // Sundman time step increment: d(tau) = dt / r
        double dtau = delta_t / r;

        double u1_new, u2_new, up1_new, up2_new;

        if (ks.H < 0.0) {
            // Bound Orbit (Elliptical): Exact Simple Harmonic Motion!
            double omega = std::sqrt(-0.5 * ks.H);
            double cos_wt = std::cos(omega * dtau);
            double sin_wt = std::sin(omega * dtau);
            double inv_omega = 1.0 / omega;

            u1_new  = ks.u1 * cos_wt  + ks.up1 * inv_omega * sin_wt;
            up1_new = -ks.u1 * omega * sin_wt + ks.up1 * cos_wt;

            u2_new  = ks.u2 * cos_wt  + ks.up2 * inv_omega * sin_wt;
            up2_new = -ks.u2 * omega * sin_wt + ks.up2 * cos_wt;
        }
        else if (ks.H > 0.0) {
            // Unbound Orbit (Hyperbolic): Exponential / Hyperbolic motion
            double omega = std::sqrt(0.5 * ks.H);
            double cosh_wt = std::cosh(omega * dtau);
            double sinh_wt = std::sinh(omega * dtau);
            double inv_omega = 1.0 / omega;

            u1_new  = ks.u1 * cosh_wt + ks.up1 * inv_omega * sinh_wt;
            up1_new = ks.u1 * omega * sinh_wt + ks.up1 * cosh_wt;

            u2_new  = ks.u2 * cosh_wt + ks.up2 * inv_omega * sinh_wt;
            up2_new = ks.u2 * omega * sinh_wt + ks.up2 * cosh_wt;
        }
        else {
            // Parabolic Orbit (H == 0): Linear drift in KS space
            u1_new = ks.u1 + ks.up1 * dtau;
            u2_new = ks.u2 + ks.up2 * dtau;
            up1_new = ks.up1;
            up2_new = ks.up2;
        }

        ks.u1 = u1_new; ks.u2 = u2_new;
        ks.up1 = up1_new; ks.up2 = up2_new;
    }

    // ── 4. Universal Interceptor: Pre-Process Before Any Integrator ──
    // Detects close pairs, extracts them into KS space, and replaces them with a COM body
    static std::vector<RegularizedPair> extractCloseEncounters(std::vector<std::unique_ptr<Body>>& bodies)
    {
        std::vector<RegularizedPair> active_ks_pairs;
        int s = (int)bodies.size();

        for (int i = 0; i < s - 1; i++) {
            if (!bodies[i] || bodies[i]->dead || !bodies[i]->movability) continue;

            for (int j = i + 1; j < s; j++) {
                if (!bodies[j] || bodies[j]->dead || !bodies[j]->movability) continue;

                vectorP rel_pos = bodies[j]->m_posVec - bodies[i]->m_posVec;
                double dist = rel_pos.mag();

                // Check if they entered the KS Regularization zone (but haven't physically collided yet)
                double min_col_dist = bodies[i]->m_radius + bodies[j]->m_radius;
                if (dist < R_KS_THRESHOLD && dist > min_col_dist) {

                    RegularizedPair pair;
                    pair.idxA = i;
                    pair.idxB = j;

                    vectorP rel_vel = bodies[j]->m_velVec - bodies[i]->m_velVec;
                    physicalToKS(rel_pos, rel_vel, bodies[i]->m_Mass, bodies[j]->m_Mass, pair);

                    // Calculate Center of Mass (COM) state
                    pair.com_pos = (bodies[i]->m_posVec * pair.m1 + bodies[j]->m_posVec * pair.m2) / pair.M_tot;
                    pair.com_vel = (bodies[i]->m_velVec * pair.m1 + bodies[j]->m_velVec * pair.m2) / pair.M_tot;

                    // Temporarily park Body A at the COM with combined mass so external bodies
                    // still feel the correct gravity during the global integrator step!
                    bodies[i]->m_posVec = pair.com_pos;
                    bodies[i]->m_velVec = pair.com_vel;
                    bodies[i]->m_Mass   = pair.M_tot;

                    // Temporarily disable Body B so global solver ignores the internal 1/r^2 spike
                    bodies[j]->movability = false;
                    bodies[j]->m_posVec = pair.com_pos; // keep nearby for grid drawing
                    bodies[j]->m_velVec = pair.com_vel;

                    active_ks_pairs.push_back(pair);
                    break; // Move to next body
                }
            }
        }
        return active_ks_pairs;
    }

    // ── 5. Universal Interceptor: Post-Process After Any Integrator ──
    // Steps the KS binary forward and restores individual bodies to the main array
    static void restoreCloseEncounters(std::vector<std::unique_ptr<Body>>& bodies,
                                       std::vector<RegularizedPair>& active_ks_pairs,
                                       double step_dt)
    {
        for (auto& pair : active_ks_pairs) {
            Body* bodA = bodies[pair.idxA].get();
            Body* bodB = bodies[pair.idxB].get();

            // 1. Advance the internal binary orbit smoothly in KS space
            stepKSAnalytical(pair, step_dt);

            // 2. Map back to Cartesian physical relative coordinates
            vectorP rel_pos(0, 0), rel_vel(0, 0);
            ksToPhysical(pair, rel_pos, rel_vel);

            // 3. Retrieve updated COM position/velocity from the global integrator
            vectorP updated_com_pos = bodA->m_posVec;
            vectorP updated_com_vel = bodA->m_velVec;

            // 4. Restore original individual masses and reactivate Body B
            bodA->m_Mass = pair.m1;
            bodB->m_Mass = pair.m2;
            bodB->movability = true;

            // 5. Unpack absolute positions and velocities from COM + Relative states
            // r_A = r_COM - (m2 / M_tot) * r_rel
            // r_B = r_COM + (m1 / M_tot) * r_rel
            double fracA = pair.m2 / pair.M_tot;
            double fracB = pair.m1 / pair.M_tot;

            bodA->m_posVec = updated_com_pos - rel_pos * fracA;
            bodA->m_velVec = updated_com_vel - rel_vel * fracA;

            bodB->m_posVec = updated_com_pos + rel_pos * fracB;
            bodB->m_velVec = updated_com_vel + rel_vel * fracB;

            // Update forces to reflect new positions for UI synchronization
            bodA->updateVal();
            bodB->updateVal();
        }
    }

    // ── 6. Universal Sundman Time-Step Regulator (Optional Helper) ──
    // Automatically shrinks dt globally when any two bodies approach, preventing integrator rejection
    static double getSundmanAdaptiveDT(const std::vector<std::unique_ptr<Body>>& bodies, double base_dt) {
        double max_inv_dist = 0.0;
        int s = (int)bodies.size();
        for (int i = 0; i < s - 1; i++) {
            if (!bodies[i] || bodies[i]->dead) continue;
            for (int j = i + 1; j < s; j++) {
                if (!bodies[j] || bodies[j]->dead) continue;
                double dist = (bodies[j]->m_posVec - bodies[i]->m_posVec).mag();
                if (dist > 1e-5) max_inv_dist = std::max(max_inv_dist, 1.0 / dist);
            }
        }
        // If bodies get within distance < 1.0, scale down dt smoothly
        if (max_inv_dist > 1.0) {
            return std::max(base_dt / max_inv_dist, 1e-7);
        }
        return base_dt;
    }
}

namespace physics {
	constexpr double G = 6.67430e-11;

	vectorP displacement(const Body& a,const Body& b)
	{
		return (a.m_posVec - b.m_posVec);
	}

	void pull(Body& a, Body& b)
	{
		vectorP disp = physics::displacement(a, b);
		double eps = 0.1;

		double distSq = disp.magSq() + eps * eps;
		double invdist =  1.0/sqrt(distSq);
		double denom = invdist * invdist * invdist ;

		vectorP pullvec = (disp) * ((physics::G * a.m_Mass * b.m_Mass) * denom);

		b.forsum(pullvec);
		pullvec.negate_inp();
		a.forsum(pullvec);
		
		//LOG(pullvec);
	}

	struct CollisionResult checkCol(std::vector<std::unique_ptr<Body>>& bodies , std::vector<std::vector<Body*>>& colClusters)
	{
		std::vector<std::unique_ptr<Body>> addtobodies;
		std::vector<std::vector<Body>> tempClusters;
		int const nBodies = static_cast<int>(std::size(bodies));
		int clusterIndex = 1;
		for (int i = 0; i < (nBodies - 1); i++)
		{
			
			Body& boda = *bodies[i];
			int& clusInA = boda.clusterIndex;
			for (int j = i+1; j < nBodies; j++)
			{
				Body& bodb = *bodies[j];
				double disp = displacement(boda, bodb).mag();
				double mindisp = boda.m_radius + bodb.m_radius;
				int& clusInB = bodb.clusterIndex;
				
				

				if (disp < mindisp)
				{
					if (clusInA == 0 && clusInB == 0)//cluster formation
					{
						clusInA = clusInB = clusterIndex;
						std::vector<Body*> clusterTBP;
						clusterTBP.push_back(bodies[i].get());
						clusterTBP.push_back(bodies[j].get());
						colClusters.push_back(clusterTBP);
						clusterIndex++;
					}

					if (clusInA != clusInB /* && clusInA != 0 && clusInB != 0*/) //could potentially remove clusInB != 0 , as it doesnt matter in the case A is not zero and not equal to it
					{
						if (clusInB == 0) //cluster expansion
							clusInB = clusInA;
						if (clusInA == 0)
							clusInA = clusInB;

						if (clusInA < clusInB) //cluster coallition
						{
							auto& clusterB = colClusters[clusInB - 1];
							for (int k = 0; k < clusterB.size() ; k++)
							{
								(*clusterB[k]).clusterIndex = clusInA;
							}
							clusterB.clear();
						}
						if (clusInA > clusInB) //cluster coallition
						{

							auto& clusterA = colClusters[clusInA - 1];
							for (int k = 0; k < clusterA.size(); k++)
							{
								(*clusterA[k]).clusterIndex = clusInB;
							}
							clusterA.clear();
						}
					}
					
					/*float newMass = boda.m_Mass + bodb.m_Mass;
					vectorP newPos = (boda.m_posVec + bodb.m_posVec) / 2;
					vectorP newVec = ((boda.m_velVec * boda.m_Mass ) + (bodb.m_velVec * bodb.m_Mass)) / newMass;
					vectorP newFor = (boda.m_forVec + bodb.m_forVec) / 2; 
					double newRadius = pow(boda.m_radius * boda.m_radius * boda.m_radius + bodb.m_radius * bodb.m_radius * bodb.m_radius, 1.0f / 3.0f);
					auto newBody = std::make_unique<Body>(newMass, newRadius, true,newPos, newVec, newFor);
					
					boda.dead = true;
					//LOG(i << j << "gone poof");
					//boda.GetVal();
					//bodb.GetVal();
					bodb.dead = true;
					bodies.push_back(std::move(newBody));*/
				}

			}
			//clusterIndex++;
		}
		colClusters.clear();
		colClusters.resize(clusterIndex - 1);

		bodies.reserve(bodies.size() + colClusters.size());

		for (int j = 0; j < bodies.size(); j++)
		{
			if ((*bodies[j]).clusterIndex > 0 )
			{
				colClusters[(*bodies[j]).clusterIndex - 1].push_back(bodies[j].get());
			}
		}

		for (int i = 0; i < colClusters.size(); i++)
		{
			if (colClusters[i].empty()) continue;
			//Body boda = (*(colClusters[i][0])); not needed in new version starts from zezro

			float   totalMass  = 0.0f;
			vectorP wPos       = vectorP(0, 0);
			vectorP wVel       = vectorP(0, 0);
			vectorP totalForce = vectorP(0, 0);
			double  totalVol   = 0.0;

			for (int k = 0; k < colClusters[i].size(); k++)
			{
				Body& b = *(colClusters[i][k]);
				totalMass  += b.m_Mass;
				wPos       += b.m_posVec * b.m_Mass;
				wVel       += b.m_velVec * b.m_Mass;
				totalForce += b.m_forVec; //force isnt averages as its a vector and vectors are additive
				totalVol   += b.m_radius * b.m_radius * b.m_radius;
				b.dead = true;
			}

			Body mergedBody      = *(colClusters[i][0]);
			mergedBody.m_Mass    = totalMass;
			mergedBody.m_posVec  = wPos / totalMass;
			mergedBody.m_velVec  = wVel / totalMass;
			mergedBody.m_forVec  = totalForce;
			mergedBody.m_radius  = pow(totalVol, 1.0 / 3.0);
			mergedBody.clusterIndex = 0;
			mergedBody.dead      = false;

			//(*(colClusters[i][0])).dead = true; //done in loop
			//auto newBody = std::make_unique<Body>(mergedBody); compact this shit
			//newBody->clusterIndex = 0; //done already
			//newBody->dead = false; //done already

			addtobodies.push_back(std::move(std::make_unique<Body>(mergedBody)));  // Use the addtobodies vector you already have!

			std::vector<Body> clusterSnapshot;
			LOG("Collisions\n-----------")
			{
				LOG("Cluster " << i+1 << "\n------------ - ")
					for (int j = 0; j < colClusters[i].size(); j++)
					{
						colClusters[i][j]->GetVal();
						clusterSnapshot.push_back(*colClusters[i][j]);
					}
			}
			tempClusters.push_back(clusterSnapshot);

		}
		// Now add all new bodies after the loop
		for (auto& nb : addtobodies)
		{
			bodies.push_back(std::move(nb));
		}

		

		std::vector<std::unique_ptr<Body>> deadBodies;
		deadBodies.reserve(bodies.size()); // optional but nice

		auto it = bodies.begin();
		while (it != bodies.end())
		{
			if ((*it)->dead)
			{
				// move the unique_ptr into deadBodies
				deadBodies.push_back(std::move(*it));
				it = bodies.erase(it);
			}
			else {
				++it;
			}
		}

		colClusters.clear();

		return { std::move(deadBodies), std::move(tempClusters) };

	}

	// ═══════════════════════════════════════════════════════════════════════════
	//  SPATIAL HASHING COLLISION DETECTION O(N) — Drop-in replacement for checkCol
	// ═══════════════════════════════════════════════════════════════════════════

	// Fast 64-bit bit-packing coordinate hash
	inline uint64_t hashGridCoords(int32_t x, int32_t y) {
		return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
		        static_cast<uint32_t>(y);
	}

	struct CollisionResult checkColSpatialHash(
	    std::vector<std::unique_ptr<Body>>& bodies,
	    std::vector<std::vector<Body*>>& colClusters)
	{
		std::vector<std::unique_ptr<Body>> addtobodies;
		std::vector<std::vector<Body>> tempClusters;
		const int nBodies = static_cast<int>(bodies.size());
		if (nBodies < 2) return { {}, {} };

		// 1. DYNAMIC CELL SIZING
		// Find the largest diameter in the current system to ensure 3x3 neighborhood capture
		double max_diam = 1e-5;
		for (int i = 0; i < nBodies; i++) {
			if (!bodies[i]->dead) {
				max_diam = std::max(max_diam, 2.0 * bodies[i]->m_radius);
			}
		}
		const double cell_size = std::max(max_diam, 1.0); // Floor size to prevent div by zero
		const double inv_cell_size = 1.0 / cell_size;

		// 2. POPULATE THE SPATIAL HASH GRID
		// Map: [Packed 64-bit Coord Key] -> [Vector of Body Indices]
		std::unordered_map<uint64_t, std::vector<int>> spatialGrid;
		spatialGrid.reserve(nBodies);

		for (int i = 0; i < nBodies; i++) {
			if (bodies[i]->dead) continue;
			int32_t cx = static_cast<int32_t>(std::floor(bodies[i]->m_posVec.icap * inv_cell_size));
			int32_t cy = static_cast<int32_t>(std::floor(bodies[i]->m_posVec.jcap * inv_cell_size));
			spatialGrid[hashGridCoords(cx, cy)].push_back(i);
		}

		// 3. BROAD-PHASE & NARROW-PHASE COLLISION SEARCH
		int clusterIndex = 1;

		for (int i = 0; i < nBodies; i++) {
			if (bodies[i]->dead) continue;
			Body& boda = *bodies[i];

			int32_t cx = static_cast<int32_t>(std::floor(boda.m_posVec.icap * inv_cell_size));
			int32_t cy = static_cast<int32_t>(std::floor(boda.m_posVec.jcap * inv_cell_size));

			// Query the 3x3 surrounding cell block
			for (int32_t dx = -1; dx <= 1; dx++) {
				for (int32_t dy = -1; dy <= 1; dy++) {

					uint64_t neighborKey = hashGridCoords(cx + dx, cy + dy);
					auto it = spatialGrid.find(neighborKey);
					if (it == spatialGrid.end()) continue;

					// Check all candidate bodies inside this cell
					for (int j : it->second) {
						// STRICT ORDERING: j > i avoids duplicate checks and self-comparisons
						if (j <= i || bodies[j]->dead) continue;

						Body& bodb = *bodies[j];
						double disp = displacement(boda, bodb).mag();
						double mindisp = boda.m_radius + bodb.m_radius;

						// NARROW-PHASE: Exact boundary overlap check
						if (disp < mindisp) {
							int& clusInA = boda.clusterIndex;
							int& clusInB = bodb.clusterIndex;

							// Case A: New Cluster Formation
							if (clusInA == 0 && clusInB == 0) {
								clusInA = clusInB = clusterIndex;
								std::vector<Body*> clusterTBP;
								clusterTBP.push_back(bodies[i].get());
								clusterTBP.push_back(bodies[j].get());
								colClusters.push_back(clusterTBP);
								clusterIndex++;
							}
							// Case B: Cluster Expansion or Merging
							else if (clusInA != clusInB) {
								if (clusInB == 0) clusInB = clusInA;
								if (clusInA == 0) clusInA = clusInB;

								if (clusInA < clusInB) {
									auto& clusterB = colClusters[clusInB - 1];
									for (size_t k = 0; k < clusterB.size(); k++) {
										clusterB[k]->clusterIndex = clusInA;
									}
									clusterB.clear();
								}
								else if (clusInA > clusInB) {
									auto& clusterA = colClusters[clusInA - 1];
									for (size_t k = 0; k < clusterA.size(); k++) {
										clusterA[k]->clusterIndex = clusInB;
									}
									clusterA.clear();
								}
							}
						}
					}
				}
			}
		}

		// 4. CLUSTER RESOLUTION & BODY MERGING (Identical to your original logic)
		colClusters.clear();
		colClusters.resize(clusterIndex - 1);
		bodies.reserve(bodies.size() + colClusters.size());

		for (size_t j = 0; j < bodies.size(); j++) {
			if (bodies[j]->clusterIndex > 0) {
				colClusters[bodies[j]->clusterIndex - 1].push_back(bodies[j].get());
			}
		}

		for (size_t i = 0; i < colClusters.size(); i++) {
			if (colClusters[i].empty()) continue;

			float   totalMass  = 0.0f;
			vectorP wPos       = vectorP(0, 0);
			vectorP wVel       = vectorP(0, 0);
			vectorP totalForce = vectorP(0, 0);
			double  totalVol   = 0.0;

			for (size_t k = 0; k < colClusters[i].size(); k++) {
				Body& b = *(colClusters[i][k]);
				totalMass  += b.m_Mass;
				wPos       += b.m_posVec * b.m_Mass;
				wVel       += b.m_velVec * b.m_Mass;
				totalForce += b.m_forVec;
				totalVol   += b.m_radius * b.m_radius * b.m_radius;
				b.dead = true;
			}

			Body mergedBody      = *(colClusters[i][0]);
			mergedBody.m_Mass    = totalMass;
			mergedBody.m_posVec  = wPos / totalMass;
			mergedBody.m_velVec  = wVel / totalMass;
			mergedBody.m_forVec  = totalForce;
			mergedBody.m_radius  = pow(totalVol, 1.0 / 3.0);
			mergedBody.clusterIndex = 0;
			mergedBody.dead      = false;

			addtobodies.push_back(std::make_unique<Body>(mergedBody));

			std::vector<Body> clusterSnapshot;
			LOG("Collisions (Spatial Hash)\n-----------")
			{
				LOG("Cluster " << i + 1 << "\n-------------")
				for (size_t j = 0; j < colClusters[i].size(); j++) {
					colClusters[i][j]->GetVal();
					clusterSnapshot.push_back(*colClusters[i][j]);
				}
			}
			tempClusters.push_back(clusterSnapshot);
		}

		// Append new merged bodies
		for (auto& nb : addtobodies) {
			bodies.push_back(std::move(nb));
		}

		// 5. MEMORY CLEANUP: Erase dead bodies
		std::vector<std::unique_ptr<Body>> deadBodies;
		deadBodies.reserve(bodies.size());

		auto it = bodies.begin();
		while (it != bodies.end()) {
			if ((*it)->dead) {
				deadBodies.push_back(std::move(*it));
				it = bodies.erase(it);
			} else {
				++it;
			}
		}

		colClusters.clear();
		return { std::move(deadBodies), std::move(tempClusters) };
	}

	void resolve(std::vector<std::unique_ptr<Body>>& bodies)
	{
		int s = bodies.size();
		for (int i = 0; i < (s-1); i++)
		{
			auto& bodya = *bodies[i];
			for (int j = i + 1; j < bodies.size() ; j++)
			{
				physics::pull( bodya, *(bodies[j]));
			}
			bodies[i]->updateVal();
		}

		bodies[s - 1]->updateVal();
	}

	void resolveWithJerk(std::vector<std::unique_ptr<Body>>& bodies)
	{
		int s = bodies.size();
		double eps = 0.1; // Softening factor to match your pull() function

		// Reset accelerations and jerks
		for (int i = 0; i < s; i++) {
			bodies[i]->m_accVec = vectorP(0, 0);
			bodies[i]->m_jerkVec = vectorP(0, 0);
		}

		// Pairwise N-Body force & jerk calculation
		for (int i = 0; i < s; i++) {
			for (int j = i + 1; j < s; j++) {
				vectorP r = bodies[j]->m_posVec - bodies[i]->m_posVec;
				vectorP v = bodies[j]->m_velVec - bodies[i]->m_velVec;

				double r2 = r.magSq() + (eps * eps);
				double r1 = std::sqrt(r2);
				double r3 = r2 * r1;
				double r5 = r3 * r2;

				double v_dot_r = (v.icap * r.icap + v.jcap * r.jcap);

				double g_mj = physics::G * bodies[j]->m_Mass;
				double g_mi = physics::G * bodies[i]->m_Mass;

				if (bodies[i]->movability) {
					bodies[i]->m_accVec  += r * (g_mj / r3);
					bodies[i]->m_jerkVec += (v * (1.0 / r3) - r * (3.0 * v_dot_r / r5)) * g_mj;
				}

				if (bodies[j]->movability) {
					bodies[j]->m_accVec  -= r * (g_mi / r3);
					bodies[j]->m_jerkVec -= (v * (1.0 / r3) - r * (3.0 * v_dot_r / r5)) * g_mi;
				}
			}
		}

		// Keep m_forVec updated so collision checks and UI display stay synced!
		for (int i = 0; i < s; i++) {
			bodies[i]->m_forVec = bodies[i]->m_accVec * bodies[i]->m_Mass;
		}
	}
	// ── Pure force helper — reads positions from argument, does NOT touch body state ──
	static void computeAccel(
		const std::vector<vectorP>&               pos,
		std::vector<vectorP>&                     a_out,
		const std::vector<std::unique_ptr<Body>>& bodies)
	{
		const int s = (int)bodies.size();
		constexpr double eps = 0.1;

		for (int i = 0; i < s; i++) a_out[i] = vectorP(0, 0);

		for (int i = 0; i < s - 1; i++) {
			for (int j = i + 1; j < s; j++) {
				vectorP r  = pos[j] - pos[i];
				double  r2 = r.magSq() + eps * eps;
				double  r3 = r2 * std::sqrt(r2);
				if (bodies[i]->movability) a_out[i] += r * (G * bodies[j]->m_Mass / r3);
				if (bodies[j]->movability) a_out[j] -= r * (G * bodies[i]->m_Mass / r3);
			}
		}
	}


	// ──────────────────────────────────────────────────────────────────────────
	//  Public API
	// ──────────────────────────────────────────────────────────────────────────

	// Shared theta.  Change here to tune accuracy vs speed globally.
	static constexpr double BH_THETA = 0.5;

	// ── Drop-in for computeAccel() — used inside moveRK45 ────────────────────
	// Same signature; swap "computeAccel" → "computeAccelBH" in moveRK45.
	void computeAccelBH(
	    const std::vector<vectorP>&               pos,
	    std::vector<vectorP>&                     a_out,
	    const std::vector<std::unique_ptr<Body>>& bodies,
	    double theta = BH_THETA)
	{
	    const int s = (int)bodies.size();
	    for (int i = 0; i < s; i++) a_out[i] = vectorP(0, 0);
	    if (s < 2) return;

	    std::vector<double> masses(s);
	    for (int i = 0; i < s; i++) masses[i] = bodies[i]->m_Mass;

	    int    root    = bh::build(pos, masses);
	    double theta_sq = theta * theta;

	    for (int i = 0; i < s; i++) {
	        if (!bodies[i]->movability) continue;
	        double ax = 0, ay = 0;
	        bh::walk(root, i, pos[i].icap, pos[i].jcap, 0, 0, theta_sq, ax, ay, nullptr, nullptr);
	        a_out[i] = vectorP(ax, ay);
	    }
	}

	// ── Drop-in for resolve() — used in moveVerlet / moveYoshida ─────────────
	// Swap "resolve(bodies)" → "resolveBarnesHut(bodies)" in those functions.
	void resolveBarnesHut(std::vector<std::unique_ptr<Body>>& bodies,
	                      double theta = BH_THETA)
	{
	    const int s = (int)bodies.size();
	    std::vector<vectorP> pos(s), a(s);
	    std::vector<double>  masses(s);
	    for (int i = 0; i < s; i++) {
	        pos[i]    = bodies[i]->m_posVec;
	        masses[i] = bodies[i]->m_Mass;
	    }

	    int    root     = bh::build(pos, masses);
	    double theta_sq = theta * theta;

	    for (int i = 0; i < s; i++) {
	        if (!bodies[i]->movability) continue;
	        double ax = 0, ay = 0;
	        bh::walk(root, i, pos[i].icap, pos[i].jcap, 0, 0, theta_sq, ax, ay, nullptr, nullptr);
	        bodies[i]->m_accVec = vectorP(ax, ay);
	        bodies[i]->m_forVec = bodies[i]->m_accVec * bodies[i]->m_Mass;
	    }
	}

	// ── Drop-in for resolveWithJerk() — used in moveHermite ──────────────────
	// Swap "resolveWithJerk(bodies)" → "resolveWithJerkBH(bodies)" there.
	// Also tracks centre-of-mass velocity per node so the Hermite jerk
	//   ȧ = G M [ v_rel/r³ − 3(v_rel·r)r/r⁵ ]
	// uses the approximate COM velocity of each accepted cell.
	void resolveWithJerkBH(std::vector<std::unique_ptr<Body>>& bodies,
	                       double theta = BH_THETA)
	{
	    const int s = (int)bodies.size();
	    std::vector<vectorP> pos(s), vel(s);
	    std::vector<double>  masses(s);
	    for (int i = 0; i < s; i++) {
	        pos[i]    = bodies[i]->m_posVec;
	        vel[i]    = bodies[i]->m_velVec;
	        masses[i] = bodies[i]->m_Mass;
	    }

	    // Build tree with velocity tracking (needed for jerk approximation)
	    int    root     = bh::build(pos, masses, &vel);
	    double theta_sq = theta * theta;

	    for (int i = 0; i < s; i++) {
	        if (!bodies[i]->movability) continue;
	        double ax = 0, ay = 0, jx = 0, jy = 0;
	        bh::walk(root, i,
	                 pos[i].icap, pos[i].jcap,
	                 vel[i].icap, vel[i].jcap,
	                 theta_sq, ax, ay, &jx, &jy);
	        bodies[i]->m_accVec  = vectorP(ax, ay);
	        bodies[i]->m_jerkVec = vectorP(jx, jy);
	        bodies[i]->m_forVec  = bodies[i]->m_accVec * bodies[i]->m_Mass;
	    }
	}

	// ── Drop-in FMM replacement for computeAccel / computeAccelBH (RK45) ────────
	void computeAccelFMM(const std::vector<vectorP>& pos,
						 std::vector<vectorP>& a_out,
						 const std::vector<std::unique_ptr<Body>>& bodies)
	{
		const int s = (int)bodies.size();
		std::vector<double> masses(s);
		for (int i = 0; i < s; ++i) masses[i] = bodies[i]->m_Mass;

		fmm::run_fmm(pos, masses, nullptr, a_out, nullptr, bodies);
	}

	// ── Drop-in FMM replacement for resolve / resolveBarnesHut (Verlet & Yoshida) ──
	void resolveFMM(std::vector<std::unique_ptr<Body>>& bodies)
	{
		const int s = (int)bodies.size();
		std::vector<vectorP> pos(s), a(s);
		std::vector<double> masses(s);
		for (int i = 0; i < s; ++i) {
			pos[i] = bodies[i]->m_posVec;
			masses[i] = bodies[i]->m_Mass;
		}

		fmm::run_fmm(pos, masses, nullptr, a, nullptr, bodies);

		for (int i = 0; i < s; ++i) {
			if (!bodies[i]->movability) continue;
			bodies[i]->m_accVec = a[i];
			bodies[i]->m_forVec = bodies[i]->m_accVec * bodies[i]->m_Mass;
		}
	}

	// ── Drop-in FMM replacement for resolveWithJerk / resolveWithJerkBH (Hermite) ──
	void resolveWithJerkFMM(std::vector<std::unique_ptr<Body>>& bodies)
	{
		const int s = (int)bodies.size();
		std::vector<vectorP> pos(s), vel(s), a(s), j(s);
		std::vector<double> masses(s);
		for (int i = 0; i < s; ++i) {
			pos[i] = bodies[i]->m_posVec;
			vel[i] = bodies[i]->m_velVec;
			masses[i] = bodies[i]->m_Mass;
		}

		fmm::run_fmm(pos, masses, &vel, a, &j, bodies);

		for (int i = 0; i < s; ++i) {
			if (!bodies[i]->movability) continue;
			bodies[i]->m_accVec  = a[i];
			bodies[i]->m_jerkVec = j[i];
			bodies[i]->m_forVec  = bodies[i]->m_accVec * bodies[i]->m_Mass;
		}
	}

	void moveVerlet(std::vector<std::unique_ptr<Body>>& bodies)
	{
		resolve(bodies);
		int s = bodies.size();
		std::vector<vectorP> oldacc(s);
		double dtb2 = dt / 2;
		double dt2b2 = dt * dtb2;
		for (int i = 0; i < s; i++)
		{
			if(bodies[i]->movability != false)
			{
				vectorP temp = bodies[i]->m_accVec;
				bodies[i]->m_posVec += /*bodies[i]->m_posVec*/  bodies[i]->m_velVec * dt + (temp * dt2b2);
				oldacc[i] = temp;
			}

		}
		resolve(bodies);
		for (int i = 0; i < s; i++)
		{
			if (bodies[i]->movability != false)
			{
				bodies[i]->m_velVec += (oldacc[i] + bodies[i]->m_accVec) * dtb2;
			}
		}
		
	}

	void moveYoshida(std::vector<std::unique_ptr<Body>>& bodies)
	{
		int s = bodies.size();

		// 4th-order Yoshida constants
		const double c[4] = {  0.6756035959798289, -0.1756035959798289, -0.1756035959798289,  0.6756035959798289 };
		const double d[3] = {  1.3512071919596578, -1.7024143839193156,  1.3512071919596578 };

		for (int step = 0; step < 4; step++)
		{
			// 1. DRIFT: Update positions using current velocity
			for (int i = 0; i < s; i++)
			{
				if (bodies[i]->movability)
				{
					bodies[i]->m_posVec += bodies[i]->m_velVec * (c[step] * dt);
				}
			}

			// 2. KICK: Calculate new forces and update velocities
			// (Sub-step 4 is drift-only, so we stop after 3 kicks)
			if (step < 3)
			{
				resolve(bodies); // Re-evaluate accelerations at the new positions

				for (int i = 0; i < s; i++)
				{
					if (bodies[i]->movability)
					{
						bodies[i]->m_velVec += bodies[i]->m_accVec * (d[step] * dt);
					}
				}
			}
		}
	}

		// Helper function to resolve both Acceleration AND Jerk at current positions/velocities
// Helper function to resolve both Acceleration AND Jerk at current positions/velocities
// Helper function to resolve both Acceleration AND Jerk at current positions/velocities

	// 4th-Order Hermite Predictor-Corrector Integrator (PECE)
	void moveHermite(std::vector<std::unique_ptr<Body>>& bodies, double& dt)
	{
		int s = bodies.size();
		resolveWithJerk(bodies);

		std::vector<vectorP> x_old(s), v_old(s), a_old(s), j_old(s);
		std::vector<vectorP> x_pred(s), v_pred(s);

		double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, dt5 = dt4*dt;

		// PREDICT
		for (int i = 0; i < s; i++) {
			if (!bodies[i]->movability) continue;
			x_old[i] = bodies[i]->m_posVec;
			v_old[i] = bodies[i]->m_velVec;
			a_old[i] = bodies[i]->m_accVec;
			j_old[i] = bodies[i]->m_jerkVec;

			x_pred[i] = x_old[i] + v_old[i]*dt + a_old[i]*(0.5*dt2) + j_old[i]*(dt3/6.0);
			v_pred[i] = v_old[i] + a_old[i]*dt  + j_old[i]*(0.5*dt2);

			bodies[i]->m_posVec = x_pred[i];
			bodies[i]->m_velVec = v_pred[i];
		}

		resolveWithJerk(bodies);

		// CORRECT + compute Aarseth dt
		const double eta = 0.02;
		double dt_candidate = 2.0 * dt;  // max allowed growth

		for (int i = 0; i < s; i++) {
			if (!bodies[i]->movability) continue;

			vectorP a_new = bodies[i]->m_accVec;
			vectorP j_new = bodies[i]->m_jerkVec;

			vectorP snap    = ((a_old[i] - a_new)*-6.0 - (j_old[i]*4.0 + j_new*2.0)*dt) * (1.0/dt2);
			vectorP crackle = ((a_old[i] - a_new)*12.0 + (j_old[i] + j_new)*(6.0*dt))   * (1.0/dt3);

			bodies[i]->m_posVec = x_pred[i] + snap*(dt4/24.0) + crackle*(dt5/120.0);
			bodies[i]->m_velVec = v_pred[i] + snap*(dt3/6.0)  + crackle*(dt4/24.0);

			// Aarseth criterion
			double a0 = a_old[i].mag();
			double a1 = j_old[i].mag();
			double a2 = snap.mag();
			double a3 = crackle.mag();

			double denom = a1*a3 + a2*a2;
			if (denom > 1e-30) {
				double dt_i = eta * std::sqrt((a0*a2 + a1*a1) / denom);
				dt_candidate = std::min(dt_candidate, dt_i);
			}
		}

		dt = std::max(dt_candidate, 1e-7);  // floor prevents dt → 0 on singular configs
	}


    // ── Dormand-Prince RK45 with error-controlled adaptive step ──
    // Accepts/rejects internally (no retry needed by caller).
    // On exit: global dt = h suggested for the NEXT step.
    // Returns: h that was actually used this step → add to your phys_time accumulator.
    double moveRK45(std::vector<std::unique_ptr<Body>>& bodies)
    {
        const int s = (int)bodies.size();

        // Butcher tableau (DOPRI5)
        // A[st][j] = coefficient for k_{j+1} when building stage st+2 (0-indexed)
        static constexpr double A[6][6] = {
            /*k2*/ { 1.0/5.0,          0,               0,              0,              0,              0         },
            /*k3*/ { 3.0/40.0,         9.0/40.0,        0,              0,              0,              0         },
            /*k4*/ { 44.0/45.0,       -56.0/15.0,       32.0/9.0,       0,              0,              0         },
            /*k5*/ { 19372.0/6561.0,  -25360.0/2187.0,  64448.0/6561.0,-212.0/729.0,   0,              0         },
            /*k6*/ { 9017.0/3168.0,    -355.0/33.0,     46732.0/5247.0, 49.0/176.0,   -5103.0/18656.0, 0         },
            /*k7*/ { 35.0/384.0,        0.0,            500.0/1113.0,  125.0/192.0,   -2187.0/6784.0,  11.0/84.0 }
        };
        // e[j] = b5[j] − b4[j]: DOPRI5 error coefficients
        static constexpr double E[7] = {
             71.0/57600.0,  0.0, -71.0/16695.0,
             71.0/1920.0,  -17253.0/339200.0,
             22.0/525.0,   -1.0/40.0
        };

        constexpr double RTOL    = 1e-6;
        constexpr double ATOL    = 1e-9;
        constexpr double SAFETY  = 0.9;
        constexpr double MAX_FAC = 5.0;
        constexpr double MIN_FAC = 0.2;
        constexpr double DT_MIN  = 1.0 / 50000.0;
        constexpr double DT_MAX  = 1.0 / 30.0;

        // Snapshot of (x0, v0) — never modified across retries
        std::vector<vectorP> x0(s), v0(s);
        for (int i = 0; i < s; i++) {
            x0[i] = bodies[i]->m_posVec;
            v0[i] = bodies[i]->m_velVec;
        }

        // Stage buffers: kx[j] = velocity at stage j, kv[j] = acceleration at stage j
        std::vector<std::vector<vectorP>> kx(7, std::vector<vectorP>(s));
        std::vector<std::vector<vectorP>> kv(7, std::vector<vectorP>(s));

        // k1 depends only on (x0,v0), not on h — compute once outside the retry loop
        for (int i = 0; i < s; i++) kx[0][i] = v0[i];
        //computeAccel(x0, kv[0], bodies);
		computeAccelFMM(x0, kv[0], bodies);
		//computeAccelBH(x0, kv[0], bodies);

        double h = dt;

        for (;;) {   // retry loop: repeats only on step rejection
            std::vector<vectorP> xs(s), vs(s), as(s);

            // Stages k2..k7
            // After st=6: xs = x5 (5th-order solution), vs = v5  [FSAL: A[5] == b5[0..5]]
            for (int st = 1; st <= 6; st++) {
                for (int i = 0; i < s; i++) {
                    if (!bodies[i]->movability) { xs[i] = x0[i]; vs[i] = v0[i]; continue; }
                    xs[i] = x0[i];
                    vs[i] = v0[i];
                    for (int j = 0; j < st; j++) {
                        xs[i] += kx[j][i] * (h * A[st-1][j]);
                        vs[i] += kv[j][i] * (h * A[st-1][j]);
                    }
                    kx[st][i] = vs[i];  // dx/dt at this stage point
                }
                //computeAccel(xs, as, bodies);
                computeAccelFMM(xs, as, bodies);
            	//computeAccelBH(x0, kv[0], bodies);

                for (int i = 0; i < s; i++) kv[st][i] = as[i];  // dv/dt at this stage
            }
            // xs = x5, vs = v5, kv[6] = a(x5) after the loop above

            // Error estimate: err_component = h * Σ_j E[j] * k_j
            double err_sq = 0.0;
            int    n_dof  = 0;
            for (int i = 0; i < s; i++) {
                if (!bodies[i]->movability) continue;
                vectorP ex(0,0), ev(0,0);
                for (int j = 0; j < 7; j++) {
                    ex += kx[j][i] * (h * E[j]);
                    ev += kv[j][i] * (h * E[j]);
                }
                // Mixed absolute/relative scale: sc = ATOL + RTOL * max(|y0|, |y1|)
                auto sqsc = [&](double e_c, double y0_c, double y1_c) {
                    double sc = ATOL + RTOL * std::max(std::abs(y0_c), std::abs(y1_c));
                    return (e_c / sc) * (e_c / sc);
                };
                err_sq += sqsc(ex.icap, x0[i].icap, xs[i].icap);
                err_sq += sqsc(ex.jcap, x0[i].jcap, xs[i].jcap);
                err_sq += sqsc(ev.icap, v0[i].icap, vs[i].icap);
                err_sq += sqsc(ev.jcap, v0[i].jcap, vs[i].jcap);
                n_dof += 4;
            }
            double err = (n_dof > 0) ? std::sqrt(err_sq / n_dof) : 0.0;

            if (err <= 1.0 || h <= DT_MIN) {
                // ── Accept ──
                for (int i = 0; i < s; i++) {
                    bodies[i]->m_posVec = xs[i];
                    bodies[i]->m_velVec = vs[i];
                    bodies[i]->m_accVec = kv[6][i];                        // FSAL: exact a at x5
                    bodies[i]->m_forVec = bodies[i]->m_accVec * bodies[i]->m_Mass;
                }
                double fac = (err > 1e-15) ? SAFETY * std::pow(1.0/err, 0.2) : MAX_FAC;
                fac        = std::clamp(fac, MIN_FAC, MAX_FAC);
                double h_used = h;
                dt = std::clamp(h * fac, DT_MIN, DT_MAX);
                return h_used;
            }

            // ── Reject: shrink h, k1 is still valid ──
            double fac = std::max(MIN_FAC, SAFETY * std::pow(1.0/err, 0.25));
            h = std::max(DT_MIN, h * fac);
        }
    }
}

struct BodyInput
{
	double x, y;
	double xv, yv;

	vectorP pos;
	vectorP vel;

	double mass, radius;
	int movable; //if its kept bool and a value other than 0 ir 1 is given it goes haywire , atleast for my poor code
};

BodyInput getValBod()
{
	BodyInput temp;

	float x;
	std::cout << "Position Coord (X) :";
	std::cin >> x;

	float y;
	std::cout << "Position Coord (Y) :";
	std::cin >> y;

	temp.pos = vectorP(x, y);

	std::cout << "Velocity Vec (X):";
	std::cin >> x;

	std::cout << "Velocity Vec (Y):";
	std::cin >> y;

	temp.vel = vectorP(x, y);

	do
	{
		std::cout << "Mass :";
		std::cin >> temp.mass;
	} while (temp.mass <= 0);

	do
	{
		std::cout << "Radius :";
		std::cin >> temp.radius;
	} while (temp.radius <= 0); 

	do
	{
		std::cout << "Movability (0/1):";
		std::cin >> temp.movable;
	} while (temp.movable < 0 || temp.movable > 1); //!= doesnt work because then both need to be satisfied which can nevver happen

	return temp;
}

void getValArr(std::vector<std::unique_ptr<Body>>& bodies)
{
	int bsize = bodies.size();
	for (int i = 0; i < bsize; i++)
	{
		LOG("["<<i<<"]");
		bodies[i]->GetVal();
	}
}

namespace create
{
	void bodyAdd(std::vector<std::unique_ptr<Body>>& bodies)
	{
		BodyInput temp = getValBod();

		auto bod = std::make_unique<Body>(temp.mass, temp.radius, temp.movable, temp.pos, temp.vel);
		bodies.push_back(std::move(bod));
	}
	/*std::unique_ptr<Body> bodyExtract(BodyInput temp)
	{
		return(std::make_unique<Body>(temp.mass, temp.radius, temp.movable, vectorP(temp.x, temp.y), vectorP(temp.xv, temp.yv)));
	}*/
}

int main()
{
	using namespace std::literals::chrono_literals;
	using clock = std::chrono::steady_clock;

	std::vector<std::unique_ptr<Body>> bodys;
	std::vector<std::unique_ptr<Body>> delBods;
	std::vector<std::vector<std::unique_ptr<Body>>> colPairs;
	std::vector<std::vector<Body*>> colClusters;
	std::vector<std::vector<Body>> Clusters;

	int operation;

	do

	{
		std::cout << "0:Exit\n1:Add Body\n2:Edit Body\n3:Delete Body\n4:Run\n5:View\n-----------------\n Choose:";
		std::cin >> operation;


		if (operation == 1)
		{

			LOG("-----")
			create::bodyAdd(bodys);
		}

		if (operation == 2)
		{
			LOG("-----")
			getValArr(bodys);
			int bsize = bodys.size();

			int b;
			do
			{
				std::cout << "Choose:";
				std::cin >> b;
			} while (b>bsize || b<0);

			Body& fn = *bodys[b]; //reference to a body fn = fornow??? i guess
			BodyInput temp = getValBod();
			fn.m_posVec = temp.pos;
			fn.m_velVec = temp.vel;
			fn.m_Mass = temp.mass;
			fn.m_radius = temp.radius;
			fn.movability = temp.movable; 

		}

		if (operation == 3)
		{
			vectorP vector(3,4);
			vectorP vector2(6, 8);
			vectorP vector3(2,-2);

			auto star = std::make_unique<Body>(1000000000000.0f, 1.0f, true, vector2);
			auto perf = std::make_unique<Body>(1000.0f, 0.2f, true, vector, vector3);

			bodys.push_back(std::move(star));
			bodys.push_back(std::move(perf));

			/*LOG("-----")
			getValArr(bodys);
			int bsize = bodys.size();

			int b;
			do
			{
				std::cout << "Choose:";
				std::cin >> b;
			} while (b >= bsize || b < 0);

			delBods.push_back(std::move(bodys[b]));
			bodys.erase(bodys.begin() + b);*/
		}

		if (operation == 5)
		{

			int moga;
			std::cout << "Choose:";
			std::cin >> moga;

			if (moga == 1)
			{
				// --- 3. THE FIGURE-8 ORBIT (SCALED FOR REAL G) ---
				float mass_fig8 = 1000000000000.0f; // 1 Trillion kg
				float rad_fig8 = 0.5f;

				// Exact starting positions
				vectorP pos1(0.97000436f, -0.24308753f);
				vectorP pos2(-0.97000436f, 0.24308753f);
				vectorP pos3(0.0f, 0.0f);

				// Velocities properly scaled by sqrt(G * M) -> multiplier is ~8.1696389
				vectorP vel1(3.808715f, 3.532270f);
				vectorP vel2(3.808715f, 3.532270f);
				vectorP vel3(-7.617430f, -7.064540f);

				auto figA = std::make_unique<Body>(mass_fig8, rad_fig8, true, pos1, vel1);
				auto figB = std::make_unique<Body>(mass_fig8, rad_fig8, true, pos2, vel2);
				auto figC = std::make_unique<Body>(mass_fig8, rad_fig8, true, pos3, vel3);

				bodys.push_back(std::move(figA));
				bodys.push_back(std::move(figB));
				bodys.push_back(std::move(figC));
			}

			if (moga == 2)
			{

				// --- 2. HIGHLY ECCENTRIC ORBIT (STRESS TEST) ---
				float mass_sun = 100000000000000.0f; // 100 Trillion kg
				float mass_planet = 1000.0f;         // 1000 kg (negligible)

				// Sun at center, planet starts at "periapsis" (closest approach)
				vectorP pos_sun(0.0f, 0.0f);
				vectorP pos_planet(10.0f, 0.0f); // 10 units away

				// Sun is stationary. Planet is moving extremely fast on the Y axis
				vectorP vel_sun(0.0f, 0.0f);
				vectorP vel_planet(0.0f, 34.65f); // Eccentricity = 0.8

				// Note: movability for sun is set to 'false' so it stays pinned
				auto sun = std::make_unique<Body>(mass_sun, 5.0f, true, pos_sun, vel_sun);
				auto planet = std::make_unique<Body>(mass_planet, 0.5f, true, pos_planet, vel_planet);

				bodys.push_back(std::move(sun));
				bodys.push_back(std::move(planet));
			}

			if (moga == 4)
			{
				// --- 1. PERFECT 2-BODY CIRCULAR ORBIT ---
				// Mass = 1 trillion kg.
				float mass_binary = 1000000000000.0f;
				float radius_binary = 1.0f;

				// Placed 10 units away from the center on the X-axis
				vectorP posA(10.0f, 0.0f);
				vectorP posB(-10.0f, 0.0f);

				// Scaled orbital velocity to perfectly balance G = 6.6743e-11
				// V = sqrt((G * M) / (4 * r)) = 1.29173
				vectorP velA(0.0f, 1.29173f);
				vectorP velB(0.0f, -1.29173f);

				auto starA = std::make_unique<Body>(mass_binary, radius_binary, true, posA, velA);
				auto starB = std::make_unique<Body>(mass_binary, radius_binary, true, posB, velB);

				bodys.push_back(std::move(starA));
				bodys.push_back(std::move(starB));

			}
			 if (moga == 5)
			 {
			 	vectorP vector(3, 4);
			 	vectorP vector2(6, 8);
			 	vectorP vector4(2, -2);

			 	auto star = std::make_unique<Body>(1000000000000.0f, 1.0f, true, vector2);
			 	auto perf = std::make_unique<Body>(10.0f, 0.1f, true, vector ,vector4);

			 	bodys.push_back(std::move(star));
			 	bodys.push_back(std::move(perf));
			 }

			if (moga == 6)
			{
				vectorP vector1(3, 4);
				vectorP vector2(4, 4);
				vectorP vector4(3, 5);
				vectorP vector5(5, 4);
				vectorP vector6(4, 6);
				vectorP vector7(6, -6);

				auto perf1 = std::make_unique<Body>(10.0f, 1.0f, true, vector1);
				auto perf2 = std::make_unique<Body>(10.0f, 1.0f, true, vector2);
				auto perf4 = std::make_unique<Body>(10.0f, 1.0f, true, vector4);
				auto perf5 = std::make_unique<Body>(10.0f, 1.0f, true, vector5);
				auto perf6 = std::make_unique<Body>(10.0f, 1.0f, true, vector6);
				auto perf7 = std::make_unique<Body>(10.0f, 1.0f, true, vector7);

				bodys.push_back(std::move(perf1));
				bodys.push_back(std::move(perf2));
				bodys.push_back(std::move(perf4));
				bodys.push_back(std::move(perf5));
				bodys.push_back(std::move(perf6));
				bodys.push_back(std::move(perf7));
			}

			if ( moga == 7)
			{
				vectorP vector(3, 3);
				vectorP vector2(6, 6);
				vectorP vector4(-1, -1);

				vectorP vector1(17, 17);
				vectorP vector21(23, 17.5);
				vectorP vector41(11, 11);

				vectorP vector11(10, 8);
				vectorP vector211(1, 17.5);
				vectorP vector411(20, 0);

				vectorP vector0(3, 3);
				vectorP vector20(6, 6);
				vectorP vector40(-1, -1);

				auto star = std::make_unique<Body>(1000000000000.0f, 1.0f, true, vector);
				auto perf = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector2,vector20);
				auto nig = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector4,vector40);


				auto star1 = std::make_unique<Body>(1000000000000.0f, 1.0f, true, vector1);
				auto perf1 = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector21,vector0);
				auto nig1 = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector41, vector40);

				auto star11 = std::make_unique<Body>(1000000000000.0f, 1.0f, true, vector11, vector40);
				auto perf11 = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector211);
				auto nig11 = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector411);

				bodys.push_back(std::move(star));
				bodys.push_back(std::move(perf));
				bodys.push_back(std::move(nig));

				bodys.push_back(std::move(star1));
				bodys.push_back(std::move(perf1));
				bodys.push_back(std::move(nig1));

				bodys.push_back(std::move(star11));
				bodys.push_back(std::move(perf11));
				bodys.push_back(std::move(nig11));

			}



			LOG("-----")
			LOG("Alive\n------------")
				for (int i = 0; i < bodys.size(); i++)
				{
					bodys[i]->GetVal();
				}
			LOG("Dead\n------------");

			for (auto& pair : colPairs)
			{
				int psize = pair.size();
				if (psize >= 2)
				{
					for (int i = 0; i < psize; i++)
					{
						pair[i]->GetVal();
					}
				}
			}
			LOG("Deleted\n-------------");
			for (int i = 0; i < delBods.size(); i++)
			{
				delBods[i]->GetVal();
			}
		}
		
		/*vectorP vectorN(0, 0);
		vectorP poso(14, 8);*/


		
		/*auto z = std::make_unique<Body>(1000000000000.0f, 0.1f, false, poso);

		
		bodys.push_back(std::move(z));*/

		if (operation == 4)
		{
			LOG("-----")
			float dur;
			do
			{
				std::cout << "Runtime:";
				std::cin >> dur;

			}while ( dur <= 0);

			float noofnd = (1/dt) * dur;

			int stat;
			do
			{
				std::cout << "Grid / Raw Data / No data ? (0/1/2) :";
				std::cin >> stat;
			} while (stat < 0 || stat > 5);

			int Draw;
			do
			{
				std::cout << "Draw ? (0/1)";
				std::cin >> Draw;

			}while ( Draw > 1 || Draw < 0 );

			int fps;
			if(stat==0)
			{
				std::cout << "Per how many frames ? (calculated at 120fps):";
				std::cin >> fps;
			}



			std::vector<std::unique_ptr<Body>> bodOs;
			bodOs.reserve(bodys.size());
			for (const auto& b : bodys) // reference is important as otherwise itll try to copy a unique_ptr into b
				bodOs.push_back(b ? b->clone() : nullptr);

			std::vector<vectorP> posOs(bodys.size());


			for (int i = 0; i < bodys.size(); i++)
			{
				posOs[i] = bodys[i]->m_posVec;
			}  //i dont need this becasue of my poor design choices , at start every posOs = 0 and since
			// i check every posOs for every body each posOs is valid because its also checked for 0 wait WTFF,nvm
			// i needed that because what it i dont have a body at 0,0  


			std::vector<char> livyur(21, '.');
			std::vector<std::vector<char>> livyud(21, livyur);

			//std::vector<char> livyurc(21, '.'); 
			//std::vector<std::vector<char>> livyudc(21, livyurc);

			InitWindow(1280 , 720 , "oto");
			SetTargetFPS(fps);

			Camera3D camera = { 0 };
			camera.position   = { 0.0f, 50.0f, 0.0f };  // directly above the scene, looking straight down
			camera.target     = { 0.0f, 0.0f, 0.0f };   // looking down at the origin
			camera.up         = { 0.0f, 0.0f, -1.0f };  // see note below -- this can't be (0,1,0) anymore
			camera.fovy       = 40.0f;                  // now means "view height in world units," not degrees
			camera.projection = CAMERA_PERSPECTIVE;    // flat 2D-style view, no perspective foreshortening

			const Vector3 planeCenter = { 0.0f, 0.0f, 0.0f }; // World-space center of the plane
			const Vector2 planeSize   = { 8.0f, 4.5f };       // Width (X) and length (Z) of the plane

			int rerun = 1;

			double ogKE ,ogPE ,ogE , ogangP ;
			vectorP oglinP;

			float RENDER_SCALE = 0.25f;

			eos( ogKE, ogPE, ogE , bodys);
			linearP(oglinP , bodys);
			angularP(ogangP , bodys);



			if (stat == 4)
			{
				do
				{

					double KE , PE , E , Edifn, angP , angPdiffn;
					vectorP linP, linPdiffn;

					bodys.clear();
					bodys.reserve(bodOs.size());
					for (const auto& b : bodOs)
						bodys.push_back(b ? b->clone() : nullptr);

					int frame = 0;
					int hmframe = 1;
					bool quit = false;

					std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

					if (Draw == 1)
					{
						BeginDrawing();
						ClearBackground(RAYWHITE);

						BeginMode3D(camera);

						for (int i = 0 ;  i < bodys.size(); i++)
						{
							std::unique_ptr temu = bodys[i]->clone();
							float temx = temu->m_posVec.icap;
							float temy = temu->m_posVec.jcap;
							float temr = temu->m_radius;
							DrawSphere(Vector3(temx * RENDER_SCALE,0, temy * RENDER_SCALE) , temr * RENDER_SCALE , RED);

						}
						DrawGrid(100,  RENDER_SCALE);
						EndMode3D();								DrawText(TextFormat("KE : %f" , KE), 0, 0, 20 , BLACK);
						DrawText(TextFormat("PE : %f" , PE), 0, 20, 20 , BLACK);
						DrawText(TextFormat("E : %f" , E), 0, 40, 20 , BLACK);
						DrawText(TextFormat("ogKE : %f" , ogKE), 0, 60, 20 , BLACK);
						DrawText(TextFormat("ogPE : %f" , ogPE), 0, 80, 20 , BLACK);
						DrawText(TextFormat("ogE : %f" , ogE), 0, 100, 20 , BLACK);
						DrawText(TextFormat("Edif : %f" , Edifn), 0, 120, 20 , BLACK);
						DrawText(TextFormat("ogangP : %f" , ogangP), 0, 140, 20 , BLACK);
						DrawText(TextFormat("angP : %f" , angP), 0, 160, 20 , BLACK);
						DrawText(TextFormat("oglinP :  %f i , %f j" , oglinP.icap , oglinP.jcap), 0, 180, 20 , BLACK);
						DrawText(TextFormat("linP :  %f i , %f j" , linP.icap , linP.jcap), 0, 200, 20 , BLACK);
						DrawText(TextFormat("oglinP : %f" , oglinP.mag()), 0, 220, 20 , BLACK);
						DrawText(TextFormat("linPP : %f" , linP.mag()), 0, 240, 20 , BLACK);
						DrawText(TextFormat("angPdiffn : %f" , angPdiffn), 0, 300, 20 , BLACK);
						DrawText(TextFormat("linPdiffn :  %f i , %f j" , linPdiffn.icap , linPdiffn.jcap), 0, 320, 20 , BLACK);
						DrawText(TextFormat("linPdiffn : %f" , linPdiffn.mag()), 0, 340, 20 , BLACK);
						DrawText(TextFormat("velocity :  %f i , %f j" , bodys[1]->m_velVec.icap ,  bodys[1]->m_velVec.jcap), 0, 400, 20 , BLACK);;
						DrawText(TextFormat("position :  %f i , %f j" , bodys[1]->m_posVec.icap ,  bodys[1]->m_posVec.jcap), 0, 420, 20 , BLACK);;
						DrawText(TextFormat("velocity :  %f i , %f j" , bodys[1]->m_forVec.icap ,  bodys[1]->m_forVec.jcap), 0, 440, 20 , BLACK);

						DrawText(TextFormat("dt : %f" , dt ) , 1000 , 20, 20 , RED);
						DrawText(TextFormat("velocity : %f" , bodys[1]->m_velVec.mag()), 0, 500, 20 , BLACK);
						DrawText(TextFormat("position : %f" , bodys[1]->m_posVec.mag()), 0, 520, 20 , BLACK);

						EndDrawing();
					}

					while (!WindowShouldClose() && !quit && !bodys.empty())
					{

						std::cout << "[t = " << frame * dt << "s | frame" << frame << "] Steps ? ( Enter = 1 , q = quit) : ";
						std::string line;
						std::getline(std::cin , line);

						if (line == "q" || line == "Q")
						{
							quit = true;
							break;
						}
						else if (line.empty())
							hmframe = 1;
						else
						{
							try   { hmframe = std::max(1, std::stoi(line)); }
							catch (...) { hmframe = 1; }
						}

						double phys_time = 0.0f;
						//auto t0 = clock::now();


						for(int s = 0 ; s < hmframe && !bodys.empty() && !WindowShouldClose(); s++)
						{
							frame++;

							auto ks_pairs = ks_regularization::extractCloseEncounters(bodys);

							//physics::moveHermite(bodys, dt);
							//phys_time += physics::moveRK45(bodys);

							double step_taken = physics::moveRK45(bodys);
							phys_time += step_taken;

							ks_regularization::restoreCloseEncounters(bodys, ks_pairs, step_taken);

							eos(KE , PE , E , bodys);
							Edifn = E - ogE;

							linearP(linP , bodys);
							angularP(angP , bodys);

							linPdiffn = linP - oglinP;
							angPdiffn = angP - ogangP;

							//LOG("dt : " << dt);
							LOG("Net Ediffn : " << Edifn);
							LOG("Net linPdiffn : " << linPdiffn.mag());
							LOG("Net angPdiffn : " << angPdiffn);

							bodys[1]->GetVal();

							//auto colData = (physics::checkCol(bodys,colClusters));
							auto colData = (physics::checkColSpatialHash(bodys, colClusters));
							auto killed = std::move(colData.deadBodies);
							auto newClusters = std::move(colData.clusters);
							for (auto& c : newClusters)
								Clusters.push_back(std::move(c));

							if (!killed.empty())
							{
								colPairs.push_back(std::move(killed));
							}

							if (Draw == 1)
							{
								BeginDrawing();
								ClearBackground(RAYWHITE);

								BeginMode3D(camera);

								for (int i = 0 ;  i < bodys.size(); i++)
								{
									std::unique_ptr temu = bodys[i]->clone();
									float temx = temu->m_posVec.icap;
									float temy = temu->m_posVec.jcap;
									float temr = temu->m_radius;
									DrawSphere(Vector3(temx * RENDER_SCALE,0, temy * RENDER_SCALE) , temr * RENDER_SCALE , RED);

								}
								DrawGrid(1000,  RENDER_SCALE);
								EndMode3D();
								DrawText(TextFormat("KE : %f" , KE), 0, 0, 20 , BLACK);
								DrawText(TextFormat("PE : %f" , PE), 0, 20, 20 , BLACK);
								DrawText(TextFormat("E : %f" , E), 0, 40, 20 , BLACK);
								DrawText(TextFormat("ogKE : %f" , ogKE), 0, 60, 20 , BLACK);
								DrawText(TextFormat("ogPE : %f" , ogPE), 0, 80, 20 , BLACK);
								DrawText(TextFormat("ogE : %f" , ogE), 0, 100, 20 , BLACK);
								DrawText(TextFormat("Edif : %f" , Edifn), 0, 120, 20 , BLACK);
								DrawText(TextFormat("ogangP : %f" , ogangP), 0, 140, 20 , BLACK);
								DrawText(TextFormat("angP : %f" , angP), 0, 160, 20 , BLACK);
								DrawText(TextFormat("oglinP :  %f i , %f j" , oglinP.icap , oglinP.jcap), 0, 180, 20 , BLACK);
								DrawText(TextFormat("linP :  %f i , %f j" , linP.icap , linP.jcap), 0, 200, 20 , BLACK);
								DrawText(TextFormat("oglinP : %f" , oglinP.mag()), 0, 220, 20 , BLACK);
								DrawText(TextFormat("linPP : %f" , linP.mag()), 0, 240, 20 , BLACK);
								DrawText(TextFormat("angPdiffn : %f" , angPdiffn), 0, 300, 20 , BLACK);
								DrawText(TextFormat("linPdiffn :  %f i , %f j" , linPdiffn.icap , linPdiffn.jcap), 0, 320, 20 , BLACK);
								DrawText(TextFormat("linPdiffn : %f" , linPdiffn.mag()), 0, 340, 20 , BLACK);

								DrawText(TextFormat("velocity :  %f i , %f j" , bodys[1]->m_velVec.icap ,  bodys[1]->m_velVec.jcap), 0, 400, 20 , BLACK);;
								DrawText(TextFormat("position :  %f i , %f j" , bodys[1]->m_posVec.icap ,  bodys[1]->m_posVec.jcap), 0, 420, 20 , BLACK);;
								DrawText(TextFormat("velocity :  %f i , %f j" , bodys[1]->m_forVec.icap ,  bodys[1]->m_forVec.jcap), 0, 440, 20 , BLACK);


								DrawText(TextFormat("velocity : %f" , bodys[1]->m_velVec.mag()), 0, 500, 20 , BLACK);
								DrawText(TextFormat("position : %f" , bodys[1]->m_posVec.mag()), 0, 520, 20 , BLACK);

								DrawText(TextFormat("dt : %f" , dt) , 1000 , 20, 20 , RED);
								DrawText(TextFormat("phys_time : %f" , phys_time) , 1000 , 40, 20 , RED);

								EndDrawing();

								if (IsKeyPressed(KEY_SPACE)) break;
							}
							else
							{
								PollInputEvents();
								if (IsKeyPressed(KEY_SPACE)) break;  // ← interrupt without draw
							}
							//valinuxitexherea
							//these are the work ms times , comment out till log
							//auto t1 = clock::now();
							//auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
							//double dt_ms = dt * 1000000.0f;
							//LOG("work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n");
							//this is for the work ms , comment it out
							//auto t0 = clock::now(); //iteration

							/*if (frame == 1000)
							{
								auto t1 = clock::now();
								auto work_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
								LOG("work_ms = " << work_us << " dt_us = " << dur * 1000000 << "\n");
							}*/
						}
					}


					std::cout << "Rerun ? (0/1)";
					std::cin >> rerun;

					if (rerun == 0)
					{
						for (int i = 0 ; i < bodOs.size(); i++)
						{
							bodys[i]->GetVal();
						}
						break;
					}

					if (rerun == 1)
					{
						Clusters.clear();
						colPairs.clear();
					}

				} while (rerun == 1);
			}


			if (stat == 1 || stat == 0)
			{
				do
				{
					double KE;
					double PE;
					double E;

					while (!WindowShouldClose())
					{

						bodys.clear();
						bodys.reserve(bodOs.size());
						for (const auto& b : bodOs)
							bodys.push_back(b ? b->clone() : nullptr);

						std::chrono::duration<double> duration(dur);

						auto dt_duration = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));

						auto start = clock::now();
						auto end = start + duration;
						auto nextFrame = start;
						int frame = 0;

						while (clock::now() < end && bodys.size() > 0)
						{

							if (stat == 0)
							{
								for (int i = 0; i < bodys.size(); i++)
								{
									vectorP tbpv = bodys[i]->m_posVec.round();  //tbpv = temporary bodies postition vector

									if (tbpv.icap < 0 || tbpv.icap > 20 || tbpv.jcap < 0 || tbpv.jcap > 20)
									{
										tbpv = (0, 0); // AHHH ts so goated as its tbps in 0 its ovec is 0 and since
										//the coords dont match ovec 0,0 will still be "." ahahhaahah
									}

									posOs[i] = tbpv;
								}
							}

							frame++;
							auto t0 = clock::now(); //iteration

							if (Draw == 1)
							{
								const float RENDER_SCALE = 0.25f;

								BeginDrawing();
								ClearBackground(RAYWHITE);

								BeginMode3D(camera);

								for (int i = 0 ;  i < bodys.size(); i++)
								{
									std::unique_ptr temu = bodys[i]->clone();
									float temx = temu->m_posVec.icap;
									float temy = temu->m_posVec.jcap;
									float temr = temu->m_radius;
									DrawSphere(Vector3(temx * RENDER_SCALE,0, temy * RENDER_SCALE) , temr * RENDER_SCALE , RED);

								}
								DrawGrid(100,  RENDER_SCALE);
								EndMode3D();
								EndDrawing();
							}

							physics::moveYoshida(bodys);

							eos(KE , PE , E , bodys);

							double Edifn = E - ogE;

							LOG("Net Ediffn : " << Edifn);

							auto colData = (physics::checkCol(bodys,colClusters));
							auto killed = std::move(colData.deadBodies);
							auto newClusters = std::move(colData.clusters);
							for (auto& c : newClusters)
								Clusters.push_back(std::move(c));
							if (!killed.empty())
							{
								colPairs.push_back(std::move(killed));
							}
							if (stat == 0)
							{
								for (int i = 0; i < posOs.size(); i++)
								{
									vectorP Ovec = posOs[i];
									bool booly = false;
									for (int j = 0; j < bodys.size(); j++)
									{
										booly = (Ovec == bodys[j]->m_posVec.round());
										if (booly == true)
										{
											break;
										}
									}
									if (booly == false)
									{
										livyud[Ovec.jcap][Ovec.icap] = '.';
									}
									else
									{
										livyud[Ovec.jcap][Ovec.icap] = 'O';
									}
								}
							}

							if (stat == 1)
							{
								for (int i = 0; i < bodys.size(); i++)
								{
							//		bodys[i]->GetVal();
								}
							}

							if (stat == 0 && (frame % int((1/dt)/fps)) == 0)
							{
								drawGrid(livyud);
								LOG("----------------------------");
							}
		//valinuxitexherea
							//these are the work ms times , comment out till log
							//auto t1 = clock::now();
							//auto work_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
							//double dt_ms = dt * 1000000.0f;
							//LOG("work_ms = " << work_ms << " dt_ms = " << dt_ms << "\n");
		//this is for the work ms , comment it out
		//auto t0 = clock::now(); //iteration
							nextFrame += dt_duration;
							std::this_thread::sleep_until(nextFrame);

						}

						std::cout << "Rerun ? (0/1)";
						std::cin >> rerun;

						if (rerun == 0)
						{
							break;
						}

						if (rerun == 1)
						{
							Clusters.clear();
							colPairs.clear();
							std::vector<char> dots(21, '.');
							for (int i = 0; i < 21; i++)
							{
								livyud[i] = dots;
							}
						}

					}


				} while (rerun == 1);

			}

			if (stat == 2)
			{
				double KE;
				double PE;
				double E;

				do
				{
					auto t0 = clock::now();

					bodys.clear();
					bodys.reserve(bodOs.size());
					for (const auto& b : bodOs)
						bodys.push_back(b ? b->clone() : nullptr);

					int frame = 0;
					noofnd = 1000;
					while (frame < noofnd && bodys.size() > 0)
					{
						frame++;

						/*BeginDrawing();
						ClearBackground(RAYWHITE);

						BeginMode3D(camera);

						for (int i = 0 ;  i < bodys.size(); i++)
						{
							std::unique_ptr temu = bodys[i]->clone();
							float temx = temu->m_posVec.icap;
							float temy = temu->m_posVec.jcap;
							float temr = temu->m_radius;
							DrawSphere(Vector3(temx,0, temy) , temr , RED);

						}
						DrawGrid(50, 1.0f);
						EndMode3D();
						EndDrawing();*/


						physics::moveYoshida(bodys);


						eos(KE , PE , E , bodys);

						double Edifn = E - ogE ;

						LOG("Net Dif : " << Edifn);


						auto colData = (physics::checkCol(bodys,colClusters));
						auto killed = std::move(colData.deadBodies);
						auto newClusters = std::move(colData.clusters);
						for (auto& c : newClusters)
							Clusters.push_back(std::move(c));
						if (!killed.empty())
						{
							colPairs.push_back(std::move(killed));
						}
					}

					auto t1 = clock::now();
					auto work_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() ;
					LOG("work_ms = " << work_us << " dt_us = " << dur * 1000000 << "\n");


					if (Draw == 1)
					{
						const float RENDER_SCALE = 0.05f;

						BeginDrawing();
						ClearBackground(RAYWHITE);

						BeginMode3D(camera);

						for (int i = 0 ;  i < bodys.size(); i++)
						{
							std::unique_ptr temu = bodys[i]->clone();
							float temx = temu->m_posVec.icap;
							float temy = temu->m_posVec.jcap;
							float temr = temu->m_radius;
							DrawSphere(Vector3(temx * RENDER_SCALE,0, temy * RENDER_SCALE) , temr * RENDER_SCALE , RED);

						}
						DrawGrid(100,  RENDER_SCALE);
						EndMode3D();
						EndDrawing();
					}

					LOG("Alive\n--------------")
					for (int i = 0; i < bodys.size(); i++)
					{
						bodys[i]->GetVal();
					}

					LOG("Dead\n------------");
					for (auto& pair : colPairs)
					{
						int psize = pair.size();
						if (psize >= 2)
						{
							for (int i = 0; i < psize; i++)
							{
								pair[i]->GetVal();
							}
						}
					}
					LOG("Collisions\n--------------")
					for (int i = 0; i < Clusters.size(); i++)
					{
						LOG("Collision" << i)
						for (int j = 0 ; j< Clusters[i].size();j++)
						{
							Clusters[i][j].GetVal();
						}
					}

					std::cout << "Rerun ? (0/1)";
					std::cin >> rerun;

					if (rerun == 0)
					{
						break;
					}

				} while (rerun == 1);

			}

			CloseWindow();


			drawGrid(livyud);
			LOG("Alive\n--------------")
				for (int i = 0; i < bodys.size(); i++)
				{
					bodys[i]->GetVal();
				}
			LOG("Dead\n------------");

			for (auto& pair : colPairs)
			{
				int psize = pair.size();
				if (psize >= 2)
				{
					for (int i = 0; i < psize; i++)
					{
						pair[i]->GetVal();
					}
				}
			}
			LOG("Collisions\n--------------")
			{
				for (int i = 0; i < Clusters.size(); i++)
				{
					LOG("Collision" << i)
					for (int j = 0 ; j< Clusters[i].size();j++)
					{
						Clusters[i][j].GetVal();
					}
				}
			}


			LOG("Deleted\n-------------");
			for (int i = 0; i < delBods.size(); i++)
			{
				delBods[i]->GetVal();
			}

		}

		LOG("---------------")
	} while (operation != 0);

	std::cin.get();
}


void eos(double& KE , double& PE , double& E , std::vector<std::unique_ptr<Body>>& bodys)
{
	KE = 0.0f;
	PE = 0.0f;
	E = 0.0f;
	for (int i = 0 ; i < bodys.size(); i++)
	{
		KE += 0.5f * bodys[i]->m_Mass * bodys[i]->m_velVec.magSq();
	}

	double eps = 0.1;

	if (bodys.size() > 1)
	{
		for (int i = 0 ; i < bodys.size() - 1; i++)
		{
			auto& bodya = *bodys[i];
			for (int j = i+1 ; j < bodys.size(); j++)
			{
				auto& bodyb = *bodys[j];

				double distSq = (physics::displacement( bodya , bodyb)).magSq();
				double softenedDist = sqrt(distSq + (eps*eps));
				PE += (-1 * physics::G * bodya.m_Mass * bodyb.m_Mass)/softenedDist;
			}
		}
	}

	E = KE + PE;

	LOG("KE:" << KE);
	LOG("PE:" << PE);
	//LOG("E:" << E);
}


void linearP(vectorP& lP , std::vector<std::unique_ptr<Body>>& bodys)
{
	lP = vectorP(0.0f,0.0f);
	for (int i = 0 ; i < bodys.size(); i++)
	{
		lP += bodys[i]->lP();
	}
}

void angularP(double& aP , std::vector<std::unique_ptr<Body>>& bodys)
{
	aP = 0;
	for (int i = 0 ; i < bodys.size(); i++)
	{
		aP += bodys[i]->aP();
	}
}