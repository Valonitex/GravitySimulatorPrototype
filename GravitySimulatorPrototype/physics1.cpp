//hi
//slop aint bop

#include <filesystem>

#include "EndBrace.h"

double dt = (1.0f / 120.0f);

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

	// ─────────────────────────────────────────────────────────────────────────
	// computeAccRK
	//   Pure gravity helper: computes gravitational acceleration for each body
	//   at ARBITRARY positions supplied in `pos` (not the current body state).
	//   Does NOT read or write any Body field — safe to call mid-RK step.
	//
	//   Pinned bodies (movability == false) always receive {0,0} in out_acc so
	//   the integrator leaves their state unchanged, but they still contribute
	//   gravity to every movable body (they are sources, never sinks).
	//
	//   Uses the same softening ε = 0.1 as pull() so force magnitudes are
	//   consistent across all integrators.
	// ─────────────────────────────────────────────────────────────────────────
	void computeAccRK(
		const std::vector<std::unique_ptr<Body>>& bodies,
		const std::vector<vectorP>& pos,
		std::vector<vectorP>& out_acc)
	{
		const int    n   = static_cast<int>(bodies.size());
		const double eps = 0.1;   // same softening ε as pull()

		for (int i = 0; i < n; i++)
			out_acc[i] = vectorP(0.0, 0.0);

		for (int i = 0; i < n; i++) {
			for (int j = i + 1; j < n; j++) {
				vectorP r  = pos[j] - pos[i];           // r_j − r_i
				double  r2 = r.magSq() + eps * eps;     // softened |r|²
				double  r1 = std::sqrt(r2);
				double  r3 = r2 * r1;                   // |r|³ (softened)

				// a_i += G·mj·r / r³
				if (bodies[i]->movability)
					out_acc[i] += r * (G * bodies[j]->m_Mass / r3);

				// a_j -= G·mi·r / r³  (Newton's 3rd: direction flips)
				if (bodies[j]->movability)
					out_acc[j] -= r * (G * bodies[i]->m_Mass / r3);
			}
		}
	}

	// ─────────────────────────────────────────────────────────────────────────
	// moveRK45 — Dormand-Prince RK45 adaptive integrator
	//
	//   Advances the N-body system by one "logical" timestep using the
	//   classic embedded 4(5) Runge-Kutta pair (Dormand & Prince, 1980).
	//
	//   State:  y = (x, v)      ẏ = (v , a(x))
	//
	//   The method computes 6 force evaluations per accepted step; a 7th
	//   stage (k7) is evaluated at the new position and serves both as the
	//   FSAL (First Same As Last) seed and as the embedded 4th-order
	//   correction for the error estimate.  Step size is automatically
	//   shrunk or grown to keep the mixed absolute/relative error below tol.
	//
	//   Comparison with the other integrators
	//   ──────────────────────────────────────
	//   Verlet / Yoshida : symplectic, exact energy conservation on average,
	//                      but fixed step — can diverge on close encounters.
	//   Hermite PECE     : adaptive via Aarseth criterion, good for stellar
	//                      dynamics; not symplectic but very accurate for
	//                      smooth trajectories.
	//   RK45             : NOT symplectic (energy drifts slowly over many
	//                      orbits), but the adaptive error control makes it
	//                      the most reliable integrator for close passages,
	//                      collisional configurations, or stiff forcings.
	//
	//   Parameters
	//   ──────────
	//   bodies   Bodies to integrate.  Pinned bodies (movability == false) act
	//            as fixed gravitational sources; their state is never modified.
	//   dt       Current step size [in/out].  Updated for the next call.
	//            Pass the same variable each frame (just like moveHermite).
	//   tol      Dimensionless error tolerance.  Default 1e-6 gives ~6
	//            significant figures per step; raise to 1e-4 for speed,
	//            lower to 1e-8 for high-precision trajectories.
	//   dt_max   Hard ceiling on step size.  Default 1/30 s.
	// ─────────────────────────────────────────────────────────────────────────
	void moveRK45(std::vector<std::unique_ptr<Body>>& bodies,
	              double& dt,
	              double  tol    = 1e-6,
	              double  dt_max = 1.0 / 30.0)
	{
		const int n = static_cast<int>(bodies.size());
		if (n == 0) return;

		// ── Dormand-Prince Butcher tableau ────────────────────────────────────
		// a-coefficients (lower-triangular, only non-zero entries listed)
		constexpr double a21 = 1.0 / 5.0;

		constexpr double a31 = 3.0 / 40.0,          a32 = 9.0 / 40.0;

		constexpr double a41 = 44.0 / 45.0,          a42 = -56.0 / 15.0,
		                 a43 = 32.0 / 9.0;

		constexpr double a51 = 19372.0 / 6561.0,     a52 = -25360.0 / 2187.0,
		                 a53 = 64448.0 / 6561.0,     a54 = -212.0 / 729.0;

		constexpr double a61 = 9017.0 / 3168.0,      a62 = -355.0 / 33.0,
		                 a63 = 46732.0 / 5247.0,     a64 = 49.0 / 176.0,
		                 a65 = -5103.0 / 18656.0;

		// 5th-order output weights  (b2 = b7 = 0, omitted)
		constexpr double b1 = 35.0 / 384.0,      b3 = 500.0 / 1113.0,
		                 b4 = 125.0 / 192.0,      b5 = -2187.0 / 6784.0,
		                 b6 = 11.0 / 84.0;

		// Error coefficients  e_i = b_i^(5) − b_i^(4)    (e2 = 0, omitted)
		// Verified from Dormand-Prince (1980) Table 1.
		constexpr double e1 =  71.0 / 57600.0,    e3 = -71.0 / 16695.0,
		                 e4 =  71.0 / 1920.0,     e5 = -17253.0 / 339200.0,
		                 e6 =  22.0 / 525.0,      e7 = -1.0 / 40.0;

		// ── Step-size controller (Hairer & Wanner, §II.4) ────────────────────
		constexpr double safety   = 0.9;      // conservative safety factor
		constexpr double expo     = 1.0 / 5.0;  // ← 1/(p+1), p = 4 (lower order)
		constexpr double max_grow = 5.0;      // cap step growth to 5×
		constexpr double min_shrk = 0.2;     // floor step shrink to 0.2×

		// ── Snapshot current state ────────────────────────────────────────────
		std::vector<vectorP> x0(n), v0(n);
		for (int i = 0; i < n; i++) {
			x0[i] = bodies[i]->m_posVec;
			v0[i] = bodies[i]->m_velVec;
		}

		// ── Stage storage ─────────────────────────────────────────────────────
		// kNp[i]  d(pos)/dt = velocity   at stage N for body i
		// kNa[i]  d(vel)/dt = accel      at stage N for body i
		// Pinned bodies always get {0,0} in both, keeping their state frozen.
		std::vector<vectorP>
			k1p(n), k1a(n),
			k2p(n), k2a(n),
			k3p(n), k3a(n),
			k4p(n), k4a(n),
			k5p(n), k5a(n),
			k6p(n), k6a(n),
			k7p(n), k7a(n),
			sp(n),  sv(n);   // scratch stage positions / velocities

		bool accepted = false;

		while (!accepted)
		{
			// ── k1 — derivatives at (x0, v0) ─────────────────────────────────
			computeAccRK(bodies, x0, k1a);
			for (int i = 0; i < n; i++)
				k1p[i] = bodies[i]->movability ? v0[i] : vectorP(0.0, 0.0);

			// ── k2 — stage at c2 = 1/5 ───────────────────────────────────────
			for (int i = 0; i < n; i++) {
				sp[i] = x0[i] + k1p[i] * (a21 * dt);
				sv[i] = v0[i] + k1a[i] * (a21 * dt);
				k2p[i] = bodies[i]->movability ? sv[i] : vectorP(0.0, 0.0);
			}
			computeAccRK(bodies, sp, k2a);

			// ── k3 — stage at c3 = 3/10 ──────────────────────────────────────
			for (int i = 0; i < n; i++) {
				sp[i] = x0[i] + k1p[i] * (a31 * dt) + k2p[i] * (a32 * dt);
				sv[i] = v0[i] + k1a[i] * (a31 * dt) + k2a[i] * (a32 * dt);
				k3p[i] = bodies[i]->movability ? sv[i] : vectorP(0.0, 0.0);
			}
			computeAccRK(bodies, sp, k3a);

			// ── k4 — stage at c4 = 4/5 ───────────────────────────────────────
			for (int i = 0; i < n; i++) {
				sp[i] = x0[i] + k1p[i] * (a41 * dt) + k2p[i] * (a42 * dt)
				              + k3p[i] * (a43 * dt);
				sv[i] = v0[i] + k1a[i] * (a41 * dt) + k2a[i] * (a42 * dt)
				              + k3a[i] * (a43 * dt);
				k4p[i] = bodies[i]->movability ? sv[i] : vectorP(0.0, 0.0);
			}
			computeAccRK(bodies, sp, k4a);

			// ── k5 — stage at c5 = 8/9 ───────────────────────────────────────
			for (int i = 0; i < n; i++) {
				sp[i] = x0[i] + k1p[i] * (a51 * dt) + k2p[i] * (a52 * dt)
				              + k3p[i] * (a53 * dt) + k4p[i] * (a54 * dt);
				sv[i] = v0[i] + k1a[i] * (a51 * dt) + k2a[i] * (a52 * dt)
				              + k3a[i] * (a53 * dt) + k4a[i] * (a54 * dt);
				k5p[i] = bodies[i]->movability ? sv[i] : vectorP(0.0, 0.0);
			}
			computeAccRK(bodies, sp, k5a);

			// ── k6 — stage at c6 = 1 (the 5th-order endpoint) ────────────────
			for (int i = 0; i < n; i++) {
				sp[i] = x0[i] + k1p[i] * (a61 * dt) + k2p[i] * (a62 * dt)
				              + k3p[i] * (a63 * dt) + k4p[i] * (a64 * dt)
				              + k5p[i] * (a65 * dt);
				sv[i] = v0[i] + k1a[i] * (a61 * dt) + k2a[i] * (a62 * dt)
				              + k3a[i] * (a63 * dt) + k4a[i] * (a64 * dt)
				              + k5a[i] * (a65 * dt);
				k6p[i] = bodies[i]->movability ? sv[i] : vectorP(0.0, 0.0);
			}
			computeAccRK(bodies, sp, k6a);

			// ── 5th-order solution  (b2 = 0 → k2 terms absent) ───────────────
			std::vector<vectorP> x5(n), v5(n);
			for (int i = 0; i < n; i++) {
				// Accumulate with += to avoid chaining operator+ on temporaries
				vectorP dp = k1p[i] * b1;
				dp += k3p[i] * b3;  dp += k4p[i] * b4;
				dp += k5p[i] * b5;  dp += k6p[i] * b6;

				vectorP dv = k1a[i] * b1;
				dv += k3a[i] * b3;  dv += k4a[i] * b4;
				dv += k5a[i] * b5;  dv += k6a[i] * b6;

				x5[i] = x0[i] + dp * dt;
				v5[i] = v0[i] + dv * dt;
			}

			// ── k7 — FSAL: derivatives at the new 5th-order state ────────────
			//   k7 is simultaneously:
			//     • the embedded 4th-order correction term (error estimator)
			//     • conceptually k1 for the next call (First Same As Last)
			//     • the final acceleration stored in m_accVec on accept
			computeAccRK(bodies, x5, k7a);
			for (int i = 0; i < n; i++)
				k7p[i] = bodies[i]->movability ? v5[i] : vectorP(0.0, 0.0);

			// ── Error norm — max over bodies, both pos and vel components ─────
			// err ≤ 1  ↔  local truncation error ≤ tol  (step accepted)
			double err = 0.0;
			for (int i = 0; i < n; i++) {
				if (!bodies[i]->movability) continue;

				// Error vector in position space
				vectorP ex = k1p[i] * e1;
				ex += k3p[i] * e3;  ex += k4p[i] * e4;
				ex += k5p[i] * e5;  ex += k6p[i] * e6;
				ex += k7p[i] * e7;
				ex = ex * dt;

				// Error vector in velocity space
				vectorP ev = k1a[i] * e1;
				ev += k3a[i] * e3;  ev += k4a[i] * e4;
				ev += k5a[i] * e5;  ev += k6a[i] * e6;
				ev += k7a[i] * e7;
				ev = ev * dt;

				// Mixed absolute/relative scale: tol·max(1, |solution|)
				double scx = tol * std::max(1.0, x5[i].mag());
				double scv = tol * std::max(1.0, v5[i].mag());

				err = std::max(err, ex.mag() / scx);
				err = std::max(err, ev.mag() / scv);
			}

			// ── Step-size factor ──────────────────────────────────────────────
			double factor;
			if (err < 1e-30) {
				factor = max_grow;                      // solution is essentially exact
			} else {
				factor = safety * std::pow(err, -expo);
				factor = std::min(max_grow, std::max(min_shrk, factor));
			}

			if (err <= 1.0 || dt <= 1e-10)
			{
				// ── Accept: commit 5th-order solution to bodies ───────────────
				accepted = true;
				for (int i = 0; i < n; i++) {
					if (!bodies[i]->movability) continue;
					bodies[i]->m_posVec = x5[i];
					bodies[i]->m_velVec = v5[i];
					bodies[i]->m_accVec = k7a[i];                        // FSAL: already computed
					bodies[i]->m_forVec = k7a[i] * bodies[i]->m_Mass;   // keep forVec in sync
					bodies[i]->m_forRes = vectorP(0.0, 0.0);             // clear accumulator
				}
			}

			// Update dt for next call (or retry this step with a smaller dt)
			dt = std::min(dt_max, std::max(1e-10, dt * factor));
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
			/*vectorP vector(3, 3);
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
			auto nig11 = std::make_unique<Body>(1000000000000.0f, 0.2f, true, vector411); */

			/*bodys.push_back(std::move(star));
			bodys.push_back(std::move(perf));
			bodys.push_back(std::move(nig));

			bodys.push_back(std::move(star1));
			bodys.push_back(std::move(perf1));
			bodys.push_back(std::move(nig1));

			bodys.push_back(std::move(star11));
			bodys.push_back(std::move(perf11));
			bodys.push_back(std::move(nig11)); */

			/*float mass = 100000000000.0f;

			vectorP pos1( 0.97f,  -0.2430f);
			vectorP pos2(-0.97f,   0.2430f);
			vectorP pos3( 0.0f,    0.0f);

			vectorP vel1( 0.4662f,   0.4323f);
			vectorP vel2( 0.4662,    0.4323f);
			vectorP vel3(-0.24f,  -0.8647f);

			auto star11 = std::make_unique<Body>(mass, 0.1f,  true, pos1, vel1);
			auto perf11 = std::make_unique<Body>(mass, 0.1f, true, pos2, vel2);
			auto nig11  = std::make_unique<Body>(mass, 0.1f, true, pos3, vel3);

			bodys.push_back(std::move(star11));
			bodys.push_back(std::move(perf11));
			bodys.push_back(std::move(nig11));*/


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


						//auto t0 = clock::now();


						for(int s = 0 ; s < hmframe && !bodys.empty() && !WindowShouldClose(); s++)
						{
							frame++;

							physics::moveRK45(bodys, dt);

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

							//bodys[1]->GetVal();

							auto colData = (physics::checkCol(bodys,colClusters));
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