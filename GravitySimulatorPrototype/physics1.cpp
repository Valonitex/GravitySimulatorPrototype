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

// ═══════════════════════════════════════════════════════════════════════════
// MIKKOLA CHAIN REGULARIZATION WITH LEVI-CIVITA TRANSFORMATIONS
// ─────────────────────────────────────────────────────────────────────────
// Implements Algorithmic Chain Regularization (Mikkola & Aarseth 1993/1998)
// for 2D gravitational N-body simulation.
//
// DISCLOSURES: Zero modifications to existing code. This entire block is
// appended by re-opening namespace physics.
//
// Chain ordering places the closest pairs adjacent. Each chain link is
// LC-transformed (z = u²) to remove the 1/r gravitational singularity.
// A global Sundman time transformation  dt/ds = Ω = (Σ 1/r_k)⁻¹
// ensures all EOM terms remain bounded as any chain distance → 0.
// ═══════════════════════════════════════════════════════════════════════════

namespace physics {

// ── Const-safe arithmetic helpers ────────────────────────────────────────
// vectorP's operator+ and operator* lack const qualifiers on *this,
// preventing their use on const/temporary operands. These free functions
// provide const-correct alternatives used throughout the LC code.

static inline vectorP vAdd(const vectorP& a, const vectorP& b) {
	return vectorP(a.icap + b.icap, a.jcap + b.jcap);
}
static inline vectorP vSub(const vectorP& a, const vectorP& b) {
	return vectorP(a.icap - b.icap, a.jcap - b.jcap);
}
static inline vectorP vScale(const vectorP& a, double s) {
	return vectorP(a.icap * s, a.jcap * s);
}
static inline double vDot(const vectorP& a, const vectorP& b) {
	return a.icap * b.icap + a.jcap * b.jcap;
}
static inline double vMagSq(const vectorP& a) {
	return a.icap * a.icap + a.jcap * a.jcap;
}
static inline double vMag(const vectorP& a) {
	return std::sqrt(vMagSq(a));
}

// ── Data structures ──────────────────────────────────────────────────────

// Per-link LC state for one chain bond
struct LCLink {
	vectorP u;       // LC coordinates (u₁, u₂)
	vectorP w;       // LC fictitious velocity  du/dτ_k
	double  h;       // Binding energy of this chain pair
};

// Full chain state for N bodies
struct ChainState {
	std::vector<int>    sigma;   // Chain permutation: sigma[i] = body index
	std::vector<LCLink> links;   // N-1 chain links
	vectorP             R;       // Center-of-mass position
	vectorP             V_cm;    // Center-of-mass velocity
	double              t_phys;  // Accumulated physical time this step
	double              Omega;   // Current time-transformation factor
	int                 N;       // Number of bodies
};

// Derivatives of the full chain state (RHS output)
struct ChainDerivs {
	std::vector<vectorP> du;     // du_k/ds for each link
	std::vector<vectorP> dw;     // dw_k/ds for each link
	std::vector<double>  dh;     // dh_k/ds for each link
	vectorP              dR;     // dR/ds
	double               dt_ds;  // dt/ds = Ω
};

// ── LC matrix operations ─────────────────────────────────────────────────
// The LC matrix L(u) represents complex multiplication by u.
// L(u)^T represents complex multiplication by conj(u).

// L(u) * v  =  u · v  (complex product)
static inline vectorP lc_L_mul(const vectorP& u, const vectorP& v) {
	return vectorP(u.icap * v.icap - u.jcap * v.jcap,
	               u.jcap * v.icap + u.icap * v.jcap);
}

// L(u)^T * v  =  conj(u) · v  (complex conjugate product)
static inline vectorP lc_LT_mul(const vectorP& u, const vectorP& v) {
	return vectorP( u.icap * v.icap + u.jcap * v.jcap,
	               -u.jcap * v.icap + u.icap * v.jcap);
}

// ── Coordinate transforms ────────────────────────────────────────────────

// Cartesian → LC coordinate  (inverse of the squaring map X = u²)
//   u₁ = √((r+x)/2),  u₂ = y / (2u₁)
// with alternate branch when x ≈ -r to avoid cancellation.
static vectorP lc_cartToU(const vectorP& X) {
	double r = vMag(X);
	if (r < 1e-30) return vectorP(0.0, 0.0);

	double u1, u2;
	if (r + X.icap > 1e-15 * r) {
		u1 = std::sqrt((r + X.icap) / 2.0);
		u2 = X.jcap / (2.0 * u1);
	} else {
		u2 = std::sqrt((r - X.icap) / 2.0);
		u1 = X.jcap / (2.0 * u2);
	}
	return vectorP(u1, u2);
}

// LC → Cartesian position  (squaring map: X = L(u)·u = u²)
static inline vectorP lc_uToCart(const vectorP& u) {
	return vectorP(u.icap * u.icap - u.jcap * u.jcap,
	               2.0 * u.icap * u.jcap);
}

// Cartesian velocity → LC fictitious velocity
//   w = (1/2) L(u)^T · Ẋ
static inline vectorP lc_velToW(const vectorP& u, const vectorP& Xdot) {
	return vScale(lc_LT_mul(u, Xdot), 0.5);
}

// LC fictitious velocity → Cartesian velocity
//   Ẋ = 2 L(u) w / |u|²
static inline vectorP lc_wToVel(const vectorP& u, const vectorP& w) {
	double uSq = vMagSq(u);
	if (uSq < 1e-30) return vectorP(0.0, 0.0);
	return vScale(lc_L_mul(u, w), 2.0 / uSq);
}

// Binding energy  h = |Ẋ|²/2 − G·M_pair / |X|
static inline double lc_bindingEnergy(const vectorP& Xdot, double r, double Mpair) {
	return 0.5 * vMagSq(Xdot) - G * Mpair / std::max(r, 1e-30);
}

// Compute Ω = (Σ 1/r_k)⁻¹   where r_k = |u_k|²
// Bounded: Ω/r_k ≤ 1 for all k, ensuring regularized EOM stay finite.
static double lc_computeOmega(const std::vector<LCLink>& links) {
	if (links.empty()) return 1.0;
	double S = 0.0;
	for (const auto& lk : links) {
		double rk = vMagSq(lk.u);
		S += 1.0 / std::max(rk, 1e-30);
	}
	return 1.0 / std::max(S, 1e-30);
}

// ── Chain construction (nearest-neighbor) ────────────────────────────────
// Builds a topological chain ordering where the closest pairs in the
// system are adjacent links. The chain is constructed by:
//   1. Seed with the global closest pair
//   2. Greedily extend from both ends with the nearest unchained body

static ChainState lc_buildChain(const std::vector<std::unique_ptr<Body>>& bodies) {
	ChainState chain;
	chain.N = static_cast<int>(bodies.size());
	int N = chain.N;

	if (N == 0) {
		chain.R = vectorP(0, 0);
		chain.V_cm = vectorP(0, 0);
		chain.t_phys = 0.0;
		chain.Omega = 1.0;
		return chain;
	}

	// ── Center of mass ──────────────────────────────────────────────
	double M = 0.0;
	vectorP wPos(0, 0), wVel(0, 0);
	for (int i = 0; i < N; i++) {
		M += bodies[i]->m_Mass;
		wPos = vAdd(wPos, vScale(bodies[i]->m_posVec, bodies[i]->m_Mass));
		wVel = vAdd(wVel, vScale(bodies[i]->m_velVec, bodies[i]->m_Mass));
	}
	chain.R = vScale(wPos, 1.0 / M);
	chain.V_cm = vScale(wVel, 1.0 / M);

	if (N == 1) {
		chain.sigma.push_back(0);
		chain.t_phys = 0.0;
		chain.Omega = 1.0;
		return chain;
	}

	// ── Find closest pair for chain seed ─────────────────────────────
	double minDistSq = 1e30;
	int bestI = 0, bestJ = 1;
	for (int i = 0; i < N; i++) {
		for (int j = i + 1; j < N; j++) {
			double d2 = vMagSq(vSub(bodies[i]->m_posVec, bodies[j]->m_posVec));
			if (d2 < minDistSq) {
				minDistSq = d2;
				bestI = i;
				bestJ = j;
			}
		}
	}

	// ── Grow chain from both ends via nearest-neighbor ───────────────
	chain.sigma.reserve(N);
	chain.sigma.push_back(bestI);
	chain.sigma.push_back(bestJ);
	std::vector<bool> chained(N, false);
	chained[bestI] = true;
	chained[bestJ] = true;

	while (static_cast<int>(chain.sigma.size()) < N) {
		int leftEnd  = chain.sigma.front();
		int rightEnd = chain.sigma.back();

		double bestLeftDist = 1e30, bestRightDist = 1e30;
		int bestLeftBody = -1, bestRightBody = -1;

		for (int i = 0; i < N; i++) {
			if (chained[i]) continue;
			double dL = vMagSq(vSub(bodies[i]->m_posVec, bodies[leftEnd]->m_posVec));
			double dR = vMagSq(vSub(bodies[i]->m_posVec, bodies[rightEnd]->m_posVec));
			if (dL < bestLeftDist)  { bestLeftDist = dL;  bestLeftBody = i; }
			if (dR < bestRightDist) { bestRightDist = dR; bestRightBody = i; }
		}

		if (bestLeftDist <= bestRightDist && bestLeftBody >= 0) {
			chain.sigma.insert(chain.sigma.begin(), bestLeftBody);
			chained[bestLeftBody] = true;
		} else if (bestRightBody >= 0) {
			chain.sigma.push_back(bestRightBody);
			chained[bestRightBody] = true;
		}
	}

	// ── Build LC links from chain vectors ────────────────────────────
	int nLinks = N - 1;
	chain.links.resize(nLinks);
	for (int k = 0; k < nLinks; k++) {
		int si = chain.sigma[k];
		int sj = chain.sigma[k + 1];

		vectorP X    = vSub(bodies[sj]->m_posVec, bodies[si]->m_posVec);
		vectorP Xdot = vSub(bodies[sj]->m_velVec, bodies[si]->m_velVec);
		double  r    = vMag(X);
		double  Mpair = bodies[si]->m_Mass + bodies[sj]->m_Mass;

		chain.links[k].u = lc_cartToU(X);
		chain.links[k].w = lc_velToW(chain.links[k].u, Xdot);
		chain.links[k].h = lc_bindingEnergy(Xdot, r, Mpair);
	}

	chain.t_phys = 0.0;
	chain.Omega  = lc_computeOmega(chain.links);

	return chain;
}

// ── Position recovery from chain state ──────────────────────────────────
// Reconstructs absolute Cartesian positions from the LC chain state
// and center-of-mass.   pos[] is indexed by BODY INDEX (not chain index).
//
//   r_{σ(0)} = R − (1/M) Σ_k (tail-mass_k · X_k)
//   r_{σ(i)} = r_{σ(0)} + Σ_{j<i} X_j

static void lc_chainToPositions(const ChainState& chain,
                                const std::vector<std::unique_ptr<Body>>& bodies,
                                std::vector<vectorP>& pos) {
	int N = chain.N;
	pos.resize(N);
	if (N == 0) return;
	if (N == 1) { pos[chain.sigma[0]] = chain.R; return; }

	int nLinks = N - 1;

	// Reconstruct chain vectors from LC
	std::vector<vectorP> X(nLinks);
	for (int k = 0; k < nLinks; k++)
		X[k] = lc_uToCart(chain.links[k].u);

	// Total mass
	double M = 0.0;
	for (int i = 0; i < N; i++) M += bodies[chain.sigma[i]]->m_Mass;

	// First body: r_{σ(0)} = R − (1/M) Σ_k tailMass_k · X_k
	vectorP offset(0.0, 0.0);
	for (int k = 0; k < nLinks; k++) {
		double tailMass = 0.0;
		for (int j = k + 1; j < N; j++)
			tailMass += bodies[chain.sigma[j]]->m_Mass;
		offset = vAdd(offset, vScale(X[k], tailMass));
	}
	vectorP r0 = vSub(chain.R, vScale(offset, 1.0 / M));
	pos[chain.sigma[0]] = r0;

	// Remaining bodies: cumulative chain vector sum
	vectorP cumX = r0;
	for (int k = 0; k < nLinks; k++) {
		cumX = vAdd(cumX, X[k]);
		pos[chain.sigma[k + 1]] = cumX;
	}
}

// ── Velocity recovery from chain state ──────────────────────────────────
// Same structure as position recovery, using  Ẋ_k = 2 L(u_k) w_k / |u_k|²

static void lc_chainToVelocities(const ChainState& chain,
                                 const std::vector<std::unique_ptr<Body>>& bodies,
                                 std::vector<vectorP>& vel) {
	int N = chain.N;
	vel.resize(N);
	if (N == 0) return;
	if (N == 1) { vel[chain.sigma[0]] = chain.V_cm; return; }

	int nLinks = N - 1;

	// Reconstruct chain velocities from LC
	std::vector<vectorP> Xdot(nLinks);
	for (int k = 0; k < nLinks; k++)
		Xdot[k] = lc_wToVel(chain.links[k].u, chain.links[k].w);

	double M = 0.0;
	for (int i = 0; i < N; i++) M += bodies[chain.sigma[i]]->m_Mass;

	vectorP velOffset(0.0, 0.0);
	for (int k = 0; k < nLinks; k++) {
		double tailMass = 0.0;
		for (int j = k + 1; j < N; j++)
			tailMass += bodies[chain.sigma[j]]->m_Mass;
		velOffset = vAdd(velOffset, vScale(Xdot[k], tailMass));
	}
	vectorP v0 = vSub(chain.V_cm, vScale(velOffset, 1.0 / M));
	vel[chain.sigma[0]] = v0;

	vectorP cumV = v0;
	for (int k = 0; k < nLinks; k++) {
		cumV = vAdd(cumV, Xdot[k]);
		vel[chain.sigma[k + 1]] = cumV;
	}
}

// ── Back-transform: write chain state → Body objects ────────────────────
// Recovers Cartesian pos/vel from the chain, writes to Body instances,
// respects movability (pinned bodies keep original pos/vel), and
// re-syncs m_accVec / m_forVec via resolve().

static void lc_chainToCartesian(const ChainState& chain,
                                std::vector<std::unique_ptr<Body>>& bodies) {
	int N = chain.N;
	if (N == 0) return;

	// Save originals for pinned bodies
	std::vector<vectorP> origPos(N), origVel(N);
	for (int i = 0; i < N; i++) {
		int bi = chain.sigma[i];
		origPos[i] = bodies[bi]->m_posVec;
		origVel[i] = bodies[bi]->m_velVec;
	}

	// Recover positions and velocities from chain
	std::vector<vectorP> pos, vel;
	lc_chainToPositions(chain, bodies, pos);
	lc_chainToVelocities(chain, bodies, vel);

	// Write back, respecting movability
	for (int i = 0; i < N; i++) {
		int bi = chain.sigma[i];
		if (bodies[bi]->movability) {
			bodies[bi]->m_posVec = pos[bi];
			bodies[bi]->m_velVec = vel[bi];
		} else {
			bodies[bi]->m_posVec = origPos[i];
			bodies[bi]->m_velVec = origVel[i];
		}
	}

	// Re-sync forces/accelerations: clear accumulators then resolve
	for (auto& b : bodies)
		b->m_forRes = vectorP(0.0, 0.0);
	resolve(bodies);
}

// ── Perturbation vector P_k ─────────────────────────────────────────────
// Computes the non-chain-pair acceleration difference acting on link k:
//   P_k = Σ_{j ∉ {σ(k),σ(k+1)}} G·m_j·[ (r_j−r_{σ(k+1)})/d³ − (r_j−r_{σ(k)})/d³ ]
//
// This formula never evaluates the singular chain-pair force 1/|X_k|³.
// Non-chain interactions use softening ε = 0.1 for safety.

static vectorP lc_perturbation(const ChainState& chain,
                               const std::vector<std::unique_ptr<Body>>& bodies,
                               const std::vector<vectorP>& positions,
                               int k) {
	const double eps = 0.1;
	int si = chain.sigma[k];
	int sj = chain.sigma[k + 1];
	int N  = chain.N;

	vectorP P(0.0, 0.0);

	for (int b = 0; b < N; b++) {
		if (b == si || b == sj) continue;

		double mj = bodies[b]->m_Mass;

		// Acceleration of σ(k+1) due to body b
		vectorP r_b_sj = vSub(positions[b], positions[sj]);
		double d2_sj = vMagSq(r_b_sj) + eps * eps;
		double d1_sj = std::sqrt(d2_sj);
		double d3_sj = d2_sj * d1_sj;

		// Acceleration of σ(k) due to body b
		vectorP r_b_si = vSub(positions[b], positions[si]);
		double d2_si = vMagSq(r_b_si) + eps * eps;
		double d1_si = std::sqrt(d2_si);
		double d3_si = d2_si * d1_si;

		// P_k += G·m_b·[ (r_b − r_sj)/d³_sj − (r_b − r_si)/d³_si ]
		P = vAdd(P, vSub(vScale(r_b_sj, G * mj / d3_sj),
		                 vScale(r_b_si, G * mj / d3_si)));
	}
	return P;
}

// ── Full RHS evaluation ──────────────────────────────────────────────────
// Computes all derivatives of the chain state in global fictitious time s:
//   du_k/ds = α_k · w_k
//   dw_k/ds = α_k · [ (h_k/2)·u_k + (r_k/2)·Lᵀ·P_k ]
//   dh_k/ds = 2·α_k · wᵀ · Lᵀ·P_k
//   dR/ds   = Ω · V_cm
//   dt/ds   = Ω
// where α_k = Ω/|u_k|² ≤ 1 (bounded by construction of Ω).

static void lc_chainRHS(ChainState& chain,
                        const std::vector<std::unique_ptr<Body>>& bodies,
                        ChainDerivs& derivs) {
	int nLinks = static_cast<int>(chain.links.size());
	derivs.du.resize(nLinks);
	derivs.dw.resize(nLinks);
	derivs.dh.resize(nLinks);

	// Recompute Ω from current link positions
	chain.Omega = lc_computeOmega(chain.links);
	double Omega = chain.Omega;

	// Recover Cartesian positions for perturbation
	std::vector<vectorP> positions;
	lc_chainToPositions(chain, bodies, positions);

	for (int k = 0; k < nLinks; k++) {
		double rk    = vMagSq(chain.links[k].u);          // |u_k|² = r_k
		double alpha = Omega / std::max(rk, 1e-30);       // bounded ≤ 1

		// du_k/ds = α · w_k
		derivs.du[k] = vScale(chain.links[k].w, alpha);

		// Perturbation P_k and its LC projection
		vectorP Pk    = lc_perturbation(chain, bodies, positions, k);
		vectorP LT_Pk = lc_LT_mul(chain.links[k].u, Pk);

		// dw_k/ds = α · [ (h_k/2)·u_k + (r_k/2)·Lᵀ·P_k ]
		vectorP term1 = vScale(chain.links[k].u, chain.links[k].h * 0.5);
		vectorP term2 = vScale(LT_Pk, rk * 0.5);
		derivs.dw[k]  = vScale(vAdd(term1, term2), alpha);

		// dh_k/ds = 2·α · wᵀ Lᵀ P_k
		double wdotLTP = vDot(chain.links[k].w, LT_Pk);
		derivs.dh[k] = 2.0 * alpha * wdotLTP;
	}

	derivs.dR    = vScale(chain.V_cm, Omega);
	derivs.dt_ds = Omega;
}

// ── State arithmetic for integrators ─────────────────────────────────────

// In-place: state += derivs * h
static void lc_axpy(ChainState& state, const ChainDerivs& f, double h) {
	int nLinks = static_cast<int>(state.links.size());
	for (int k = 0; k < nLinks; k++) {
		state.links[k].u = vAdd(state.links[k].u, vScale(f.du[k], h));
		state.links[k].w = vAdd(state.links[k].w, vScale(f.dw[k], h));
		state.links[k].h += f.dh[k] * h;
	}
	state.R      = vAdd(state.R, vScale(f.dR, h));
	state.t_phys += f.dt_ds * h;
}

// Copy: return state + derivs * h
static ChainState lc_addScaled(const ChainState& state, const ChainDerivs& f, double h) {
	ChainState result = state;
	lc_axpy(result, f, h);
	return result;
}

// ── Drift & kick building blocks (for symplectic integrators) ────────────

// Drift: advance positions (u, R, t) using current velocities (w).
// Ω is evaluated at the START of the drift, then recomputed after.
static void lc_drift(ChainState& chain, double ds) {
	double Omega = chain.Omega;
	int nLinks   = static_cast<int>(chain.links.size());

	for (int k = 0; k < nLinks; k++) {
		double rk    = vMagSq(chain.links[k].u);
		double alpha = Omega / std::max(rk, 1e-30);
		chain.links[k].u = vAdd(chain.links[k].u,
		                        vScale(chain.links[k].w, alpha * ds));
	}

	chain.R      = vAdd(chain.R, vScale(chain.V_cm, Omega * ds));
	chain.t_phys += Omega * ds;

	// Recompute Ω at new positions
	chain.Omega = lc_computeOmega(chain.links);
}

// Kick: advance velocities (w, h) using forces at current positions (u).
// Recovers Cartesian positions from chain, computes perturbations,
// then updates w and h for each link.
static void lc_kick(ChainState& chain,
                    const std::vector<std::unique_ptr<Body>>& bodies,
                    double ds) {
	double Omega = chain.Omega;
	int nLinks   = static_cast<int>(chain.links.size());

	// Cartesian positions for perturbation
	std::vector<vectorP> positions;
	lc_chainToPositions(chain, bodies, positions);

	for (int k = 0; k < nLinks; k++) {
		double rk    = vMagSq(chain.links[k].u);
		double alpha = Omega / std::max(rk, 1e-30);

		vectorP Pk    = lc_perturbation(chain, bodies, positions, k);
		vectorP LT_Pk = lc_LT_mul(chain.links[k].u, Pk);

		// dw/ds = α · [(h/2)·u + (r_k/2)·Lᵀ·P]
		vectorP dw = vScale(
			vAdd(vScale(chain.links[k].u, chain.links[k].h * 0.5),
			     vScale(LT_Pk, rk * 0.5)),
			alpha);

		// dh/ds = 2·α · wᵀ Lᵀ P
		double wdotLTP = vDot(chain.links[k].w, LT_Pk);
		double dh = 2.0 * alpha * wdotLTP;

		chain.links[k].w = vAdd(chain.links[k].w, vScale(dw, ds));
		chain.links[k].h += dh * ds;
	}
}


// ═════════════════════════════════════════════════════════════════════════
// INTEGRATOR WRAPPERS
// ═════════════════════════════════════════════════════════════════════════

// ── moveVerletLC ─────────────────────────────────────────────────────────
// Velocity Verlet (KDK leapfrog) in chain-regularized LC space.
// Kick(Δs/2) → Drift(Δs) → Kick(Δs/2)
//
// dt [in/out]: physical timestep. On return, holds the actual physical
//              time advanced (may differ from input due to Sundman).

void moveVerletLC(std::vector<std::unique_ptr<Body>>& bodies, double& dt) {
	int N = static_cast<int>(bodies.size());
	if (N == 0) return;
	if (N == 1) {
		if (bodies[0]->movability)
			bodies[0]->m_posVec = vAdd(bodies[0]->m_posVec,
			                           vScale(bodies[0]->m_velVec, dt));
		resolve(bodies);
		return;
	}

	ChainState chain = lc_buildChain(bodies);
	double ds = dt / std::max(chain.Omega, 1e-30);

	lc_kick(chain, bodies, ds * 0.5);
	lc_drift(chain, ds);
	lc_kick(chain, bodies, ds * 0.5);

	lc_chainToCartesian(chain, bodies);
	dt = chain.t_phys;
}

// ── moveYoshidaLC ────────────────────────────────────────────────────────
// 4th-order Yoshida symplectic integrator (DKD structure) in
// chain-regularized LC space. Uses the same coefficients as moveYoshida.

void moveYoshidaLC(std::vector<std::unique_ptr<Body>>& bodies, double& dt) {
	int N = static_cast<int>(bodies.size());
	if (N == 0) return;
	if (N == 1) {
		if (bodies[0]->movability)
			bodies[0]->m_posVec = vAdd(bodies[0]->m_posVec,
			                           vScale(bodies[0]->m_velVec, dt));
		resolve(bodies);
		return;
	}

	const double c[4] = {  0.6756035959798289, -0.1756035959798289,
	                       -0.1756035959798289,  0.6756035959798289 };
	const double d[3] = {  1.3512071919596578, -1.7024143839193156,
	                        1.3512071919596578 };

	ChainState chain = lc_buildChain(bodies);
	double ds = dt / std::max(chain.Omega, 1e-30);

	for (int step = 0; step < 4; step++) {
		lc_drift(chain, c[step] * ds);
		if (step < 3)
			lc_kick(chain, bodies, d[step] * ds);
	}

	lc_chainToCartesian(chain, bodies);
	dt = chain.t_phys;
}

// ── moveHermiteLC ────────────────────────────────────────────────────────
// 4th-order Hermite predictor-corrector (PECE) on the full chain state
// vector. Uses numerical jerk estimation (one extra RHS evaluation).
//
// dt [in/out]: On entry, the current physical timestep.
//              On return, the Aarseth-recommended next timestep.

double moveHermiteLC(std::vector<std::unique_ptr<Body>>& bodies, double& dt) {
	int N = static_cast<int>(bodies.size());
	if (N == 0) return dt;
	if (N == 1) {
		if (bodies[0]->movability)
			bodies[0]->m_posVec = vAdd(bodies[0]->m_posVec,
			                           vScale(bodies[0]->m_velVec, dt));
		resolve(bodies);
		return dt;
	}

	ChainState chain = lc_buildChain(bodies);
	int nLinks = N - 1;
	double ds  = dt / std::max(chain.Omega, 1e-30);

	// ── 1. Evaluate RHS at current state ────────────────────────────
	ChainDerivs f0;
	lc_chainRHS(chain, bodies, f0);

	// ── 2. Numerical jerk: j₀ ≈ (f(y+f·ε) − f(y)) / ε ─────────────
	double eps_j = std::max(ds * 0.01, 1e-12);
	ChainState state_eps = lc_addScaled(chain, f0, eps_j);
	ChainDerivs f_eps;
	lc_chainRHS(state_eps, bodies, f_eps);

	ChainDerivs j0;
	j0.du.resize(nLinks);  j0.dw.resize(nLinks);  j0.dh.resize(nLinks);
	for (int k = 0; k < nLinks; k++) {
		j0.du[k] = vScale(vSub(f_eps.du[k], f0.du[k]), 1.0 / eps_j);
		j0.dw[k] = vScale(vSub(f_eps.dw[k], f0.dw[k]), 1.0 / eps_j);
		j0.dh[k] = (f_eps.dh[k] - f0.dh[k]) / eps_j;
	}
	j0.dR    = vScale(vSub(f_eps.dR, f0.dR), 1.0 / eps_j);
	j0.dt_ds = (f_eps.dt_ds - f0.dt_ds) / eps_j;

	// ── 3. Predict (3rd order): y_p = y₀ + f₀·Δs + j₀·(Δs²/2) ─────
	ChainState state_pred = lc_addScaled(chain, f0, ds);
	lc_axpy(state_pred, j0, ds * ds * 0.5);

	// ── 4. Evaluate at predicted state ───────────────────────────────
	ChainDerivs f1;
	lc_chainRHS(state_pred, bodies, f1);

	// ── 5. Jerk at predicted: j₁ ≈ (f₁ − f₀) / Δs ──────────────────
	ChainDerivs j1;
	j1.du.resize(nLinks);  j1.dw.resize(nLinks);  j1.dh.resize(nLinks);
	for (int k = 0; k < nLinks; k++) {
		j1.du[k] = vScale(vSub(f1.du[k], f0.du[k]), 1.0 / ds);
		j1.dw[k] = vScale(vSub(f1.dw[k], f0.dw[k]), 1.0 / ds);
		j1.dh[k] = (f1.dh[k] - f0.dh[k]) / ds;
	}
	j1.dR    = vScale(vSub(f1.dR, f0.dR), 1.0 / ds);
	j1.dt_ds = (f1.dt_ds - f0.dt_ds) / ds;

	// ── 6. Correct (4th order Hermite for first-order ODE):
	//       y₁ = y₀ + Δs·(f₀+f₁)/2 + Δs²·(j₀−j₁)/12 ──────────────
	ChainState state_corr = chain;
	double ds2_12 = ds * ds / 12.0;

	for (int k = 0; k < nLinks; k++) {
		// Trapezoidal: Δs·(f₀+f₁)/2
		state_corr.links[k].u = vAdd(state_corr.links[k].u,
			vScale(vAdd(f0.du[k], f1.du[k]), ds * 0.5));
		state_corr.links[k].w = vAdd(state_corr.links[k].w,
			vScale(vAdd(f0.dw[k], f1.dw[k]), ds * 0.5));
		state_corr.links[k].h += (f0.dh[k] + f1.dh[k]) * ds * 0.5;

		// Hermite correction: Δs²·(j₀−j₁)/12
		state_corr.links[k].u = vAdd(state_corr.links[k].u,
			vScale(vSub(j0.du[k], j1.du[k]), ds2_12));
		state_corr.links[k].w = vAdd(state_corr.links[k].w,
			vScale(vSub(j0.dw[k], j1.dw[k]), ds2_12));
		state_corr.links[k].h += (j0.dh[k] - j1.dh[k]) * ds2_12;
	}
	state_corr.R = vAdd(state_corr.R,
		vScale(vAdd(f0.dR, f1.dR), ds * 0.5));
	state_corr.R = vAdd(state_corr.R,
		vScale(vSub(j0.dR, j1.dR), ds2_12));
	state_corr.t_phys += (f0.dt_ds + f1.dt_ds) * ds * 0.5;
	state_corr.t_phys += (j0.dt_ds - j1.dt_ds) * ds2_12;

	// ── 7. Aarseth adaptive timestep ─────────────────────────────────
	const double eta = 0.02;
	double ds_candidate = 2.0 * ds;

	for (int k = 0; k < nLinks; k++) {
		double a0 = vMag(f0.dw[k]);
		double a1 = vMag(j0.dw[k]);
		vectorP snap_k = vScale(vSub(j0.dw[k], j1.dw[k]), 1.0 / ds);
		double a2 = vMag(snap_k);
		double a3 = (a2 > 1e-30 && ds > 1e-30) ? a2 / ds : 0.0;

		double denom = a1 * a3 + a2 * a2;
		if (denom > 1e-30) {
			double ds_i = eta * std::sqrt((a0 * a2 + a1 * a1) / denom);
			ds_candidate = std::min(ds_candidate, ds_i);
		}
	}
	ds_candidate = std::max(ds_candidate, 1e-12);

	// ── 8. Finalize ──────────────────────────────────────────────────
	state_corr.Omega = lc_computeOmega(state_corr.links);
	lc_chainToCartesian(state_corr, bodies);

	// Write back recommended next physical timestep
	dt = std::max(ds_candidate * state_corr.Omega, 1e-7);

	return ds;
}

// ── moveRK45LC ───────────────────────────────────────────────────────────
// Dormand-Prince RK45 adaptive integrator applied to the full chain state.
// Uses the same Butcher tableau and error control as the existing moveRK45.
//
// dt [in/out]: current step size. Updated for next call.
// tol:         error tolerance (default 1e-6).
// dt_max:      hard ceiling on physical step size (default 1/30 s).

double moveRK45LC(std::vector<std::unique_ptr<Body>>& bodies,
                double& dt,
                double  tol    /*= 1e-6*/,
                double  dt_max /*= 1.0 / 30.0*/) {
	int N = static_cast<int>(bodies.size());
	if (N == 0) return dt;
	if (N == 1) {
		if (bodies[0]->movability)
			bodies[0]->m_posVec = vAdd(bodies[0]->m_posVec,
			                           vScale(bodies[0]->m_velVec, dt));
		resolve(bodies);
		return dt;
	}

	int nLinks = N - 1;

	// ── Dormand-Prince Butcher tableau ───────────────────────────────
	constexpr double a21 = 1.0 / 5.0;
	constexpr double a31 = 3.0 / 40.0,         a32 = 9.0 / 40.0;
	constexpr double a41 = 44.0 / 45.0,         a42 = -56.0 / 15.0,
	                 a43 = 32.0 / 9.0;
	constexpr double a51 = 19372.0 / 6561.0,    a52 = -25360.0 / 2187.0,
	                 a53 = 64448.0 / 6561.0,    a54 = -212.0 / 729.0;
	constexpr double a61 = 9017.0 / 3168.0,     a62 = -355.0 / 33.0,
	                 a63 = 46732.0 / 5247.0,    a64 = 49.0 / 176.0,
	                 a65 = -5103.0 / 18656.0;

	constexpr double b1 = 35.0 / 384.0,     b3 = 500.0 / 1113.0,
	                 b4 = 125.0 / 192.0,     b5 = -2187.0 / 6784.0,
	                 b6 = 11.0 / 84.0;

	constexpr double e1 =  71.0 / 57600.0,   e3 = -71.0 / 16695.0,
	                 e4 =  71.0 / 1920.0,    e5 = -17253.0 / 339200.0,
	                 e6 =  22.0 / 525.0,     e7 = -1.0 / 40.0;

	constexpr double safety   = 0.9;
	constexpr double expo     = 1.0 / 5.0;
	constexpr double max_grow = 5.0;
	constexpr double min_shrk = 0.2;

	// ── Build chain and compute fictitious step ──────────────────────
	ChainState chain0 = lc_buildChain(bodies);
	double ds = dt / std::max(chain0.Omega, 1e-30);

	bool accepted = false;

	while (!accepted) {
		// ── k1 ───────────────────────────────────────────────────────
		ChainDerivs k1;
		{
			ChainState tmp = chain0;
			lc_chainRHS(tmp, bodies, k1);
			chain0.Omega = tmp.Omega;
		}

		// ── k2 at c2 = 1/5 ──────────────────────────────────────────
		ChainState s2 = lc_addScaled(chain0, k1, a21 * ds);
		ChainDerivs k2;
		lc_chainRHS(s2, bodies, k2);

		// ── k3 at c3 = 3/10 ─────────────────────────────────────────
		ChainState s3 = chain0;
		lc_axpy(s3, k1, a31 * ds);
		lc_axpy(s3, k2, a32 * ds);
		ChainDerivs k3;
		lc_chainRHS(s3, bodies, k3);

		// ── k4 at c4 = 4/5 ──────────────────────────────────────────
		ChainState s4 = chain0;
		lc_axpy(s4, k1, a41 * ds);
		lc_axpy(s4, k2, a42 * ds);
		lc_axpy(s4, k3, a43 * ds);
		ChainDerivs k4;
		lc_chainRHS(s4, bodies, k4);

		// ── k5 at c5 = 8/9 ──────────────────────────────────────────
		ChainState s5 = chain0;
		lc_axpy(s5, k1, a51 * ds);
		lc_axpy(s5, k2, a52 * ds);
		lc_axpy(s5, k3, a53 * ds);
		lc_axpy(s5, k4, a54 * ds);
		ChainDerivs k5;
		lc_chainRHS(s5, bodies, k5);

		// ── k6 at c6 = 1 ────────────────────────────────────────────
		ChainState s6 = chain0;
		lc_axpy(s6, k1, a61 * ds);
		lc_axpy(s6, k2, a62 * ds);
		lc_axpy(s6, k3, a63 * ds);
		lc_axpy(s6, k4, a64 * ds);
		lc_axpy(s6, k5, a65 * ds);
		ChainDerivs k6;
		lc_chainRHS(s6, bodies, k6);

		// ── 5th-order solution (b2 = 0 → k2 absent) ─────────────────
		ChainState y5 = chain0;
		lc_axpy(y5, k1, b1 * ds);
		lc_axpy(y5, k3, b3 * ds);
		lc_axpy(y5, k4, b4 * ds);
		lc_axpy(y5, k5, b5 * ds);
		lc_axpy(y5, k6, b6 * ds);

		// ── k7 (FSAL) at the 5th-order solution ─────────────────────
		ChainDerivs k7;
		lc_chainRHS(y5, bodies, k7);

		// ── Error estimate ───────────────────────────────────────────
		double err = 0.0;
		for (int k = 0; k < nLinks; k++) {
			// Position error
			vectorP eu = vScale(k1.du[k], e1);
			eu = vAdd(eu, vScale(k3.du[k], e3));
			eu = vAdd(eu, vScale(k4.du[k], e4));
			eu = vAdd(eu, vScale(k5.du[k], e5));
			eu = vAdd(eu, vScale(k6.du[k], e6));
			eu = vAdd(eu, vScale(k7.du[k], e7));
			eu = vScale(eu, ds);

			// Velocity error
			vectorP ew = vScale(k1.dw[k], e1);
			ew = vAdd(ew, vScale(k3.dw[k], e3));
			ew = vAdd(ew, vScale(k4.dw[k], e4));
			ew = vAdd(ew, vScale(k5.dw[k], e5));
			ew = vAdd(ew, vScale(k6.dw[k], e6));
			ew = vAdd(ew, vScale(k7.dw[k], e7));
			ew = vScale(ew, ds);

			// Energy error
			double eh = (k1.dh[k]*e1 + k3.dh[k]*e3 + k4.dh[k]*e4
			           + k5.dh[k]*e5 + k6.dh[k]*e6 + k7.dh[k]*e7) * ds;

			// Scaled error norms
			double scu = tol * std::max(1.0, vMag(y5.links[k].u));
			double scw = tol * std::max(1.0, vMag(y5.links[k].w));
			double sch = tol * std::max(1.0, std::abs(y5.links[k].h));

			err = std::max(err, vMag(eu) / scu);
			err = std::max(err, vMag(ew) / scw);
			err = std::max(err, std::abs(eh) / sch);
		}

		// ── Step-size factor ─────────────────────────────────────────
		double factor;
		if (err < 1e-30)
			factor = max_grow;
		else {
			factor = safety * std::pow(err, -expo);
			factor = std::min(max_grow, std::max(min_shrk, factor));
		}

		if (err <= 1.0 || ds <= 1e-15) {
			// ── Accept ───────────────────────────────────────────────
			accepted = true;
			y5.Omega = lc_computeOmega(y5.links);
			lc_chainToCartesian(y5, bodies);

			// Convert fictitious step adjustment to physical time
			double ds_new = ds * factor;
			double Omega_end = y5.Omega;
			dt = std::min(dt_max, std::max(1e-10, ds_new * Omega_end));
		} else {
			// ── Reject: shrink ds and retry ──────────────────────────
			ds = std::max(1e-15, ds * factor);
		}
	}
	return ds;
}

} // namespace physics  (chain regularization extension)

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
						double phys_time,diht;

						for(int s = 0 ; s < hmframe && !bodys.empty() && !WindowShouldClose(); s++)
						{
							frame++;

							diht = physics::moveHermiteLC(bodys, dt);
							//diht = physics::moveRK45LC(bodys, dt , 0.00001 , dt);
							phys_time += diht;

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
								DrawText(TextFormat("diht : %f" , diht) , 1000 , 40, 20 , RED);
								DrawText(TextFormat("phys_time : %f" , phys_time) , 1000 , 60, 20 , RED);

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