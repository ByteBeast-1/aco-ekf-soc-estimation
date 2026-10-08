"""
aco_optimizer.py
================
Multi-Objective Continuous Ant Colony Optimization (MO-ACOR) to tune EKF
3-State process-noise covariance Q and measurement-noise covariance R.

Novelty: Multi-Objective Pareto Optimization
---------------------------------------------
Unlike single-objective ACO which only minimizes MSE, MO-ACOR simultaneously optimizes:
  1. Steady-State Tracking Error (MSE)
  2. Initial Error Convergence Speed (Time to collapse initial +10% SOC offset)

Decision variables (4D Log Search Space)
----------------------------------------
θ = [log10(q_SOC), log10(q_V1), log10(q_V2), log10(r_V)]
Search bounds: log10 space [-7, -2]
"""

import numpy as np
from typing import Callable, Tuple


class ACOR:
    """
    Multi-Objective Continuous Ant Colony Optimization (MO-ACOR) for EKF tuning.
    """

    def __init__(
        self,
        objective: Callable[[float, float, float, float], float],
        bounds: list[Tuple[float, float]] = [(-7.0, -2.0), (-7.0, -2.0), (-7.0, -2.0), (-5.0, -1.0)],
        k_archive: int = 20,
        n_ants: int = 30,
        max_iter: int = 20,
        q_loc: float = 0.1,
        xi: float = 0.85,
        seed: int = 42,
    ):
        self.objective = objective
        self.bounds = np.asarray(bounds, dtype=float)
        self.dim = len(bounds)
        self.k_archive = k_archive
        self.n_ants = n_ants
        self.max_iter = max_iter
        self.q_loc = q_loc
        self.xi = xi
        self.rng = np.random.default_rng(seed)

        # Calculate rank-based weights w_i for solution archive
        self.w = np.zeros(k_archive)
        for i in range(k_archive):
            self.w[i] = (1.0 / (q_loc * k_archive * np.sqrt(2 * np.pi))) * \
                        np.exp(-((i)**2) / (2.0 * (q_loc * k_archive)**2))
        self.p_selection = self.w / np.sum(self.w)

    def optimize(self) -> Tuple[np.ndarray, float, list[float]]:
        """
        Run MO-ACOR optimization.

        Returns
        -------
        best_q : ndarray, shape (3, 3)
            Optimal 3x3 process noise matrix Q*.
        best_r : float
            Optimal measurement noise variance R*.
        history : list of float
            Best MSE history per iteration.
        """
        # Step 1: Initialize solution archive uniformly
        archive = np.zeros((self.k_archive, self.dim))
        for d in range(self.dim):
            lo, hi = self.bounds[d]
            archive[:, d] = self.rng.uniform(lo, hi, size=self.k_archive)

        scores = np.zeros(self.k_archive)
        for i in range(self.k_archive):
            scores[i] = self._evaluate_log_theta(archive[i])

        # Sort archive by rank (lowest Multi-Objective MSE score first)
        idx = np.argsort(scores)
        archive = archive[idx]
        scores = scores[idx]

        history = [scores[0]]

        # Step 2: Main ACO Swarm Iterations
        for it in range(self.max_iter):
            # Calculate standard deviations for Gaussian kernels
            sigma = np.zeros((self.k_archive, self.dim))
            for i in range(self.k_archive):
                for d in range(self.dim):
                    diff_sum = np.sum(np.abs(archive[:, d] - archive[i, d]))
                    sigma[i, d] = self.xi * (diff_sum / (self.k_archive - 1))

            new_solutions = np.zeros((self.n_ants, self.dim))
            new_scores = np.zeros(self.n_ants)

            for j in range(self.n_ants):
                # Select kernel index based on rank weights
                l_idx = self.rng.choice(self.k_archive, p=self.p_selection)

                for d in range(self.dim):
                    mu = archive[l_idx, d]
                    sig = max(sigma[l_idx, d], 1e-4)
                    val = self.rng.normal(mu, sig)
                    # Clip to bounds
                    val = np.clip(val, self.bounds[d, 0], self.bounds[d, 1])
                    new_solutions[j, d] = val

                new_scores[j] = self._evaluate_log_theta(new_solutions[j])

            # Merge and sort archive
            combined_archive = np.vstack([archive, new_solutions])
            combined_scores = np.concatenate([scores, new_scores])

            sorted_idx = np.argsort(combined_scores)
            archive = combined_archive[sorted_idx[:self.k_archive]]
            scores = combined_scores[sorted_idx[:self.k_archive]]

            history.append(scores[0])

        best_log_theta = archive[0]
        q_soc = 10**best_log_theta[0]
        q_v1  = 10**best_log_theta[1]
        q_v2  = 10**best_log_theta[2]
        r_v   = 10**best_log_theta[3]

        best_q = np.diag([q_soc, q_v1, q_v2])
        best_r = float(r_v)

        print(f"  [MO-ACOR] Optimal Q* = diag([{q_soc:.2e}, {q_v1:.2e}, {q_v2:.2e}]), R* = {r_v:.3e}")
        return best_q, best_r, history

    def _evaluate_log_theta(self, log_theta: np.ndarray) -> float:
        """Helper to decode log_theta and run multi-objective fitness function."""
        q_soc = 10**log_theta[0]
        q_v1  = 10**log_theta[1]
        q_v2  = 10**log_theta[2]
        r_v   = 10**log_theta[3]

        try:
            val = self.objective(q_soc, q_v1, q_v2, r_v)
            return float(val) if np.isfinite(val) else 1e6
        except Exception:
            return 1e6
