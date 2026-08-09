"""
aco_optimizer.py
================
Ant Colony Optimisation for Continuous Domains (ACOR) to tune EKF
process-noise covariance Q and measurement-noise covariance R.

Background
----------
In the base paper (Kannan M, Sundareswaran K, et al.), ACO is used to find
optimal values of Q and R such that the resulting EKF minimises SOC
estimation error on training data.

This implementation follows the ACOR algorithm:
  Socha K, Dorigo M, "Ant colony optimization for continuous domains",
  European Journal of Operational Research, 2008.

Decision variables (search space)
──────────────────────────────────
θ = [log10(q_SOC),  log10(q_Vp),  log10(r_V)]

We search in log-space so that ACO can efficiently explore the typical
engineering range  1e-8 … 1e-1 without the search space being dominated
by large values.

ACOR algorithm summary
─────────────────────
1.  Initialise an archive T of k_archive solutions uniformly in the
    search bounds.
2.  Evaluate and sort archive by objective J (MSE of SOC estimation).
3.  For each new ant:
    a. Select a Gaussian kernel from the archive, weighted by rank.
    b. Sample a candidate solution from that kernel.
    c. Evaluate J for the candidate.
4.  Add all new solutions to the archive and keep only the best k_archive.
5.  Repeat for max_iter iterations.
6.  Return the best solution found.

Objective function
──────────────────
J(θ) = MSE(SOC_estimated, SOC_reference)
      = (1/N) Σ (SOC_est_k − SOC_ref_k)²
"""

import numpy as np
from typing import Callable


class ACOR:
    """
    ACO for continuous domains (ACOR) applied to EKF covariance tuning.

    Parameters
    ----------
    objective : callable
        f(q_soc, q_vp, r_v) → float  (lower is better)
        The function that runs the EKF and returns MSE.
    bounds : list of (lo, hi) tuples, length 3
        Search bounds for [log10(q_SOC), log10(q_Vp), log10(r_V)].
    k_archive : int
        Size of the solution archive (number of elite ants).
    n_ants : int
        Number of new ants generated per iteration.
    max_iter : int
        Maximum number of iterations.
    q_acor : float
        ACOR diversity parameter.  Smaller → more exploitation.
    xi : float
        Pheromone evaporation / locality parameter (Gaussian spread scaling).
    seed : int, optional
        Random seed for reproducibility.
    verbose : bool
        Print progress every 10 iterations.
    """

    def __init__(
        self,
        objective:  Callable,
        bounds:     list  = None,
        k_archive:  int   = 20,
        n_ants:     int   = 10,
        max_iter:   int   = 60,
        q_acor:     float = 0.1,
        xi:         float = 0.85,
        seed:       int   = 42,
        verbose:    bool  = True,
    ):
        self.objective  = objective
        self.bounds     = bounds or [(-7, -2), (-7, -2), (-5, -1)]
        self.k_archive  = k_archive
        self.n_ants     = n_ants
        self.max_iter   = max_iter
        self.q_acor     = q_acor
        self.xi         = xi
        self.verbose    = verbose

        self.n_dim = len(self.bounds)

        # RNG
        self.rng = np.random.default_rng(seed)

        # Results
        self.best_solution  = None   # [log q_SOC, log q_Vp, log r_V]
        self.best_objective = np.inf
        self.convergence    = []     # best J per iteration

    # ------------------------------------------------------------------
    # Gaussian kernel weight by rank (Eq. 7 in Socha & Dorigo 2008)
    # ------------------------------------------------------------------
    def _weights(self) -> np.ndarray:
        """Compute selection weights for archive solutions (best → highest)."""
        ranks = np.arange(1, self.k_archive + 1, dtype=float)
        w = np.exp(-(ranks - 1)**2 / (2 * self.q_acor**2 * self.k_archive**2))
        return w / w.sum()

    # ------------------------------------------------------------------
    # Sample from one Gaussian kernel centred on an archive solution
    # ------------------------------------------------------------------
    def _sample_gaussian(
        self,
        archive:    np.ndarray,
        sigma:      np.ndarray,
        weights:    np.ndarray,
    ) -> np.ndarray:
        """
        Sample one new candidate by selecting a kernel and drawing from it.

        Parameters
        ----------
        archive : shape (k_archive, n_dim) – archive solutions
        sigma   : shape (k_archive, n_dim) – per-dimension standard deviations
        weights : shape (k_archive,) – selection probabilities

        Returns
        -------
        candidate : shape (n_dim,)
        """
        # Choose kernel
        idx = self.rng.choice(self.k_archive, p=weights)
        mean = archive[idx]
        std  = sigma[idx]

        # Draw from Gaussian and clip to bounds
        candidate = self.rng.normal(mean, std)
        lo = np.array([b[0] for b in self.bounds])
        hi = np.array([b[1] for b in self.bounds])
        return np.clip(candidate, lo, hi)

    # ------------------------------------------------------------------
    # Compute sigma for each archive solution
    # ------------------------------------------------------------------
    def _compute_sigma(self, archive: np.ndarray) -> np.ndarray:
        """
        Standard deviation for each Gaussian kernel.

        σ_l,i = ξ * (1/(k-1)) * Σ_{j≠l} |archive_j,i − archive_l,i|

        Parameters
        ----------
        archive : shape (k_archive, n_dim)

        Returns
        -------
        sigma : shape (k_archive, n_dim)
        """
        k = self.k_archive
        sigma = np.zeros_like(archive)
        for l in range(k):
            diff = np.abs(archive - archive[l])   # (k, n_dim)
            sigma[l] = self.xi / (k - 1) * diff.sum(axis=0)

        # Guard against zero sigma
        sigma = np.maximum(sigma, 1e-6)
        return sigma

    # ------------------------------------------------------------------
    # Main optimisation loop
    # ------------------------------------------------------------------
    def run(self) -> dict:
        """
        Execute the ACOR algorithm.

        Returns
        -------
        dict with keys:
            'best_params'     – [q_SOC, q_Vp, r_V]  (linear scale)
            'best_objective'  – minimum MSE achieved
            'convergence'     – list of best J per iteration
            'archive'         – final archive (log scale)
        """
        lo = np.array([b[0] for b in self.bounds])
        hi = np.array([b[1] for b in self.bounds])

        # ── Initialise archive uniformly ───────────────────────────────
        archive_log = self.rng.uniform(lo, hi, size=(self.k_archive, self.n_dim))
        archive_obj = np.zeros(self.k_archive)

        if self.verbose:
            print("  [ACO] Evaluating initial archive …")

        for i in range(self.k_archive):
            params = 10.0 ** archive_log[i]
            archive_obj[i] = self._evaluate(params)

        # Sort archive (ascending objective)
        order = np.argsort(archive_obj)
        archive_log = archive_log[order]
        archive_obj = archive_obj[order]

        self.best_solution  = archive_log[0].copy()
        self.best_objective = archive_obj[0]
        self.convergence.append(self.best_objective)

        if self.verbose:
            print(f"  [ACO] Iter 0 | Best MSE = {self.best_objective:.6f}")

        # ── Main loop ──────────────────────────────────────────────────
        for iteration in range(1, self.max_iter + 1):
            weights = self._weights()
            sigma   = self._compute_sigma(archive_log)

            new_log = np.zeros((self.n_ants, self.n_dim))
            new_obj = np.zeros(self.n_ants)

            for ant in range(self.n_ants):
                candidate_log = self._sample_gaussian(archive_log, sigma, weights)
                params        = 10.0 ** candidate_log
                new_log[ant]  = candidate_log
                new_obj[ant]  = self._evaluate(params)

            # Merge and keep best k_archive
            combined_log = np.vstack([archive_log, new_log])
            combined_obj = np.concatenate([archive_obj, new_obj])

            order        = np.argsort(combined_obj)
            archive_log  = combined_log[order[:self.k_archive]]
            archive_obj  = combined_obj[order[:self.k_archive]]

            if archive_obj[0] < self.best_objective:
                self.best_solution  = archive_log[0].copy()
                self.best_objective = archive_obj[0]

            self.convergence.append(self.best_objective)

            if self.verbose and (iteration % 5 == 0 or iteration == 1):
                params_str = ", ".join(
                    [f"{10**v:.2e}" for v in self.best_solution]
                )
                print(
                    f"  [ACO] Iter {iteration:3d}/{self.max_iter} | "
                    f"Best MSE = {self.best_objective:.6f} | "
                    f"Params = [{params_str}]"
                )

        best_params_linear = 10.0 ** self.best_solution

        return {
            "best_params":    best_params_linear,   # [q_SOC, q_Vp, r_V]
            "best_objective": self.best_objective,
            "convergence":    self.convergence,
            "archive":        archive_log,
        }

    # ------------------------------------------------------------------
    # Wrap the objective to decode parameters
    # ------------------------------------------------------------------
    def _evaluate(self, params: np.ndarray) -> float:
        """
        Evaluate the objective function.

        Parameters
        ----------
        params : [q_SOC, q_Vp, r_V] in linear scale

        Returns
        -------
        MSE (float)
        """
        q_soc, q_vp, r_v = params
        try:
            val = self.objective(q_soc, q_vp, r_v)
        except Exception:
            val = 1e6   # penalise failed evaluations
        return float(val)
