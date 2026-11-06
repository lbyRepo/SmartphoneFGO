# Factor graph optimization in GNU Octave

A standalone educational example of batch trajectory estimation. All executable
code in this folder is GNU Octave `.m` code; no packages, GTSAM, MATLAB toolboxes,
Android build, or downloaded data are required. This README uses LaTeX math
blocks supported by GitHub Markdown and math-enabled Markdown viewers.

This is an original, simplified demonstration inspired by **Taro Suzuki's
winning Google Smartphone Decimeter Challenge solutions**. It is not a port,
reproduction, or implementation of his competition pipeline. This folder is
independent of `fgo_android_app` and `validation_tools`; the Android project's
canonical technical documentation remains in `fgo_android_app/docs`.

## Run

Start GNU Octave, change to this folder, then run:

```octave
demo_fgo();        % trajectory and error plots plus console metrics
test_fgo();        % numerical checks, no display required
result = demo_fgo(false);  % return data/results without opening a figure
```

From a terminal in this folder:

```sh
octave-cli --quiet --eval "test_fgo();"
octave --persist --eval "demo_fgo();"
```

The interactive plot needs a working Octave graphics toolkit and display.
No files are written by these functions. The data generator restores the caller's
random-number state. GNU Octave is the supported runtime; Octave-specific syntax
is used deliberately.

| File | Purpose |
| --- | --- |
| `demo_fgo.m` | Compare raw absolute measurements, Gaussian FGO, and robust FGO |
| `make_demo_data.m` | Generate the reproducible synthetic trajectory and observations |
| `solve_fgo.m` | Assemble sparse factors and solve using iteratively reweighted least squares |
| `test_fgo.m` | Check exact recovery, invariances, convergence, and regression behavior |

## Graph, states, and measurements

There are 120 epochs at one-second intervals. Each variable node is a 2D
position in a fixed local east/north frame, in metres:

$$
\mathbf p_k = [e_k, n_k]^T, \qquad
\mathbf x = [\mathbf p_1^T,\ldots,\mathbf p_N^T]^T.
$$

A unary factor anchors each position to a noisy absolute position measurement
$\mathbf z_k$. A binary factor connects adjacent positions using a measured
displacement $\mathbf d_k$:

```text
       z1          z2          z3        absolute measurements
       |           |           |
      [p1] --d1-- [p2] --d2-- [p3] ...   position variables and relative factors
```

Synthetic observations are generated as

$$
\mathbf z_k = \mathbf p_k^{\mathrm{true}} + \boldsymbol\epsilon_k,
\qquad
\mathbf d_k = \mathbf p_{k+1}^{\mathrm{true}}-\mathbf p_k^{\mathrm{true}}
               + \boldsymbol\eta_k,
$$

with per-axis standard deviations $\sigma_a=4$ m and $\sigma_d=0.15$ m.
Four absolute observations also receive an injected offset of $(28,-22)$ m.
The seed, noise levels, and Huber threshold are fixed teaching choices, not fitted
to benchmark ground truth. Truth is used to generate the simulation and evaluate
errors; it is never an input to `solve_fgo`.

Displacement is already in metres. If starting with velocity, one would first
use $\mathbf d_k\approx\mathbf v_k\Delta t_k$ and propagate its uncertainty.
Here there is no velocity state, clock state, latitude/longitude conversion,
IMU integration, or raw satellite measurement processing.

## Objective and uncertainty

Whiten the residuals by their standard deviations:

$$
\mathbf r_k^a = \frac{\mathbf p_k-\mathbf z_k}{\sigma_a},
\qquad
\mathbf r_k^d = \frac{\mathbf p_{k+1}-\mathbf p_k-\mathbf d_k}{\sigma_d}.
$$

The Gaussian solution minimizes

$$
F_G(\mathbf x)=\frac12\sum_{k=1}^{N}\|\mathbf r_k^a\|^2
             +\frac12\sum_{k=1}^{N-1}\|\mathbf r_k^d\|^2.
$$

This is the maximum a posteriori estimate with independent isotropic Gaussian
measurement errors and a flat state prior. Absolute factors fix the graph's
translation gauge; relative factors alone cannot determine absolute location.
Smaller standard deviations give factors greater influence.

To limit the influence of corrupted absolute positions, use a Huber loss on each
**2D residual norm**, with dimensionless threshold $\delta=1.5$:

$$
\rho_\delta(s)=
\begin{cases}
\tfrac12 s^2 & 0\leq s\leq\delta,\\
\delta(s-\tfrac12\delta) & s>\delta,
\end{cases}
\qquad
F_H(\mathbf x)=\sum_k\rho_\delta(\|\mathbf r_k^a\|)
              +\frac12\sum_k\|\mathbf r_k^d\|^2.
$$

Only absolute factors are robustified. Corrupted relative measurements, long
absolute outages, and correlated GNSS errors need additional modeling; this
example does not address them. Robust estimation does not guarantee improvement
on arbitrary data.

## Sparse solution and its relation to nonlinear FGO

Let $D$ have $-1$ on its diagonal and $+1$ on its next diagonal, so each row
computes the difference between consecutive epochs. The stacked whitened
residual is $A\mathbf x-\mathbf b$, where

$$
A=\begin{bmatrix}I_{2N}/\sigma_a\\(D\otimes I_2)/\sigma_d\end{bmatrix},
\qquad
\mathbf b=\begin{bmatrix}\operatorname{vec}(Z)/\sigma_a\\
\operatorname{vec}(M)/\sigma_d\end{bmatrix}.
$$

$Z$ and $M$ store absolute positions and displacements as columns, matching
Octave's column-major `(:)` operation. The Jacobians of the unwhitened unary
and binary factors are $I_2$ and $[-I_2, I_2]$, respectively.

For Huber iteratively reweighted least squares (IRLS), at iteration $j$ compute

$$
w_k^{(j)}=\min\left(1,\frac{\delta}{\|\mathbf r_k^a(\mathbf x^{(j)})\|}\right),
\qquad
\mathbf x^{(j+1)}=\arg\min_{\mathbf x}
\|W^{(j)}(A\mathbf x-\mathbf b)\|^2.
$$

At zero residual the weight is one. $W$ repeats $\sqrt{w_k}$ for the two
coordinates of each absolute factor and uses one for relative factors.
The solver uses sparse backslash, `(W*A) \ (W*b)`, rather than an explicit
matrix inverse. It stops when the infinity-norm update falls below
$10^{-8}(1+\|\mathbf x\|_\infty)$ or after 100 iterations, and returns its
convergence flag and objective history. Setting the threshold to `Inf`
disables robust weighting.

This example has linear measurement functions; Gaussian FGO is therefore a
linear least-squares problem. IRLS adds robust weighting, not satellite-range
nonlinearity. In a raw-GNSS nonlinear graph, one instead linearizes residuals
around the current state and solves for increments, for example

$$
\Delta\mathbf x=\arg\min_{\Delta\mathbf x}
\|W(J\Delta\mathbf x+\mathbf r(\mathbf x))\|^2,
\qquad \mathbf x\leftarrow\mathbf x+\Delta\mathbf x.
$$

Batch optimization uses all epochs together, so later observations can improve
earlier positions. It is an offline smoother rather than a causal filter.

## Inspiration and references

Suzuki's **2021 first-place** solution used accumulated delta range (ADR) for
relative positioning and reference-station-corrected pseudorange for absolute
constraints in FGO. His **2022 first-place** solution used two steps: velocity
and receiver clock drift estimation from Doppler, then position and clock bias
estimation with velocity/clock constraints, time-differenced carrier phase
(TDCP), and corrected pseudorange.

The conceptual connection here is combining weaker absolute information with
stronger relative information across a complete trajectory. Our synthetic
absolute positions stand in for absolute information; synthetic displacements
stand in for relative information. **They are not pseudorange or TDCP factors.**
The robust loss here is a teaching choice, not a claim about his exact code.

Primary references, checked 6 November 2026:

1. Taro Suzuki, “First Place Award Winner of the Smartphone Decimeter Challenge:
   Global Optimization of Position and Velocity by Factor Graph Optimization,”
   ION GNSS+ 2021, pp. 2974–2985.
   [ION abstract](https://www.ion.org/publications/abstract.cfm?articleID=18109),
   [DOI](https://doi.org/10.33012/2021.18109).
2. Taro Suzuki, “1st Place Winner of the Smartphone Decimeter Challenge:
   Two-Step Optimization of Velocity and Position Using Smartphone’s Carrier
   Phase Observations,” ION GNSS+ 2022, pp. 2276–2286.
   [ION abstract](https://www.ion.org/publications/abstract.cfm?articleID=18377),
   [DOI](https://doi.org/10.33012/2022.18377).
3. Taro Suzuki, “Precise Position Estimation Using Smartphone Raw GNSS Data
   Based on Two-Step Optimization,” Sensors 23(3), 1205, 2023.
   [Open-access article](https://www.mdpi.com/1424-8220/23/3/1205).

The competition name does not establish decimetre accuracy for this demo, and
none of these synthetic results establishes physical Samsung S23+ performance.

## Verification and limits

On GNU Octave 11.3.0, `test_fgo()` passed exact noiseless recovery, convergence,
non-increasing robust objective, translation and rotation invariance, and a
fixed-fixture regression comparing robust and Gaussian FGO. The synthetic
horizontal RMSE was 8.553 m for absolute measurements, 1.025 m for Gaussian FGO,
and 0.578 m for Huber FGO (9 IRLS iterations). RMSE here is

$$
\mathrm{RMSE}_{2D}=\sqrt{\frac1N\sum_k\|\widehat{\mathbf p}_k-\mathbf p_k^{true}\|^2}.
$$

This is not the challenge's percentile-based scoring metric. Simulation noise
assumptions are substantially simpler than real smartphone observations.

Android public-data replay and UI viewport/font validation are **incomplete**:
this standalone synthetic Octave demonstration is not integrated into Android
and supplies no Android playback evidence. It does not satisfy the repository's
Android feature acceptance criteria or validate the production estimator.

Plot verification was attempted but is **incomplete**: the installed Octave
reported `no graphics toolkits are available!`. Use `demo_fgo(false)` in this
environment. Numerical tests exited with status 0; Octave also printed
`ignoring const execution_exception& while preparing to exit` during shutdown.
