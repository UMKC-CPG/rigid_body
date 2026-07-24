"""
Spike: verify the closed-form torque-free asymmetric-top solution.

DESIGN.md 8.4 records the exact solution of Euler's equations for a
freely rotating body with three distinct principal moments, written in
Jacobi elliptic functions.  Those formulas were reproduced there from
memory, and 8.4 itself insists they be checked numerically before the
code trusts them -- the same discipline that caught a factor-of-two error
in the icosahedron constant of 3.2.

This script performs that check.  For a chosen body and initial spin it
evaluates the analytic angular velocity omega(t) and, independently,
integrates Euler's equations at high accuracy, then reports the largest
disagreement between the two.  A transcription error in any coefficient,
in the elliptic modulus, or in the time scale would show up as an
order-one discrepancy; genuine agreement runs near the integrator's own
tolerance.

Three claims of 8.4 are exercised:

  1. The main branch of the solution -- spin nearest the largest-moment
     axis, where |L|^2 > 2 T I_2.
  2. The complementary branch -- spin nearest the smallest-moment axis,
     |L|^2 < 2 T I_2 -- which 8.4 obtains by exchanging axes 1 and 3.
  3. The intermediate-axis growth rate sigma of 4.4, which 8.4 claims is
     the departure exponent of the separatrix dividing those two
     branches, tying the linear result of 4.4 to the elliptic solution.

It needs only numpy and scipy:

    python3 dev/spikes/free_top_elliptic.py
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.special import ellipj, ellipk


# Pointwise agreement this good means the formula is right; a wrong
# coefficient or modulus would miss by order unity, not by 1e-6.
POINTWISE_TOLERANCE = 1.0e-6

# The growth-rate fit is a least-squares slope and carries a little more
# noise than a pointwise comparison, so it earns a looser bound.
GROWTH_RATE_TOLERANCE = 5.0e-3


def euler_derivative(time, angular_velocity_body, principal_moments):
    """Torque-free Euler equations in body components (DESIGN.md 4.3).

    Returns d(omega)/dt for the angular velocity ``angular_velocity_body``
    and the three principal moments.  With no torque acting this is the
    pure gyroscopic term ``-omega x (I omega)`` divided through by I.
    """
    moment_1, moment_2, moment_3 = principal_moments
    omega_1, omega_2, omega_3 = angular_velocity_body
    rate_1 = (moment_2 - moment_3) / moment_1 * omega_2 * omega_3
    rate_2 = (moment_3 - moment_1) / moment_2 * omega_3 * omega_1
    rate_3 = (moment_1 - moment_2) / moment_3 * omega_1 * omega_2
    return [rate_1, rate_2, rate_3]


def conserved_quantities(angular_velocity_body, principal_moments):
    """Return (twice_kinetic_energy, angular_momentum_squared).

    Both are constant under torque-free motion (DESIGN.md 2.5), and the
    pair ``2T`` and ``|L|^2`` is exactly what parameterises the analytic
    solution of 8.4.
    """
    moments = np.asarray(principal_moments, dtype=float)
    omega = np.asarray(angular_velocity_body, dtype=float)
    twice_kinetic_energy = float(np.sum(moments * omega**2))
    angular_momentum_squared = float(np.sum((moments * omega)**2))
    return twice_kinetic_energy, angular_momentum_squared


def elliptic_parameters(principal_moments, initial_spin):
    """Branch parameters for the analytic free-top solution (8.4).

    ``principal_moments`` must be strictly ordered I_1 < I_2 < I_3, and
    ``initial_spin`` must have a zero middle component -- omega_2(0) = 0 --
    with non-negative outer components.  That is the phase where the
    elliptic functions start from (sn, cn, dn) = (0, 1, 1), which is where
    8.4's formulas are anchored.

    Returns a dict holding the three amplitude coefficients on axes
    1, 2, 3; the name of the elliptic function riding each axis; the time
    scale ``rate``; the squared modulus ``modulus_sq``; and the
    body-frame ``period``.  Keeping the parameters separate from their
    evaluation lets the caller learn the period before it builds a time
    grid, and guarantees the solution and its period cannot disagree.
    """
    moment_1, moment_2, moment_3 = (float(m) for m in principal_moments)
    assert moment_1 < moment_2 < moment_3, "moments must be I1 < I2 < I3"
    assert abs(initial_spin[1]) < 1.0e-12, "middle component must be zero"

    two_ke, l_sq = conserved_quantities(initial_spin, principal_moments)

    if l_sq - two_ke * moment_2 >= 0.0:
        # Main branch: the spin is nearest axis 3 (the largest moment).
        # omega_1 rides cn, omega_2 rides sn, omega_3 rides dn.
        coefficients = [
            np.sqrt((two_ke * moment_3 - l_sq)
                    / (moment_1 * (moment_3 - moment_1))),
            np.sqrt((two_ke * moment_3 - l_sq)
                    / (moment_2 * (moment_3 - moment_2))),
            np.sqrt((l_sq - two_ke * moment_1)
                    / (moment_3 * (moment_3 - moment_1))),
        ]
        axis_functions = ["cn", "sn", "dn"]
        rate = np.sqrt((moment_3 - moment_2) * (l_sq - two_ke * moment_1)
                       / (moment_1 * moment_2 * moment_3))
        modulus_sq = ((moment_2 - moment_1) * (two_ke * moment_3 - l_sq)
                      / ((moment_3 - moment_2)
                         * (l_sq - two_ke * moment_1)))
    else:
        # Complementary branch: spin nearest axis 1 (the smallest
        # moment); DESIGN.md 8.4 with axes 1 and 3 exchanged.  Now
        # omega_1 rides dn, omega_2 rides sn, omega_3 rides cn.
        coefficients = [
            np.sqrt((l_sq - two_ke * moment_3)
                    / (moment_1 * (moment_1 - moment_3))),
            np.sqrt((two_ke * moment_1 - l_sq)
                    / (moment_2 * (moment_1 - moment_2))),
            np.sqrt((two_ke * moment_1 - l_sq)
                    / (moment_3 * (moment_1 - moment_3))),
        ]
        axis_functions = ["dn", "sn", "cn"]
        rate = np.sqrt((moment_1 - moment_2) * (l_sq - two_ke * moment_3)
                       / (moment_1 * moment_2 * moment_3))
        modulus_sq = ((moment_2 - moment_3) * (two_ke * moment_1 - l_sq)
                      / ((moment_1 - moment_2)
                         * (l_sq - two_ke * moment_3)))

    # The body-frame motion repeats every 4 K(k) in the scaled time,
    # because sn and cn each have quarter-period K (DESIGN.md 8.4).
    period = 4.0 * ellipk(modulus_sq) / rate

    return {
        "coefficients": coefficients,
        "axis_functions": axis_functions,
        "rate": rate,
        "modulus_sq": modulus_sq,
        "period": period,
    }


def evaluate_analytic(parameters, times):
    """Evaluate the analytic omega(t) on a time grid from parameters.

    ``parameters`` is the dict returned by ``elliptic_parameters``.  The
    Jacobi functions are computed once at the scaled time and then each
    axis picks out the one that rides it.  Returns an array of shape
    (len(times), 3).
    """
    scaled_time = parameters["rate"] * np.asarray(times, dtype=float)
    sn, cn, dn, _phase = ellipj(scaled_time, parameters["modulus_sq"])
    function_table = {"sn": sn, "cn": cn, "dn": dn}

    columns = []
    for axis in range(3):
        coefficient = parameters["coefficients"][axis]
        function_name = parameters["axis_functions"][axis]
        columns.append(coefficient * function_table[function_name])
    return np.column_stack(columns)


def integrate_euler(principal_moments, initial_spin, times):
    """High-accuracy numerical solution of the same free motion.

    Integrates the torque-free Euler equations with an eighth-order
    Runge-Kutta method (DOP853) at tight tolerance, sampled at ``times``.
    This is the reference the analytic formula is checked against; its own
    error is far smaller than any transcription mistake we are hunting.
    """
    solution = solve_ivp(
        euler_derivative,
        t_span=(float(times[0]), float(times[-1])),
        y0=list(initial_spin),
        t_eval=times,
        args=(principal_moments,),
        method="DOP853",
        rtol=1.0e-12,
        atol=1.0e-13,
    )
    assert solution.success, "reference integration failed"
    return solution.y.T


def verify_branch(label, principal_moments, initial_spin, periods=3):
    """Compare analytic and numerical omega for one initial condition.

    Builds a time grid spanning several full body-frame periods, forms
    both solutions on it, and reports the largest disagreement along with
    how well the analytic solution conserves 2T and |L|^2 on its own.
    Returns the peak relative deviation so the caller can pass or fail.
    """
    parameters = elliptic_parameters(principal_moments, initial_spin)
    span = periods * parameters["period"]
    times = np.linspace(0.0, span, 4000)

    omega_analytic = evaluate_analytic(parameters, times)
    omega_numeric = integrate_euler(principal_moments, initial_spin, times)

    difference = np.abs(omega_analytic - omega_numeric)
    velocity_scale = float(np.max(np.abs(omega_numeric)))
    max_relative_deviation = float(np.max(difference)) / velocity_scale

    # Independent of the numerics: the analytic solution must itself hold
    # the two invariants that define it constant along the whole run.
    two_ke_initial, l_sq_initial = conserved_quantities(
        initial_spin, principal_moments)
    moments = np.asarray(principal_moments, dtype=float)
    two_ke_history = np.sum(moments * omega_analytic**2, axis=1)
    l_sq_history = np.sum((moments * omega_analytic)**2, axis=1)
    invariant_drift = max(
        float(np.max(np.abs(two_ke_history - two_ke_initial)))
        / two_ke_initial,
        float(np.max(np.abs(l_sq_history - l_sq_initial)))
        / l_sq_initial,
    )

    print(f"  {label}")
    print(f"    modulus k^2             : "
          f"{parameters['modulus_sq']:.6f}")
    print(f"    body-frame period       : "
          f"{parameters['period']:.6f}")
    print(f"    max relative deviation  : "
          f"{max_relative_deviation:.3e}")
    print(f"    analytic invariant drift: {invariant_drift:.3e}")
    print()
    return max_relative_deviation


def verify_growth_rate(principal_moments, spin_rate=2.0,
                       perturbation=1.0e-4):
    """Check the 4.4 intermediate-axis growth rate against numerics.

    Spins the body almost purely about the intermediate axis (axis 2 in
    the sorted convention) with a tiny transverse perturbation, then fits
    the exponential growth of that perturbation in the linear regime and
    compares the fitted rate against

        sigma = Omega sqrt((I3 - I2)(I2 - I1) / (I1 I3)).

    DESIGN.md 8.4 identifies this sigma with the separatrix's departure
    exponent, so agreement links the linear picture of 4.4 to the
    elliptic solution.  Returns the relative error of the fitted rate.
    """
    moment_1, moment_2, moment_3 = (float(m) for m in principal_moments)
    predicted_rate = spin_rate * np.sqrt(
        (moment_3 - moment_2) * (moment_2 - moment_1)
        / (moment_1 * moment_3))

    initial_spin = [perturbation, spin_rate, 0.0]
    # Integrate several e-folding times, but read the slope only from the
    # tail, where the growing cosh has become a clean exponential and the
    # perturbation is still far smaller than the spin (linear regime).
    span = 6.0 / predicted_rate
    times = np.linspace(0.0, span, 4000)
    omega = integrate_euler(principal_moments, initial_spin, times)

    transverse_amplitude = np.hypot(omega[:, 0], omega[:, 2])
    tail = times > 0.6 * span
    fitted_rate = np.polyfit(
        times[tail], np.log(transverse_amplitude[tail]), 1)[0]
    relative_error = abs(fitted_rate - predicted_rate) / predicted_rate

    print("  intermediate-axis growth rate (4.4 <-> 8.4)")
    print(f"    predicted sigma         : {predicted_rate:.6f}")
    print(f"    fitted sigma            : {fitted_rate:.6f}")
    print(f"    relative error          : {relative_error:.3e}")
    print()
    return relative_error


def main():
    """Run every check and report a single pass/fail verdict."""
    print("Verifying the torque-free asymmetric-top solution (8.4)\n")

    body = (2.0, 3.0, 4.0)
    outcomes = []

    # Main branch: most of the spin sits on axis 3, the largest moment.
    outcomes.append((
        "main branch",
        verify_branch("main branch (spin near axis 3)",
                      body, [0.3, 0.0, 2.0]),
        POINTWISE_TOLERANCE,
    ))

    # Complementary branch: most of the spin sits on axis 1, the
    # smallest moment; this exercises 8.4's axis-1/axis-3 exchange.
    outcomes.append((
        "complementary branch",
        verify_branch("complementary branch (spin near axis 1)",
                      body, [2.0, 0.0, 0.3]),
        POINTWISE_TOLERANCE,
    ))

    # The separatrix exponent shared with 4.4.
    outcomes.append((
        "growth rate",
        verify_growth_rate(body),
        GROWTH_RATE_TOLERANCE,
    ))

    print("-" * 60)
    all_passed = True
    for name, measured, tolerance in outcomes:
        passed = measured <= tolerance
        all_passed = all_passed and passed
        status = "PASS" if passed else "FAIL"
        print(f"  {status}  {name:<22} "
              f"{measured:.3e} <= {tolerance:.0e}")
    print("-" * 60)
    print("ALL CHECKS PASSED" if all_passed else "SOME CHECKS FAILED")


if __name__ == "__main__":
    main()
