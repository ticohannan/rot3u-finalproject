"""SP-10: overall stack rate for a placement-error model."""
import math

from stackpieces.placement import radial_cdf, stack_rate


def test_step_curve_uniform():
    # seats always within r0, never beyond: fraction = area of the disk inside the square
    h, r0 = 5.0, 3.0
    rate = stack_rate([0, r0, r0 + 1e-9], [1, 1, 0], dict(model="uniform", resolution_mm=2 * h))
    assert abs(rate - math.pi * r0 ** 2 / (4 * h * h)) < 0.005


def test_gaussian_cdf():
    s = 2.0
    p = radial_cdf(dict(model="gaussian", sigma_mm=s), [s])[0]     # Rayleigh: 1 - exp(-1/2)
    assert abs(p - (1 - math.exp(-0.5))) < 0.01
