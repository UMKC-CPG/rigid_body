"""analysis: conservation monitoring and analytic solutions.

This subpackage judges and complements the numerical motion (ARCHITECTURE
Section 3.4). The conservation monitor watches the two invariants for
drift and reports it live; the analytic solutions supply the closed-form
motions that serve both as an on-screen overlay beside the simulation and
as the oracle the tests measure the integrator against.
"""
