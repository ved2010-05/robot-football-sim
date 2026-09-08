"""
Motor catalogue and electrical model.

DERIVATION APPROACH
-------------------
Every entry stores what the vendor actually publishes for the assembled
gearmotor, measured AT THE OUTPUT SHAFT:

    nominal voltage, no-load output speed, stall torque, stall current,
    no-load current, encoder counts per motor revolution, gear ratio.

The electrical constants are then derived at the output shaft directly:

    R  = V_nom / I_stall                    winding resistance
    Ke = (V_nom - I_noload * R) / w_noload  back-EMF constant  [V.s/rad]
    Kt = tau_stall / I_stall                torque constant    [N.m/A]

This deliberately lumps the gearbox into Ke and Kt. The payoff is that the
model reproduces BOTH published operating points exactly -- it stalls at the
quoted torque and free-runs at the quoted speed -- with no assumed gearbox
efficiency anywhere. Deriving from bare-motor specs instead would force an
efficiency fudge factor and would mix Pololu's spur- and helical-pinion
generations, which have genuinely different bare motors.

Torque available at output speed w, at applied voltage V:

    I    = (V - Ke * w) / R
    tau  = Kt * I

Sanity: at w=0, V=V_nom this gives exactly tau_stall. At w=w_noload it gives
Kt*I_noload, a small residual that goes into overcoming no-load friction.

Note that Kt != Ke numerically here. For an ideal lossless motor they would
be equal. Real published figures never satisfy that, because stall torque is
measured (or conservatively extrapolated) while no-load speed is measured
under different conditions. Forcing them equal would break one of the two
operating points. Keeping them separate is the honest choice.

WHAT IS QUOTED VS WHAT IS ESTIMATED
-----------------------------------
Every entry carries `verified`. True means the performance figures come from
the vendor's published specifications. False means the torque figure is
estimated (from the base motor and an assumed gearbox efficiency) because the
vendor page was not reachable at build time -- check the datasheet before
relying on it.

Rotor inertia is NEVER published by any vendor in this class. It is estimated
from can geometry for all entries, and it matters: reflected through the gear
ratio as J * N^2, it is the dominant source of drivetrain lag.

Sources:
  Pololu 37D 12V range   https://www.pololu.com/category/271/12v-37d-metal-gearmotors
  goBILDA 5203 series    https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
  Pololu micro metal N20 https://www.pololu.com/product/5137
"""

import math
from dataclasses import dataclass, field

# Unit helpers -------------------------------------------------------------
KGCM_TO_NM = 0.0980665
OZIN_TO_NM = 0.00706155


def rpm_to_rads(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


@dataclass
class Motor:
    """A gearmotor, described by its published output-shaft performance."""

    key: str
    name: str
    nominal_v: float
    noload_rpm: float          # OUTPUT shaft, at nominal_v
    stall_torque_nm: float     # OUTPUT shaft, at nominal_v
    stall_current_a: float
    noload_current_a: float
    gear_ratio: float
    encoder_cpr_motor: float   # counts per MOTOR revolution (quadrature x4)
    rotor_inertia_kgm2: float  # ESTIMATED, see module docstring
    verified: bool
    source: str
    note: str = ""

    # Derived, filled in __post_init__
    resistance_ohm: float = field(init=False)
    ke: float = field(init=False)   # V.s/rad at the OUTPUT shaft
    kt: float = field(init=False)   # N.m/A  at the OUTPUT shaft
    noload_rads: float = field(init=False)
    inertia_output_kgm2: float = field(init=False)
    encoder_cpr_output: float = field(init=False)

    def __post_init__(self) -> None:
        self.resistance_ohm = self.nominal_v / self.stall_current_a
        self.noload_rads = rpm_to_rads(self.noload_rpm)
        back_emf_at_noload = (
            self.nominal_v - self.noload_current_a * self.resistance_ohm
        )
        self.ke = back_emf_at_noload / self.noload_rads
        self.kt = self.stall_torque_nm / self.stall_current_a

        # Rotor inertia reflected through the gearbox dominates everything
        # else in the drivetrain. This is why a high-ratio gearmotor feels
        # sluggish even when it has plenty of torque.
        self.inertia_output_kgm2 = (
            self.rotor_inertia_kgm2 * self.gear_ratio ** 2
        )
        self.encoder_cpr_output = self.encoder_cpr_motor * self.gear_ratio

    # -- model ------------------------------------------------------------

    def torque_at(self, omega_rads: float, voltage: float) -> float:
        """Steady-state output torque at a given shaft speed and voltage.

        Signed: negative voltage gives negative torque. Current is not
        limited here; the caller applies traction and current limits.
        """
        current = (voltage - self.ke * omega_rads) / self.resistance_ohm
        return self.kt * current

    def current_at(self, omega_rads: float, voltage: float) -> float:
        return (voltage - self.ke * omega_rads) / self.resistance_ohm

    def max_speed_rads(self, voltage: float) -> float:
        return voltage / self.ke

    def counts_per_output_rad(self) -> float:
        return self.encoder_cpr_output / (2.0 * math.pi)

    def describe(self) -> str:
        tag = "verified" if self.verified else "ESTIMATED torque"
        return (
            f"{self.name}\n"
            f"  {self.noload_rpm:.0f} rpm no-load, "
            f"{self.stall_torque_nm / KGCM_TO_NM:.1f} kg-cm stall, "
            f"{self.stall_current_a:.1f} A stall  [{tag}]\n"
            f"  R={self.resistance_ohm:.3f} ohm  "
            f"Ke={self.ke:.4f} V.s/rad  Kt={self.kt:.4f} N.m/A\n"
            f"  gear {self.gear_ratio:.1f}:1, "
            f"{self.encoder_cpr_output:.0f} counts/output-rev, "
            f"J_out={self.inertia_output_kgm2:.2e} kg.m^2 (estimated)"
        )


# ---------------------------------------------------------------------------
# Estimated rotor inertias, from can size and typical rotor mass.
# J ~= 0.5 * m_rotor * r_rotor^2
# ---------------------------------------------------------------------------
_J_RS545 = 3.0e-6      # Pololu 37D class, ~24 mm rotor, ~50 g
_J_RS555 = 3.5e-6      # goBILDA 5203 class, slightly larger
_J_N20 = 3.0e-8        # N20 micro, ~8 mm rotor, ~3 g

_POLOLU = "https://www.pololu.com/category/271/12v-37d-metal-gearmotors"
_GOBILDA = "https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/"
_N20SRC = "https://www.pololu.com/product/5137"

# goBILDA does not publish stall torque on the pages reachable here, so those
# entries are estimated from the 1:1 base motor (RS-555: 6000 rpm, 9.2 A
# stall, 0.25 A no-load at 12 V) assuming 72% planetary gearbox efficiency:
#     Ke_base   = (12 - 0.25 * 12/9.2) / (6000 rpm in rad/s) = 0.01859
#     tau_base  = Ke_base * 9.2 = 0.171 N.m       (assuming Kt ~ Ke on the bare motor)
#     tau_out   = 0.171 * ratio * 0.72
_GB_BASE_TORQUE_NM = 0.171
_GB_EFFICIENCY = 0.72


def _gobilda_torque(ratio: float) -> float:
    return _GB_BASE_TORQUE_NM * ratio * _GB_EFFICIENCY


CATALOG: dict[str, Motor] = {}


def _add(m: Motor) -> None:
    CATALOG[m.key] = m


_add(Motor(
    key="pololu_37d_19_1",
    name="Pololu 37Dx52L 12V, 19:1, 64 CPR encoder",
    nominal_v=12.0, noload_rpm=500.0,
    stall_torque_nm=5.0 * KGCM_TO_NM,      # 84 oz-in published
    stall_current_a=5.0, noload_current_a=0.30,
    gear_ratio=19.0, encoder_cpr_motor=64.0,
    rotor_inertia_kgm2=_J_RS545,
    verified=True, source=_POLOLU,
    note="Good default: fast enough for football with 40 mm wheels.",
))

_add(Motor(
    key="pololu_37d_30_1",
    name="Pololu 37Dx52L 12V, 30:1, 64 CPR encoder",
    nominal_v=12.0, noload_rpm=350.0,
    stall_torque_nm=8.0 * KGCM_TO_NM,      # 110 oz-in published
    stall_current_a=5.0, noload_current_a=0.30,
    gear_ratio=30.0, encoder_cpr_motor=64.0,
    rotor_inertia_kgm2=_J_RS545,
    verified=True, source=_POLOLU,
    note="More torque, noticeably slower top speed.",
))

_add(Motor(
    key="pololu_37d_50_1",
    name="Pololu 37Dx70L 12V, 50:1, 64 CPR encoder (helical pinion)",
    nominal_v=12.0, noload_rpm=200.0,
    stall_torque_nm=21.0 * KGCM_TO_NM,
    stall_current_a=5.5, noload_current_a=0.20,
    gear_ratio=50.0, encoder_cpr_motor=64.0,
    rotor_inertia_kgm2=_J_RS545,
    verified=True, source=_POLOLU,
    note="Strong but slow. Reflected inertia is large -- sluggish response.",
))

_add(Motor(
    key="pololu_37d_100_1",
    name="Pololu 37Dx57L 12V, 100:1, 64 CPR encoder",
    nominal_v=12.0, noload_rpm=100.0,
    stall_torque_nm=34.0 * KGCM_TO_NM,
    stall_current_a=5.5, noload_current_a=0.20,
    gear_ratio=100.0, encoder_cpr_motor=64.0,
    rotor_inertia_kgm2=_J_RS545,
    verified=True, source=_POLOLU,
    note="Too slow for football. Included for comparison.",
))

_add(Motor(
    key="gobilda_5203_5_2",
    name="goBILDA 5203 Yellow Jacket, 5.2:1",
    nominal_v=12.0, noload_rpm=1150.0,
    stall_torque_nm=_gobilda_torque(5.2),
    stall_current_a=9.2, noload_current_a=0.25,
    gear_ratio=5.2, encoder_cpr_motor=28.0,
    rotor_inertia_kgm2=_J_RS555,
    verified=False, source=_GOBILDA,
    note="Very fast; needs small wheels or it is uncontrollable.",
))

_add(Motor(
    key="gobilda_5203_13_7",
    name="goBILDA 5203 Yellow Jacket, 13.7:1",
    nominal_v=12.0, noload_rpm=435.0,
    stall_torque_nm=_gobilda_torque(13.7),
    stall_current_a=9.2, noload_current_a=0.25,
    gear_ratio=13.7, encoder_cpr_motor=28.0,
    rotor_inertia_kgm2=_J_RS555,
    verified=False, source=_GOBILDA,
    note="Strong all-rounder. More power than the 37D range.",
))

_add(Motor(
    key="gobilda_5203_19_2",
    name="goBILDA 5203 Yellow Jacket, 19.2:1",
    nominal_v=12.0, noload_rpm=312.0,
    stall_torque_nm=_gobilda_torque(19.2),
    stall_current_a=9.2, noload_current_a=0.25,
    gear_ratio=19.2, encoder_cpr_motor=28.0,
    rotor_inertia_kgm2=_J_RS555,
    verified=False, source=_GOBILDA,
    note="Torquey; will be traction-limited on most surfaces.",
))

_add(Motor(
    key="n20_12v_200",
    name="N20 micro gearmotor 12V, ~50:1, magnetic encoder",
    nominal_v=12.0, noload_rpm=200.0,
    stall_torque_nm=2.0 * KGCM_TO_NM,
    stall_current_a=1.2, noload_current_a=0.12,
    gear_ratio=50.0, encoder_cpr_motor=28.0,
    rotor_inertia_kgm2=_J_N20,
    verified=False, source=_N20SRC,
    note="Only for a much smaller/lighter robot than the default config.",
))


def get(key: str) -> Motor:
    if key not in CATALOG:
        available = "\n  ".join(sorted(CATALOG))
        raise KeyError(
            f"Unknown motor {key!r}. Available:\n  {available}"
        )
    return CATALOG[key]


def list_motors() -> str:
    lines = []
    for key in CATALOG:
        lines.append(f"[{key}]\n{CATALOG[key].describe()}")
        if CATALOG[key].note:
            lines.append(f"  note: {CATALOG[key].note}")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    print(list_motors())
