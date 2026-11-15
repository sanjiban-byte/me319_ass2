"""Engineering schematic and closed-loop block diagram for the ME319 A2 paper.

Run:  python3 make_diagrams.py   (matplotlib >= 3.5). Writes PNG (300 dpi),
PDF and SVG versions of both figures next to this file. All numbers are the
frozen design values (D18, D27, D30, D38, D39).
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle, Polygon, Wedge, Arc, FancyArrowPatch
import numpy as np

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8.5, "mathtext.fontset": "dejavusans"})

C_EST = "#e8f1fb"   # estimation / prediction
C_REF = "#eef7ea"   # reference shaping
C_FB = "#fdf1e3"    # feedback controller
C_PL = "#f2f2f2"    # plant / physical
C_FIRE = "#fbe9ee"  # firing logic
EDGE = "#333333"


def box(ax, x, y, w, h, title, body="", fc="w", fs_title=8.6, fs_body=7.4):
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                       fc=fc, ec=EDGE, lw=1.0, zorder=2)
    ax.add_patch(p)
    if body:
        ax.text(x + w / 2, y + h * 0.70, title, ha="center", va="center", fontsize=fs_title,
                fontweight="bold", zorder=3)
        ax.text(x + w / 2, y + h * 0.32, body, ha="center", va="center", fontsize=fs_body,
                zorder=3, linespacing=1.25)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=fs_title,
                fontweight="bold", zorder=3)
    return (x, y, w, h)


def arrow(ax, p0, p1, label="", lpos=0.5, dy=0.11, fs=7.6, color=EDGE, ha="center", ls="-"):
    a = FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=9, lw=1.0, color=color,
                        zorder=1, shrinkA=0, shrinkB=0, linestyle=ls)
    ax.add_patch(a)
    if label:
        x = p0[0] + (p1[0] - p0[0]) * lpos
        y = p0[1] + (p1[1] - p0[1]) * lpos
        ax.text(x, y + dy, label, ha=ha, va="bottom", fontsize=fs, color=color, zorder=3)


def line(ax, pts, color=EDGE, ls="-"):
    pts = np.array(pts)
    ax.plot(pts[:, 0], pts[:, 1], color=color, lw=1.0, zorder=1, linestyle=ls)


def save(fig, stem):
    for ext in ("png", "pdf", "svg"):
        fig.savefig(f"{stem}.{ext}", dpi=300, bbox_inches="tight", facecolor="white")
    plt.close(fig)


# =====================================================================
# 1. Closed-loop block diagram
# =====================================================================
def block_diagram():
    fig, ax = plt.subplots(figsize=(13.5, 5.0))
    ax.set_xlim(0, 27.0); ax.set_ylim(-0.9, 9.2); ax.axis("off")
    yM = 5.6; h = 2.0
    T, B = 8.0, 6.8                                   # title / body font sizes
    # target and radar
    box(ax, 0.1, yM - h / 2, 2.3, h, "Radar", "20 Hz: $r_m$, $\\theta_m$\n(+ noise)\nmeas_valid", C_PL, T, B)
    ax.text(1.25, yM + 1.75, "V-44 target $(x, y)$", ha="center", va="center", fontsize=B)
    arrow(ax, (1.25, yM + 1.5), (1.25, yM + h / 2))
    # estimator
    box(ax, 3.3, yM - h / 2, 3.6, h, "State estimation",
        "Kalman filter in $(x, y)$\nSinger model, $\\tau_a$ = 1 s\nout: $\\hat\\theta, \\hat r, \\dot{\\hat\\theta}, \\sigma_{cross}$", C_EST, T, B)
    arrow(ax, (2.4, yM), (3.3, yM))
    # predictor
    box(ax, 7.9, yM - h / 2, 3.2, h, "Target prediction",
        "$\\theta_{pred} = \\hat\\theta + t_p\\,\\dot{\\hat\\theta}$\n$t_p$ = 0.5 s, order $n$ = 1", C_EST, T, B)
    arrow(ax, (6.9, yM), (7.9, yM))
    # prefilter + clamp
    box(ax, 12.1, yM - h / 2, 3.9, h, "Prefilter $F(z)$ + clamp",
        "$x_k = \\alpha\\,x_{k-1} + \\beta\\,u_k + \\beta_1 u_{k-1}$\npole 0.52 s, zero 0.145 s\nclamp to $-10°$ ... $85°$", C_REF, T, B)
    arrow(ax, (11.1, yM), (12.1, yM), "$\\theta_{pred}$", fs=B)
    # summing junction
    sx, sy, sr = 17.1, yM, 0.3
    ax.add_patch(Circle((sx, sy), sr, fc="w", ec=EDGE, lw=1.0, zorder=2))
    ax.plot([sx - sr * 0.7, sx + sr * 0.7], [sy, sy], color=EDGE, lw=0.8, zorder=3)
    ax.plot([sx, sx], [sy - sr * 0.7, sy + sr * 0.7], color=EDGE, lw=0.8, zorder=3)
    ax.text(sx - 0.62, sy - 0.62, "+", fontsize=9); ax.text(sx + 0.12, sy - 0.75, "−", fontsize=10)
    arrow(ax, (16.0, yM), (sx - sr, yM), "$\\theta_{ref}$", fs=B)
    # K_P
    box(ax, 18.1, yM - 0.65, 1.5, 1.3, "$K_P$", "8012 V/rad", C_FB, 9.0, B)
    arrow(ax, (sx + sr, yM), (18.1, yM), "$e$", dy=0.08, fs=B)
    # saturation
    sbx, sbw = 20.2, 1.3
    ax.add_patch(FancyBboxPatch((sbx, yM - 0.65), sbw, 1.3, boxstyle="round,pad=0.02,rounding_size=0.06",
                                fc=C_FB, ec=EDGE, lw=1.0, zorder=2))
    xs = np.array([-0.45, -0.17, 0.17, 0.45]) + sbx + sbw / 2
    ys = np.array([-0.25, -0.25, 0.25, 0.25]) + yM + 0.12
    ax.plot(xs, ys, color=EDGE, lw=1.0, zorder=3)
    ax.text(sbx + sbw / 2, yM - 0.5, "±600 V", ha="center", fontsize=B)
    arrow(ax, (19.6, yM), (sbx, yM))
    # notch
    box(ax, 22.1, yM - 0.65, 2.2, 1.3, "Notch $N(z)$", "56 rad/s mode:\n$\\zeta$ 0.094 → 0.7", C_FB, T, B - 0.3)
    arrow(ax, (sbx + sbw, yM), (22.1, yM))
    # plant
    box(ax, 24.9, yM - h / 2, 2.0, h, "Turret", "$G_p(s)$\nmotor, 100:1\ngear, turret", C_PL, T, B)
    arrow(ax, (24.3, yM), (24.9, yM))
    ax.text(24.6, yM + 0.12, "$V_m$", ha="center", va="bottom", fontsize=B)
    # controller bracket and notes
    ax.add_patch(Rectangle((16.6, yM - 1.15), 8.0, 2.55, fill=False, ec="#999999", lw=0.8, ls="--", zorder=0))
    ax.text(20.6, yM + 1.12, "updated on a 50 ms grid, zero-order hold to the plant", ha="center",
            fontsize=B - 0.3, style="italic", color="#555555")
    ax.text(20.6, yM - 1.55, "Feedback controller $G_c = K_P\\,N$: LTI, type 1, $K_{v,fb}$ = 8 s$^{-1}$",
            ha="center", fontsize=B, color="#444444")
    # output and feedback
    line(ax, [(26.9, yM), (26.95, yM), (26.95, 2.9), (sx, 2.9)])
    arrow(ax, (sx, 2.9), (sx, sy - sr))
    ax.text(26.8, 3.05, "$\\theta_{turret}$, $\\dot\\theta_{turret}$", ha="right", va="bottom", fontsize=B)
    # firing logic
    box(ax, 6.6, 0.05, 9.0, 1.95, "Firing logic",
        "pulse if valid, $|d| < R_{eng}$ and $P_{hit} = \\Phi\\left(\\frac{R_{eng}-d}{\\sigma}\\right) - \\Phi\\left(\\frac{-R_{eng}-d}{\\sigma}\\right) \\geq 0.95$,\n"
        "$d = \\hat r\\,(\\theta_{turret}-\\hat\\theta)$, $\\sigma = \\sigma_{cross}$, and $\\geq$ 1.01 s since last pulse",
        C_FIRE, T, B - 0.5)
    line(ax, [(5.1, yM - h / 2), (5.1, 1.0)])
    arrow(ax, (5.1, 1.0), (6.6, 1.0))
    ax.text(5.25, 1.15, "$\\hat\\theta, \\hat r, \\dot{\\hat\\theta}, \\sigma_{cross}$", fontsize=B)
    line(ax, [(sx + 1.2, 2.9), (sx + 1.2, 1.0)])
    arrow(ax, (sx + 1.2, 1.0), (15.6, 1.0))
    arrow(ax, (11.1, 0.05), (11.1, -0.55))
    ax.text(11.3, -0.55, "fire_cmd (0→1 pulse) → laser", fontsize=B, va="center")
    # re-initialisation path
    line(ax, [(4.6, yM + h / 2), (4.6, 8.25), (14.05, 8.25)], ls="--")
    arrow(ax, (14.05, 8.25), (14.05, yM + h / 2), ls="--")
    ax.text(9.3, 8.38, "est. coasting → valid again: re-initialise $F$ from $\\hat\\theta + \\dot{\\hat\\theta}/K_{v,fb}$ (D27)",
            ha="center", fontsize=B, color="#444444")
    # legend
    lx, ly = 19.3, -0.6
    for i, (c, t) in enumerate([(C_EST, "estimation and prediction"), (C_REF, "reference shaping"),
                                (C_FB, "feedback controller"), (C_FIRE, "firing"), (C_PL, "physical system")]):
        ax.add_patch(Rectangle((lx, ly + i * 0.42), 0.35, 0.28, fc=c, ec=EDGE, lw=0.6))
        ax.text(lx + 0.5, ly + i * 0.42 + 0.14, t, va="center", fontsize=B - 0.3)
    ax.text(22.9, -0.6, "Overall reference → turret:\n$F(z)\\,T(z) \\approx 1/(1 + t_p s)$, so the\n"
            "prediction lead and the lag cancel\nand the turret points at $\\hat\\theta(t)$.",
            fontsize=B - 0.3, va="bottom", color="#333333")
    save(fig, "block_diagram")


# =====================================================================
# 2. Engineering schematic of the elevation drive and geometry
# =====================================================================
def schematic():
    fig, ax = plt.subplots(figsize=(9.6, 4.6))
    ax.set_xlim(0, 19.2); ax.set_ylim(0, 9.2); ax.axis("off"); ax.set_aspect("equal")

    # ---- armature circuit ----
    cx, cy = 0.6, 2.2
    line(ax, [(cx, cy), (cx, cy + 3.2), (cx + 4.0, cy + 3.2)])
    line(ax, [(cx, cy), (cx + 4.0, cy)])
    ax.add_patch(Circle((cx, cy + 1.6), 0.45, fc="w", ec=EDGE, lw=1.0, zorder=2))
    ax.text(cx, cy + 1.75, "+", ha="center", va="center", fontsize=9, zorder=3)
    ax.text(cx, cy + 1.42, "−", ha="center", va="center", fontsize=9, zorder=3)
    ax.text(cx - 0.55, cy + 1.6, "$V_m$\n(±600 V)", ha="right", va="center", fontsize=8)
    # resistor zig-zag
    xr = np.linspace(cx + 0.8, cx + 1.9, 9); yr = cy + 3.2 + 0.18 * np.array([0, 1, -1, 1, -1, 1, -1, 1, 0])
    ax.plot(xr, yr, color=EDGE, lw=1.0, zorder=3)
    ax.add_patch(Rectangle((cx + 0.8, cy + 3.0), 1.1, 0.4, fc="w", ec="w", zorder=2))
    ax.plot(xr, yr, color=EDGE, lw=1.0, zorder=3)
    ax.text(cx + 1.35, cy + 3.55, "$R_m$ = 1 Ω", ha="right", fontsize=8)
    # inductor coils
    for k in range(4):
        ax.add_patch(Arc((cx + 2.35 + 0.3 * k, cy + 3.2), 0.3, 0.45, theta1=0, theta2=180, lw=1.0, zorder=3))
    ax.add_patch(Rectangle((cx + 2.2, cy + 3.05), 1.2, 0.12, fc="w", ec="w", zorder=2))
    ax.text(cx + 2.35, cy + 3.55, "$L_m$ = 0.1 H", ha="left", fontsize=8)
    ax.annotate("", xy=(cx + 2.6, cy + 2.6), xytext=(cx + 1.8, cy + 2.6),
                arrowprops=dict(arrowstyle="-|>", lw=0.9, color=EDGE))
    ax.text(cx + 2.2, cy + 2.15, "$i$", ha="center", fontsize=9)
    # motor
    mx, my = cx + 4.0, cy + 1.6
    ax.add_patch(Circle((mx, my), 0.95, fc=C_PL, ec=EDGE, lw=1.2, zorder=2))
    ax.text(mx, my + 0.15, "M", ha="center", va="center", fontsize=12, fontweight="bold", zorder=3)
    ax.text(mx, my - 0.42, "$e = K_e\\omega_m$", ha="center", fontsize=7.2, zorder=3)
    line(ax, [(mx, cy + 3.2), (mx, my + 0.95)]); line(ax, [(mx, cy), (mx, my - 0.95)])
    ax.text(mx - 0.4, my - 1.85, "DC motor\n$K_t$ = 10 N m/A\n$K_e$ = 10 V s/rad\nrotor $J_m$ = 0.2 kg m$^2$",
            ha="center", va="top", fontsize=7.6)
    # shaft
    ax.add_patch(Rectangle((mx + 0.95, my - 0.12), 1.2, 0.24, fc="#cccccc", ec=EDGE, lw=0.8, zorder=1))
    ax.text(mx + 1.55, my + 0.3, "$\\omega_m = N\\dot\\theta$", ha="center", fontsize=7.6)
    # gearbox
    gx = mx + 2.15
    ax.add_patch(Circle((gx + 0.35, my), 0.35, fc="#dddddd", ec=EDGE, lw=1.0, zorder=2))
    ax.add_patch(Circle((gx + 0.35 + 1.25, my), 0.9, fc="#dddddd", ec=EDGE, lw=1.0, zorder=2))
    ax.text(gx + 1.7, my - 1.85, "gearbox, N = 100:1\n(ideal: rigid,\nlossless, no backlash)", ha="center", va="top", fontsize=7.6)
    # output shaft to trunnion
    tx, ty = 13.2, 2.4
    line(ax, [(gx + 2.5, my), (tx - 0.6, my), (tx - 0.6, ty), (tx - 0.25, ty)])
    ax.text((gx + 2.5 + tx - 0.6) / 2, my + 0.25, "torque $NK_ti$", ha="center", fontsize=7.6)

    # ---- turret on trunnion, elevation geometry ----
    ax.add_patch(Polygon([(tx - 1.0, ty - 1.2), (tx + 1.0, ty - 1.2), (tx + 0.45, ty - 0.1), (tx - 0.45, ty - 0.1)],
                         closed=True, fc=C_PL, ec=EDGE, lw=1.0, zorder=2))
    ax.add_patch(Circle((tx, ty), 0.3, fc="w", ec=EDGE, lw=1.2, zorder=4))
    th = np.radians(32)
    L = 3.3
    bw = 0.32
    d = np.array([np.cos(th), np.sin(th)]); n = np.array([-np.sin(th), np.cos(th)])
    p0 = np.array([tx, ty]) - 0.6 * d
    barrel = [p0 + bw / 2 * n, p0 + L * d + bw / 2 * n, p0 + L * d - bw / 2 * n, p0 - bw / 2 * n]
    ax.add_patch(Polygon(barrel, closed=True, fc="#c9d6e8", ec=EDGE, lw=1.0, zorder=3))
    ax.text(tx + 1.1, ty - 1.65, "laser turret + cradle\n$J_{turret}$ = 1200 kg m$^2$, $b$ = 1500 N m s/rad\n"
            "radar co-located at the trunnion", ha="center", va="top", fontsize=7.6)
    # horizontal reference and angle
    line(ax, [(tx, ty), (tx + 4.6, ty)], color="#777777", ls=":")
    ax.add_patch(Arc((tx, ty), 3.2, 3.2, theta1=0, theta2=32, lw=0.9, color=EDGE))
    ax.text(tx + 1.85, ty + 0.42, "$\\theta_{turret}$", fontsize=8.5)
    # stops
    for ang, lab in [(-10, "−10° stop"), (85, "+85° stop")]:
        a = np.radians(ang)
        pe = np.array([tx, ty]) + 2.1 * np.array([np.cos(a), np.sin(a)])
        line(ax, [(tx, ty), tuple(pe)], color="#b23b3b", ls="--")
        ax.text(pe[0] + (0.1 if ang < 0 else -0.1), pe[1] + (-0.3 if ang < 0 else 0.15), lab,
                fontsize=7.4, color="#b23b3b", ha="left" if ang < 0 else "right")
    # beam to target
    th_t = np.radians(38)
    tgt = np.array([tx, ty]) + 6.8 * np.array([np.cos(th_t), np.sin(th_t)])
    tgt = np.minimum(tgt, [18.6, 8.6])
    line(ax, [(tx, ty), tuple(tgt)], color="#2a6fb0")
    ax.add_patch(Polygon([tgt + [-0.35, -0.1], tgt + [0.35, -0.1], tgt + [0, 0.25]], closed=True,
                         fc="#2a6fb0", ec=EDGE, lw=0.8, zorder=4))
    ax.text(tgt[0] - 0.2, tgt[1] + 0.35, "V-44 target $(x, y)$", ha="right", fontsize=7.8)
    mid = (np.array([tx, ty]) + tgt) / 2
    ax.text(mid[0] - 0.55, mid[1] + 0.25, "range $r$", fontsize=7.6, color="#2a6fb0", rotation=38)
    ax.add_patch(Arc((tx, ty), 5.0, 5.0, theta1=0, theta2=38, lw=0.8, color="#2a6fb0", ls="--"))
    ax.text(tx + 2.65, ty + 1.2, "$\\theta$ (line of sight)", fontsize=7.6, color="#2a6fb0")
    # equations box
    ax.text(0.3, 8.9,
            "Electrical:  $L_m\\,di/dt + R_m i + K_e N\\dot\\theta = V_m$\n"
            "Mechanical (turret side):  $(J_{turret} + N^2 J_m)\\ddot\\theta + b\\dot\\theta = N K_t i$\n"
            "$J_{eq} = J_{turret} + N^2 J_m$ = 3200 kg m$^2$\n"
            "$G_p(s) = \\theta(s)/V_m(s) = 1000\\,/\\,[\\,s\\,(320 s^2 + 3350 s + 1.0015\\times10^6)\\,]$",
            va="top", fontsize=8.0, linespacing=1.7,
            bbox=dict(boxstyle="round,pad=0.35", fc="#fafafa", ec="#999999", lw=0.8))
    save(fig, "drive_schematic")


if __name__ == "__main__":
    block_diagram()
    schematic()
    print("written: block_diagram.(png|pdf|svg), drive_schematic.(png|pdf|svg)")
