import matplotlib.pyplot as plt
import numpy as np


# Current calibrated RPM anchors.
anchor_distance = np.array([30.0, 34.0, 120.0])
anchor_rpm = np.array([29.0, 31.5, 34.2])


def calibrated_rpm(distance):
    distance = np.asarray(distance)
    return np.where(
        distance <= 34.0,
        np.interp(distance, [30.0, 34.0], [29.0, 31.5]),
        np.interp(distance, [34.0, 120.0], [31.5, 34.2]),
    )


# Groups with a valid camera distance and a reported outcome.
# Directional misses are marked separately because they should not tune RPM.
groups = [
    # distance, rpm, scored, total, note, usable_for_speed
    (30.1, 29.0, 2, 2, "2/2", True),
    (32.0, 29.0, 3, 3, "3/3", True),
    (34.0, 31.0, 4, 4, "4/4", True),
    (29.0, 29.0, 2, 4, "2/4; 2 left", False),
    (32.4, 30.0, 1, 2, "1/2; angle off", False),
    (34.0, 30.0, 0, 2, "0/2 short", True),
    (33.2, 31.0, 3, 4, "3/4; 1 short", True),
    (33.2, 31.0, 4, 4, "4/4", True),
    (42.5, 31.0, 0, 2, "0/2 left", False),
    (30.3, 31.0, 1, 2, "1/2; 1 long", True),
    (35.3, 31.0, 0, 3, "0/3 short", True),
    (33.6, 31.0, 0, 2, "0/2 left", False),
    (33.7, 31.3, 2, 3, "2/3; 1 short", True),
    (33.7, 31.3, 4, 4, "4/4", True),
]

fig, (ax_curve, ax_rate) = plt.subplots(
    2, 1, figsize=(11, 9), gridspec_kw={"height_ratios": [2.0, 1.2]}
)

distance_line = np.linspace(20, 120, 500)
ax_curve.plot(
    distance_line,
    calibrated_rpm(distance_line),
    color="#c62828",
    linewidth=3,
    label="Current calibrated target RPM",
)
ax_curve.scatter(
    anchor_distance,
    anchor_rpm,
    marker="D",
    s=90,
    color="#8e0000",
    zorder=5,
    label="Calibration anchors",
)

for distance, rpm, scored, total, note, usable in groups:
    success = scored / total
    color = plt.cm.RdYlGn(success)
    marker = "o" if usable else "X"
    ax_curve.scatter(distance, rpm, s=85, marker=marker, color=color,
                     edgecolor="black", linewidth=0.7, zorder=4)
    ax_curve.annotate(note, (distance, rpm), xytext=(5, 7),
                      textcoords="offset points", fontsize=8)

ax_curve.set_title("Red TeleOp Shooter Calibration: Distance vs. Wheel RPM", fontsize=15)
ax_curve.set_xlabel("AprilTag range (inches)")
ax_curve.set_ylabel("Shooter wheel target (RPM)")
ax_curve.set_xlim(20, 125)
ax_curve.set_ylim(26.5, 35.5)
ax_curve.grid(True, alpha=0.25)
ax_curve.legend(loc="lower right")
ax_curve.text(
    0.01, 0.98,
    "Circle = usable for RPM tuning   X = directional/alignment miss\n"
    "Point color: red 0% scored → green 100% scored",
    transform=ax_curve.transAxes,
    va="top",
    fontsize=9,
    bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
)

usable_groups = [group for group in groups if group[5]]
labels = [f"{g[0]:.1f}in\n{g[1]:.0f}RPM" for g in usable_groups]
rates = [100.0 * g[2] / g[3] for g in usable_groups]
colors = [plt.cm.RdYlGn(rate / 100.0) for rate in rates]
bars = ax_rate.bar(np.arange(len(rates)), rates, color=colors,
                   edgecolor="black", linewidth=0.6)
ax_rate.set_xticks(np.arange(len(labels)), labels)
ax_rate.set_ylim(0, 110)
ax_rate.set_ylabel("Scored (%)")
ax_rate.set_title("Speed-relevant test groups")
ax_rate.grid(axis="y", alpha=0.25)
for bar, rate in zip(bars, rates):
    ax_rate.text(bar.get_x() + bar.get_width() / 2, rate + 2,
                 f"{rate:.0f}%", ha="center", va="bottom", fontsize=9)

fig.tight_layout()
output_path = "red_teleop_shooter_calibration.png"
fig.savefig(output_path, dpi=180, bbox_inches="tight")
print(output_path)
