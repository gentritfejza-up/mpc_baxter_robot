#!/usr/bin/env python3
"""Render docs/dataflow.gif: the control data flow of the Baxter MPC stack.

Boxes are the running nodes; animated dots travel along the arrows at a speed
proportional to each link's update rate. Run from the repository root:

    python3 docs/make_dataflow_gif.py

Only Pillow is required.
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT = 1100, 620
FRAMES, FRAME_MS = 60, 70
BACKGROUND = (250, 250, 252)
INK = (40, 44, 52)
MUTED = (120, 126, 136)
COLOURS = {
    "reference": (66, 133, 244),
    "mpc": (52, 168, 83),
    "torque": (251, 140, 0),
    "robot": (156, 39, 176),
    "state": (96, 125, 139),
    "guard": (229, 57, 53),
}

BOXES = {
    "reference": ((20, 240), (230, 380), "Reference source", "pick-and-place demo or\nrqt_reconfigure: joint set\npoint q_ref (<= 10 Hz)"),
    "mpc": ((320, 200), (590, 420), "MPC node (50 Hz)", "OSQP QP, 25-step / 1.5 s horizon\ncollision guard: floor, body, other arm\nacceleration + base state (q0, v0)"),
    "torque": ((690, 240), (880, 380), "Torque loop (800 Hz)", "KDL or Lagrange-Euler\ntau = M qdd + C + g\nidle hold when idle"),
    "robot": ((970, 240), (1080, 380), "Baxter", "Gazebo\nor real robot"),
}

ARROWS = [
    # (from, to, label, colour, rate_hz, path points)
    ("reference", "mpc", "q_ref", COLOURS["reference"], 10, [(230, 310), (320, 310)]),
    ("mpc", "torque", "acceleration", COLOURS["mpc"], 50, [(590, 310), (690, 310)]),
    ("torque", "robot", "torque", COLOURS["torque"], 800, [(880, 310), (970, 310)]),
    ("robot", "state", "", COLOURS["state"], 1000,
     [(1025, 380), (1025, 480), (455, 480), (455, 420)]),
    ("robot", "torque_state", "", COLOURS["state"], 1000, [(785, 480), (785, 380)]),
    ("other", "guard", "", COLOURS["guard"], 1000, [(455, 120), (455, 200)]),
]


def font(size, bold=False):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


TITLE_FONT, BOX_FONT, SMALL_FONT = font(22, True), font(15, True), font(12)


def draw_box(draw, key, highlight):
    (x0, y0), (x1, y1), title, body = BOXES[key]
    colour = COLOURS[key]
    fill = tuple(int(c * 0.12 + 255 * 0.88) for c in colour)
    draw.rounded_rectangle((x0, y0, x1, y1), radius=14, fill=fill,
                           outline=colour, width=4 if highlight else 2)
    draw.text((x0 + 12, y0 + 10), title, fill=INK, font=BOX_FONT)
    draw.text((x0 + 12, y0 + 38), body, fill=MUTED, font=SMALL_FONT, spacing=4)


def polyline_length(points):
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def point_at(points, fraction):
    target = fraction * polyline_length(points)
    for i in range(len(points) - 1):
        segment = math.dist(points[i], points[i + 1])
        if target <= segment:
            ratio = target / segment if segment else 0.0
            return (points[i][0] + (points[i + 1][0] - points[i][0]) * ratio,
                    points[i][1] + (points[i + 1][1] - points[i][1]) * ratio)
        target -= segment
    return points[-1]


def draw_arrow(draw, points, colour, label):
    draw.line(points, fill=colour, width=3, joint="curve")
    (x0, y0), (x1, y1) = points[-2], points[-1]
    angle = math.atan2(y1 - y0, x1 - x0)
    head = [(x1, y1),
            (x1 - 12 * math.cos(angle - 0.45), y1 - 12 * math.sin(angle - 0.45)),
            (x1 - 12 * math.cos(angle + 0.45), y1 - 12 * math.sin(angle + 0.45))]
    draw.polygon(head, fill=colour)
    if label:
        mid = point_at(points, 0.5)
        width = draw.textlength(label, font=SMALL_FONT)
        box = (mid[0] - width / 2 - 4, mid[1] - 30, mid[0] + width / 2 + 4, mid[1] - 14)
        draw.rounded_rectangle(box, radius=4, fill=BACKGROUND)
        draw.text((mid[0], mid[1] - 22), label, fill=colour, font=SMALL_FONT, anchor="mm")


def render_frame(index):
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.text((WIDTH // 2, 36), "Baxter MPC stack: control data flow", fill=INK, font=TITLE_FONT, anchor="mm")
    draw.text((WIDTH // 2, 62), "dots move at a speed proportional to each link's update rate", fill=MUTED,
              font=SMALL_FONT, anchor="mm")
    phase = index / FRAMES
    draw.text((455, 104), "other arm: joint states (1 kHz)", fill=COLOURS["guard"], font=SMALL_FONT, anchor="mm")
    draw.rounded_rectangle((640, 440, 1080, 520), radius=12, fill=(240, 243, 246), outline=COLOURS["state"], width=2)
    draw.text((860, 462), "state feedback", fill=COLOURS["state"], font=BOX_FONT, anchor="mm")
    draw.text((860, 500), "/robot/joint_states (q, qd) at 1 kHz -> MPC and torque loop",
              fill=COLOURS["state"], font=SMALL_FONT, anchor="mm")
    for key in BOXES:
        active = {"reference": phase < 0.25, "mpc": 0.2 <= phase < 0.55, "torque": 0.5 <= phase < 0.8,
                  "robot": phase >= 0.75}[key]
        draw_box(draw, key, active)
    for _, _, label, colour, rate, points in ARROWS:
        draw_arrow(draw, points, colour, label)
        dots = max(1, min(6, int(math.log10(rate) * 2)))
        for dot in range(dots):
            fraction = (phase * (0.5 + math.log10(rate) / 2) + dot / dots) % 1.0
            x, y = point_at(points, fraction)
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=colour)
    guard_phase = (phase * 2) % 1.0
    radius = 10 + 6 * math.sin(guard_phase * 2 * math.pi)
    draw.ellipse((345 - radius, 395 - radius, 345 + radius, 395 + radius), outline=COLOURS["guard"], width=2)
    draw.text((362, 395), "guard checks the plan every cycle", fill=COLOURS["guard"], font=SMALL_FONT, anchor="lm")
    draw.text((WIDTH // 2, HEIGHT - 24),
              "Gentrit Fejza, Master's thesis, University of Prishtina (Electronics, Automation and Robotics), 2026",
              fill=MUTED, font=SMALL_FONT, anchor="mm")
    return image


def main():
    frames = [render_frame(i) for i in range(FRAMES)]
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataflow.gif")
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=FRAME_MS, loop=0, optimize=True)
    print("wrote", output, "(%d frames)" % len(frames))


if __name__ == "__main__":
    main()
