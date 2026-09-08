"""
tkinter renderer and real-time loop.

tkinter rather than pygame purely because it ships with Python and this
machine could not reach PyPI. It is entirely adequate: the scene is a few
dozen canvas items, and Windows tkinter reports held keys cleanly (KeyPress
repeats while down, KeyRelease fires only on actual release), which is the
one thing that would have made it unusable.

The renderer draws GROUND TRUTH solid and the AI's BELIEF as ghosts. Being
able to see those two diverge -- the ghost lagging behind the real ball,
snapping when a detection lands, drifting during an occlusion -- is the
single most useful debugging view in the whole project, and it is why the
overlays are here from the start rather than bolted on later.
"""

from __future__ import annotations

import math
import time
import tkinter as tk

import config
from sim.geometry import to_world, rect_corners
from game.match import Phase

# Palette
C_BG = "#12161c"
C_PITCH = "#1b4d2e"
C_LINE = "#cfe8d8"
C_BALL = "#ff8c1a"
C_AI = "#4da3ff"
C_HUMAN = "#ff5566"
C_GHOST_AI = "#2a5f94"
C_TEXT = "#e8eef5"
C_DIM = "#8fa3b8"
C_WARN = "#ffcc44"


class Renderer:
    def __init__(self, match, human, ai_agent=None, title="Robot Football"):
        self.match = match
        self.human = human
        self.ai = ai_agent

        self.scale = config.RENDER_SCALE_PX_PER_M
        margin = config.RENDER_MARGIN_PX
        pitch_w = config.ARENA_LENGTH_M * self.scale
        pitch_h = config.ARENA_WIDTH_M * self.scale
        goal_d = config.GOAL_DEPTH_M * self.scale

        self.width = int(pitch_w + 2 * goal_d + 2 * margin)
        self.height = int(pitch_h + 2 * margin) + 96
        self.cx = self.width / 2.0
        self.cy = margin + pitch_h / 2.0

        self.root = tk.Tk()
        self.root.title(title)
        self.root.configure(bg=C_BG)
        self.root.resizable(False, False)

        self.canvas = tk.Canvas(self.root, width=self.width,
                                height=self.height, bg=C_BG,
                                highlightthickness=0)
        self.canvas.pack()

        self.root.bind("<KeyPress>", self._on_key_down)
        self.root.bind("<KeyRelease>", self._on_key_up)
        self.root.bind("<FocusOut>", lambda e: self.human.clear())
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

        self.paused = False
        self._overlay_error = None
        self.running = True
        self._last_wall = None
        self._accum = 0.0
        self.frame_times: list[float] = []

    # -- input -------------------------------------------------------------

    def _on_key_down(self, event) -> None:
        k = (event.keysym or "").lower()
        if k == "escape":
            self._quit()
            return
        if k == "space":
            self.paused = not self.paused
            if self.paused:
                self.human.clear()
            return
        if k == "r":
            self.match.world.kickoff()
            return
        # Overlay toggles
        if k == "f1":
            config.SHOW_AI_BELIEF = not config.SHOW_AI_BELIEF
        elif k == "f2":
            config.SHOW_PREDICTION = not config.SHOW_PREDICTION
        elif k == "f3":
            config.SHOW_REACHABILITY = not config.SHOW_REACHABILITY
        elif k == "f4":
            config.SHOW_MPC_ROLLOUTS = not config.SHOW_MPC_ROLLOUTS
        elif k == "f5":
            config.SHOW_ESTIMATOR_HUD = not config.SHOW_ESTIMATOR_HUD
        else:
            self.human.key_down(k)

    def _on_key_up(self, event) -> None:
        self.human.key_up((event.keysym or "").lower())

    def _quit(self) -> None:
        self.running = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    # -- transforms --------------------------------------------------------

    def sx(self, x: float) -> float:
        return self.cx + x * self.scale

    def sy(self, y: float) -> float:
        return self.cy - y * self.scale       # world +y is up, screen +y down

    def pt(self, p) -> tuple[float, float]:
        return (self.sx(p[0]), self.sy(p[1]))

    # -- drawing -----------------------------------------------------------

    def _draw_pitch(self) -> None:
        c = self.canvas
        hx, hy = config.HALF_LENGTH_M, config.HALF_WIDTH_M
        gd, gh = config.GOAL_DEPTH_M, config.HALF_GOAL_M

        c.create_rectangle(self.sx(-hx), self.sy(hy), self.sx(hx), self.sy(-hy),
                           fill=C_PITCH, outline=C_LINE, width=2)
        c.create_line(self.sx(0), self.sy(hy), self.sx(0), self.sy(-hy),
                      fill=C_LINE, width=1)
        r = 0.25 * self.scale
        c.create_oval(self.sx(0) - r, self.sy(0) - r,
                      self.sx(0) + r, self.sy(0) + r, outline=C_LINE, width=1)

        # Goal recesses, drawn behind each goal line.
        for sgn, col in ((+1, C_AI), (-1, C_HUMAN)):
            x0 = sgn * hx
            x1 = sgn * (hx + gd)
            c.create_rectangle(self.sx(min(x0, x1)), self.sy(gh),
                               self.sx(max(x0, x1)), self.sy(-gh),
                               fill="#0d1117", outline=col, width=3)
            # Goal mouth highlight
            c.create_line(self.sx(x0), self.sy(gh), self.sx(x0), self.sy(-gh),
                          fill=col, width=4)

    def _draw_robot(self, robot, colour: str, ghost: bool = False,
                    pose=None) -> None:
        c = self.canvas
        pos = pose[0] if pose else robot.pos
        th = pose[1] if pose else robot.theta

        corners = rect_corners(pos, th, robot.half_len, robot.half_wid)
        flat = []
        for p in corners:
            flat.extend(self.pt(p))
        if ghost:
            c.create_polygon(*flat, outline=colour, fill="", width=1,
                             dash=(3, 3))
        else:
            c.create_polygon(*flat, outline=colour, fill="#232b36", width=2)

        # Horns
        y_off = config.HORN_GAP_M / 2.0 + config.HORN_WIDTH_M / 2.0
        x0 = robot.half_len
        x1 = robot.half_len + config.HORN_LENGTH_M
        for sy_ in (+1.0, -1.0):
            a = to_world((x0, sy_ * y_off), pos, th)
            b = to_world((x1, sy_ * y_off), pos, th)
            w = max(2, int(config.HORN_WIDTH_M * self.scale))
            c.create_line(*self.pt(a), *self.pt(b), fill=colour,
                          width=w, capstyle=tk.ROUND,
                          dash=(3, 3) if ghost else None)

        if ghost:
            return

        # Vision markers: the large team circle and the offset heading dot.
        # These are what the overhead camera actually sees -- everything the
        # AI knows about pose comes from finding these two blobs.
        rc = config.MARKER_CENTRE_DIA_M / 2.0 * self.scale
        p = self.pt(pos)
        c.create_oval(p[0] - rc, p[1] - rc, p[0] + rc, p[1] + rc,
                      fill=colour, outline="")
        dot = to_world((config.MARKER_DOT_SEPARATION_M, 0.0), pos, th)
        rd = config.MARKER_DOT_DIA_M / 2.0 * self.scale
        d = self.pt(dot)
        c.create_oval(d[0] - rd, d[1] - rd, d[0] + rd, d[1] + rd,
                      fill="#f2f6fb", outline="")

    def _draw_ball(self, pos, ghost: bool = False) -> None:
        r = config.BALL_RADIUS_M * self.scale
        p = self.pt(pos)
        if ghost:
            self.canvas.create_oval(p[0] - r, p[1] - r, p[0] + r, p[1] + r,
                                    outline=C_BALL, dash=(2, 2), width=1)
        else:
            self.canvas.create_oval(p[0] - r, p[1] - r, p[0] + r, p[1] + r,
                                    fill=C_BALL, outline="#ffd9a0")

    def _draw_overlays(self) -> None:
        """AI belief, prediction, and planner internals."""
        if self.ai is None:
            return
        dbg = getattr(self.ai, "debug", None)
        if not dbg:
            return

        if config.SHOW_REACHABILITY and dbg.get("reach_rects"):
            # Merged runs, not one item per grid cell -- see
            # ReachableSet.rects(). This is what keeps the overlay from
            # costing more than the entire rest of the frame.
            for (x0, y0, x1, y1) in dbg["reach_rects"]:
                a = self.pt((x0, y0))
                b = self.pt((x1, y1))
                self.canvas.create_rectangle(a[0], a[1], b[0], b[1],
                                             fill="#5a2a2a", outline="")

        if config.SHOW_MPC_ROLLOUTS and dbg.get("rollouts"):
            best = dbg.get("best_index", -1)
            paths = dbg["rollouts"]
            for i, path in enumerate(paths):
                if i == best:
                    continue
                # Every other point is plenty for a 60-step path, and halves
                # the line segments tk has to rasterise.
                #
                # Check the length AFTER downsampling, not before: a rollout
                # that terminated early (a goal, one or two steps in) has two
                # points, and path[::2] leaves it with one. create_line then
                # gets a single coordinate pair and raises TclError, which
                # takes down the whole render callback.
                pts = path[::2]
                if len(pts) < 2:
                    pts = path
                if len(pts) < 2:
                    continue
                flat = []
                for p in pts:
                    flat.extend(self.pt(p))
                self.canvas.create_line(*flat, fill="#3d4c5e", width=1)
            # Draw the chosen one last so it sits on top.
            if 0 <= best < len(paths) and len(paths[best]) >= 2:
                flat = []
                for p in paths[best]:
                    flat.extend(self.pt(p))
                self.canvas.create_line(*flat, fill="#ffe08a", width=2)

        if config.SHOW_AI_BELIEF:
            if dbg.get("ball_belief"):
                self._draw_ball(dbg["ball_belief"], ghost=True)
            for idx, key in ((0, "self_belief"), (1, "opp_belief")):
                if dbg.get(key):
                    self._draw_robot(self.match.world.robots[idx],
                                     C_GHOST_AI if idx == 0 else "#8a3a44",
                                     ghost=True, pose=dbg[key])

        if config.SHOW_PREDICTION and dbg.get("ball_predicted"):
            p = self.pt(dbg["ball_predicted"])
            r = config.BALL_RADIUS_M * self.scale * 0.7
            self.canvas.create_oval(p[0] - r, p[1] - r, p[0] + r, p[1] + r,
                                    outline="#ffee88", width=2)
        if config.SHOW_PREDICTION and dbg.get("aim_point"):
            a = self.pt(dbg["aim_point"])
            self.canvas.create_line(a[0] - 7, a[1] - 7, a[0] + 7, a[1] + 7,
                                    fill="#ffee88", width=2)
            self.canvas.create_line(a[0] - 7, a[1] + 7, a[0] + 7, a[1] - 7,
                                    fill="#ffee88", width=2)

    def _draw_hud(self) -> None:
        c = self.canvas
        m = self.match
        y0 = self.height - 88

        a, b = m.score
        c.create_text(self.cx, y0, text=f"{a}   -   {b}",
                      fill=C_TEXT, font=("Consolas", 26, "bold"))
        c.create_text(self.cx - 130, y0, text="AI", fill=C_AI,
                      font=("Consolas", 14, "bold"))
        c.create_text(self.cx + 130, y0, text="YOU", fill=C_HUMAN,
                      font=("Consolas", 14, "bold"))

        mins, secs = divmod(max(0.0, m.clock), 60)
        c.create_text(self.cx, y0 + 26, text=f"{int(mins)}:{secs:04.1f}",
                      fill=C_DIM, font=("Consolas", 12))

        left = []
        if config.SHOW_ESTIMATOR_HUD and self.ai is not None:
            dbg = getattr(self.ai, "debug", {}) or {}
            err = dbg.get("rms_error")
            if err:
                left.append(f"belief err  ball {err.get('ball', 0)*1000:5.1f}mm"
                            f"  self {err.get('self', 0)*1000:5.1f}mm"
                            f"  opp {err.get('opp', 0)*1000:5.1f}mm")
            if dbg.get("latency_est") is not None:
                left.append(f"latency est {dbg['latency_est']*1000:5.1f} ms"
                            f"  (true {config.total_loop_latency_s()*1000:.1f})")
            if dbg.get("state"):
                left.append(f"state       {dbg['state']}")

        fps = 0.0
        if len(self.frame_times) > 4:
            span = self.frame_times[-1] - self.frame_times[0]
            if span > 0:
                fps = (len(self.frame_times) - 1) / span
        if self._overlay_error:
            left.append(f"overlay error: {self._overlay_error[:60]}")
        left.append(f"{fps:4.0f} fps   "
                    f"space=pause  r=kickoff  F1-F5=overlays  esc=quit")

        for i, line in enumerate(left):
            c.create_text(14, y0 - 10 + i * 15, text=line, anchor="w",
                          fill=C_DIM, font=("Consolas", 9))

        if self.paused:
            c.create_text(self.cx, self.cy, text="PAUSED",
                          fill=C_WARN, font=("Consolas", 30, "bold"))
        if m.phase is Phase.GOAL_PAUSE:
            who = "AI SCORES" if m.world.last_goal_by == 0 else "YOU SCORE"
            c.create_text(self.cx, self.cy, text=who, fill=C_WARN,
                          font=("Consolas", 30, "bold"))
        if m.phase is Phase.FINISHED:
            c.create_text(self.cx, self.cy, text="FULL TIME", fill=C_WARN,
                          font=("Consolas", 30, "bold"))

    def draw(self) -> None:
        self.canvas.delete("all")
        self._draw_pitch()
        # Overlays are diagnostics. A fault in one must never take down the
        # render callback and with it the whole match -- which is exactly
        # what happened when a degenerate rollout path reached create_line.
        try:
            self._draw_overlays()
        except Exception as exc:                      # noqa: BLE001
            self._overlay_error = repr(exc)
        w = self.match.world
        self._draw_robot(w.robots[0], C_AI)
        self._draw_robot(w.robots[1], C_HUMAN)
        self._draw_ball(w.ball.pos)
        self._draw_hud()

    # -- loop --------------------------------------------------------------

    def _tick(self) -> None:
        if not self.running:
            return

        now = time.perf_counter()
        if self._last_wall is None:
            self._last_wall = now
        elapsed = now - self._last_wall
        self._last_wall = now

        self.frame_times.append(now)
        if len(self.frame_times) > 30:
            self.frame_times.pop(0)

        if not self.paused:
            # Fixed-step integration with an accumulator, so physics stays
            # deterministic regardless of how the display is behaving.
            self._accum += min(elapsed, 0.1)   # cap: never spiral after a stall
            dt = config.PHYSICS_DT
            steps = 0
            max_steps = int(config.PHYSICS_HZ / config.RENDER_HZ * 3)
            while self._accum >= dt and steps < max_steps:
                self.match.step(dt)
                self._accum -= dt
                steps += 1

        self.draw()
        self.root.after(max(1, int(1000 / config.RENDER_HZ)), self._tick)

    def run(self) -> None:
        self.root.after(10, self._tick)
        self.root.focus_force()
        self.root.mainloop()
