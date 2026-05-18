from .common import G, WalkerParams, clamp


class ZMPPreviewPlanner:
    def __init__(self, params: WalkerParams):
        self.params = params
        self.com_x = 0.0
        self.com_y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.initialized = False

    def reset(self, params: WalkerParams, x: float = 0.0, y: float = 0.0) -> None:
        self.params = params
        self.com_x = x
        self.com_y = y
        self.vx = 0.0
        self.vy = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.initialized = True

    def update(self, dt: float, zmp_now: tuple, zmp_preview: tuple) -> tuple:
        if not self.initialized:
            self.reset(self.params, zmp_now[0], zmp_now[1])

        p = self.params
        horizon = max(0.2, p.zmp_preview_time)
        target_x = 0.72 * zmp_now[0] + 0.28 * zmp_preview[0]
        target_y = 0.78 * zmp_now[1] + 0.22 * zmp_preview[1]

        omega2 = G / max(0.25, p.pelvis_height)
        ax_cmd = p.zmp_kp * omega2 * (target_x - self.com_x) - p.zmp_kd * self.vx / horizon
        ay_cmd = p.zmp_kp * omega2 * (target_y - self.com_y) - p.zmp_kd * self.vy / horizon
        ax_cmd = clamp(ax_cmd, -p.max_com_accel, p.max_com_accel)
        ay_cmd = clamp(ay_cmd, -p.max_com_accel, p.max_com_accel)

        self.vx = clamp(self.vx + ax_cmd * dt, -p.max_com_speed, p.max_com_speed)
        self.vy = clamp(self.vy + ay_cmd * dt, -p.max_com_speed, p.max_com_speed)
        self.com_x += self.vx * dt
        self.com_y += self.vy * dt
        self.ax = ax_cmd
        self.ay = ay_cmd
        return self.com_x, self.com_y, self.vx, self.vy, self.ax, self.ay

