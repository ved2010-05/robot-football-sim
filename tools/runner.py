"""A 'straight runner': drives forward, only small steady turns. The exploit."""
import math, config
from sim.geometry import wrap_angle, clamp
from sim.kinematics import (max_forward_speed, max_yaw_rate,
                            twist_to_wheels_clamped)

class StraightRunner:
    """Aims at the ball, but only ever corrects by <=TURN_DEG per decision,
    and never stops driving forward. Deliberately simple and committed."""
    def __init__(self, world, index, turn_deg=10.0, decide_hz=5.0):
        self.world=world; self.index=index
        self.turn=math.radians(turn_deg); self.period=1.0/decide_hz
        self._t=0.0; self._twist=(0.0,0.0)
    def update(self, dt):
        self._t+=dt
        if self._t < self.period: return self._twist
        self._t=0.0
        me=self.world.robots[self.index]; ball=self.world.ball
        goal=self.world.goal_centre(self.index)
        bx,by=ball.pos
        dx,dy=goal[0]-bx, goal[1]-by
        n=max(math.hypot(dx,dy),1e-6); ux,uy=dx/n,dy/n
        stand=config.ROBOT_LENGTH_M/2+config.HORN_LENGTH_M*0.5+ball.radius
        tgt=(bx-ux*stand, by-uy*stand)
        err=wrap_angle(math.atan2(tgt[1]-me.pos[1], tgt[0]-me.pos[0]) - me.theta)
        # only a small, steady correction -- this is the whole trick
        w=clamp(err, -self.turn, self.turn)/max(self.period,1e-6)
        w=clamp(w, -max_yaw_rate(), max_yaw_rate())
        self._twist=(max_forward_speed()*0.85, w)
        return self._twist
    def wheel_command(self): return twist_to_wheels_clamped(*self._twist)
    def intent(self):
        return self._twist if config.USE_INTENT_CHANNEL else None
    def clear(self): self._twist=(0.0,0.0)
    def key_down(self,k): pass
    def key_up(self,k): pass
