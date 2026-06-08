from aim_fsm import *
import math

# Goal: midpoint of the two nearest BARRELS (any color, so orange counts).
# If fewer than 2 barrels are seen, fall back to a point 20 cm straight ahead.
GOAL_FORWARD_MM = 200    # fallback goal: 20 cm ahead
GOAL_LEFT_MM = 0
LEFT_SIGN = +1           # flip to -1 if it aims the mirror-opposite way


def pose_xy(pose):
    """Pull (x, y) out of a pose, trying a few common access styles."""
    if pose is None:
        return None
    for attr in ("position", "translation", "t"):
        sub = getattr(pose, attr, None)
        if sub is not None:
            if hasattr(sub, "x"):
                return sub.x, sub.y
            try:
                return sub[0], sub[1]
            except Exception:
                pass
    if hasattr(pose, "x") and hasattr(pose, "y"):
        return pose.x, pose.y
    try:
        return pose[0], pose[1]
    except Exception:
        return None


def ball_rel(ball):
    """Ball position relative to the robot in mm, as (forward, left)."""
    xy = pose_xy(getattr(ball, "pose", None))
    if xy is None:
        return None
    x, y = xy
    return x, y * LEFT_SIGN


def is_barrel(obj):
    return "Barrel" in type(obj).__name__       # matches OrangeBarrel / BlueBarrel


def find_goal_center():
    """Midpoint of the two NEAREST barrels, as (forward, left) mm. None if <2 seen.
    This is the goal-detection logic, same idea as the goal_detection module."""
    posts = []
    for obj in robot.world_map.objects.values():
        if is_barrel(obj):
            xy = pose_xy(getattr(obj, "pose", None))
            if xy is not None:
                x, y = xy
                d = getattr(obj, "sensor_distance", None)
                posts.append((d if d is not None else 9999, x, y * LEFT_SIGN))
    if len(posts) < 2:
        return None
    posts.sort(key=lambda p: p[0])
    _, x0, y0 = posts[0]
    _, x1, y1 = posts[1]
    return (x0 + x1) / 2.0, (y0 + y1) / 2.0


class FindBall(StateNode):
    """See a ball -> detect the goal (barrels) -> work out the turn that lines them up -> grab."""
    def start(self, event=None):
        super().start(event)
        ball = None
        for obj in robot.world_map.objects.values():
            if isinstance(obj, SportsBallObj) and getattr(obj, "is_visible", False):
                ball = obj
                break
        if ball is None:
            print("[FIND] no ball -> look around")
            self.post_failure()
            return
        self.parent.ball = ball

        rel = ball_rel(ball)
        if rel is None:
            print("[FIND] couldn't read ball pose -> fallback: kick straight")
            self.parent.aim_turn = 0
            self.post_data(ball)
            return

        bx, by = rel
        goal = find_goal_center()                    # <-- goal detection from the barrels
        if goal is None:
            gx, gy = GOAL_FORWARD_MM, GOAL_LEFT_MM
            print("[GOAL] no 2 barrels -> hardcoded 20cm-ahead goal")
        else:
            gx, gy = goal
            print(f"[GOAL] barrels -> center fwd={gx:.0f} left={gy:.0f}")

        dir_to_ball   = math.degrees(math.atan2(by, bx))
        dir_ball_goal = math.degrees(math.atan2(gy - by, gx - bx))
        aim = (dir_ball_goal - dir_to_ball + 180) % 360 - 180
        self.parent.aim_turn = aim

        print(f"[FIND] ball fwd={bx:.0f} left={by:.0f} | "
              f"to_ball={dir_to_ball:.0f} ball->goal={dir_ball_goal:.0f} | "
              f"turn_after_grab={aim:.0f} deg")
        self.post_data(ball)


class GetGrabDist(StateNode):
    """Drive the full distance INTO the ball so the front magnet grabs it."""
    def start(self, event=None):
        super().start(event)
        d = getattr(self.parent.ball, "sensor_distance", None)
        dist = (d + 20) if d else 100
        print(f"[GRAB] driving {dist:.0f} mm into ball")
        self.post_data(max(dist, 20))


class AimAtGoal(StateNode):
    """Holding the ball now: turn to face the goal. The kick then releases it there."""
    def start(self, event=None):
        super().start(event)
        turn = getattr(self.parent, "aim_turn", 0)
        print(f"[AIM] turning {turn:.0f} deg toward goal, then kicking")
        self.post_data(turn)


class kick(StateMachineProgram):
    def __init__(self):
        super().__init__(launch_cam_viewer=True, launch_worldmap_viewer=True)
        self.ball = None
        self.aim_turn = 0

    def setup(self):
        #         Glow(vex.LightType.ALL_LEDS, 0, 0, 255) =C=> find
        #         find: FindBall()
        #         find =D=> charge          # ball seen, then go score it
        #         find =F=> look_around     # no ball, then scan
        #         look_around: Turn(40)
        #         look_around =C=> find
        #         charge: TurnToward()
        #         charge =C=> grab
        #         grab: GetGrabDist()       # drive into ball, then magnet grabs it
        #         grab =D=> Forward() =C=> aim
        #         aim: AimAtGoal()
        #         aim =D=> Turn() =C=> kicker
        #         kicker: Kick()
        #         kicker =C=> find
        
        # Code generated by genfsm on Mon Jun  8 11:17:13 2026:
        
        glow1 = Glow(vex.LightType.ALL_LEDS, 0, 0, 255) .set_name("glow1") .set_parent(self)
        find = FindBall() .set_name("find") .set_parent(self)
        look_around = Turn(40) .set_name("look_around") .set_parent(self)
        charge = TurnToward() .set_name("charge") .set_parent(self)
        grab = GetGrabDist() .set_name("grab") .set_parent(self)
        forward1 = Forward() .set_name("forward1") .set_parent(self)
        aim = AimAtGoal() .set_name("aim") .set_parent(self)
        turn1 = Turn() .set_name("turn1") .set_parent(self)
        kicker = Kick() .set_name("kicker") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(glow1) .add_destinations(find)
        
        datatrans1 = DataTrans() .set_name("datatrans1")
        datatrans1 .add_sources(find) .add_destinations(charge)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(find) .add_destinations(look_around)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(look_around) .add_destinations(find)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(charge) .add_destinations(grab)
        
        datatrans2 = DataTrans() .set_name("datatrans2")
        datatrans2 .add_sources(grab) .add_destinations(forward1)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(forward1) .add_destinations(aim)
        
        datatrans3 = DataTrans() .set_name("datatrans3")
        datatrans3 .add_sources(aim) .add_destinations(turn1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(turn1) .add_destinations(kicker)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(kicker) .add_destinations(find)
        
        return self
