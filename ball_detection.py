from aim_fsm import *
import math
import time
from goal_detection import pose_xy

LEFT_SIGN = +1
SCAN_TURN_DEG = 30
SCAN_WAIT_SEC = 0.45
BALL_CONTROL_MM = 160
AIM_LIMIT_DEG = 70


def wrap_deg(a):
    while a <= -180:
        a += 360
    while a > 180:
        a -= 360
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def xy_from_pose(pose):
    try:
        xy = pose_xy(pose)
        if xy and xy[0] is not None:
            return float(xy[0]), float(xy[1])
    except:
        pass
    try:
        return float(pose.x), float(pose.y)
    except:
        pass
    return None


def ball_rel(ball):
    xy = xy_from_pose(getattr(ball, 'pose', None))
    if xy:
        return xy[0], xy[1] * LEFT_SIGN
    d = getattr(ball, 'sensor_distance', None)
    if d is not None:
        return float(d), 0.0
    return None


def is_ball(obj):
    SportsBall = globals().get('SportsBallObj', None)
    if SportsBall is not None and isinstance(obj, SportsBall):
        return True
    text = type(obj).__name__.lower()
    return 'sportsball' in text or 'sports_ball' in text or 'soccer' in text or 'ball' in text


def best_visible_ball(wm):
    choices = []
    for obj in wm.objects.values():
        if not getattr(obj, 'is_visible', False) or not is_ball(obj):
            continue
        rel = ball_rel(obj)
        if rel is None:
            d = getattr(obj, 'sensor_distance', None)
            d = float(d) if d is not None else 9999.0
            choices.append((d, obj, None))
        else:
            fwd, left = rel
            choices.append((math.sqrt(fwd * fwd + left * left), obj, rel))
    choices.sort(key=lambda x: x[0])
    return choices[0] if choices else None


def aim_from_ball_to_goal(parent, rel):
    if rel is None:
        return 0.0
    goal = getattr(parent.field_memory, 'goal_center', None)
    if goal is None:
        return 0.0
    bx, by = rel
    gx, gy = goal
    gy = gy * LEFT_SIGN
    dir_to_ball = math.degrees(math.atan2(by, bx))
    dir_ball_goal = math.degrees(math.atan2(gy - by, gx - bx))
    return clamp(wrap_deg(dir_ball_goal - dir_to_ball), -AIM_LIMIT_DEG, AIM_LIMIT_DEG)

class ball_detection(StateNode):

    class Check(StateNode):
        def start(self, event=None):
            super().start(event)
            
            self.robot.world_map.update()
            time.sleep(SCAN_WAIT_SEC)
            detector = self.parent
            result = best_visible_ball(self.robot.world_map)

            if result is None:
                last_d = getattr(detector.parent, 'last_ball_dist', None)
                last_seen = getattr(detector.parent, 'last_ball_seen', 0)
                if last_d is not None and last_d <= BALL_CONTROL_MM and time.time() - last_seen < 3.0:
                    print('[BALL] lost close ball; treating it as controlled')
                    detector.parent.ball_control_guess = True
                    detector.post_success()
                    return
                print('[BALL] no ball on this check')
                self.post_failure()
                return

            dist, ball, rel = result
            detector.parent.ball = ball
            detector.parent.ball_control_guess = dist <= BALL_CONTROL_MM
            detector.parent.last_ball_seen = time.time()
            detector.parent.last_ball_dist = dist
            detector.parent.aim_turn = aim_from_ball_to_goal(detector.parent, rel)

            if rel is not None:
                fwd, left = rel

                if abs(left) < 40:   # 40 mm = wide enough to avoid oscillation
                    print("[BALL] centered enough, going to grab")
                    detector.post_completion()
                    return

                if abs(detector.parent.aim_turn) < 12:  # 12 degrees deadband
                    print("[BALL] aim small enough, going to grab")
                    detector.post_success()
                    return

            if rel is None:
                print('[BALL] visible, no pose; using straight aim')
            else:
                print('[BALL] visible fwd=%.0f left=%.0f dist=%.0f aim=%.0f' %
                      (rel[0], rel[1], dist, detector.parent.aim_turn))

            detector.post_data(ball)

    def setup(self):
        #         c0: self.Check()
        #         c0 =F=> Turn(SCAN_TURN_DEG) =C=> c1
        # 
        #         c1: self.Check()
        #         c1 =F=> Turn(SCAN_TURN_DEG) =C=> c2
        # 
        #         c2: self.Check()
        #         c2 =F=> Turn(SCAN_TURN_DEG) =C=> c3
        # 
        #         c3: self.Check()
        #         c3 =F=> Turn(SCAN_TURN_DEG) =C=> c4
        # 
        #         c4: self.Check()
        #         c4 =F=> Turn(SCAN_TURN_DEG) =C=> c5
        # 
        #         c5: self.Check()
        #         c5 =F=> Turn(SCAN_TURN_DEG) =C=> c6
        # 
        #         c6: self.Check()
        #         c6 =F=> Turn(SCAN_TURN_DEG) =C=> c7
        # 
        #         c7: self.Check()
        #         c7 =F=> Turn(SCAN_TURN_DEG) =C=> c8
        # 
        #         c8: self.Check()
        #         c8 =F=> Turn(SCAN_TURN_DEG) =C=> c9
        # 
        #         c9: self.Check()
        #         c9 =F=> Turn(SCAN_TURN_DEG) =C=> c10
        # 
        #         c10: self.Check()
        #         c10 =F=> Turn(SCAN_TURN_DEG) =C=> c11
        # 
        #         c11: self.Check()
        #         c11 =F=> ParentFails()
        
        # Code generated by genfsm on Tue Jun  9 01:03:30 2026:
        
        c0 = self.Check() .set_name("c0") .set_parent(self)
        turn1 = Turn(SCAN_TURN_DEG) .set_name("turn1") .set_parent(self)
        c1 = self.Check() .set_name("c1") .set_parent(self)
        turn2 = Turn(SCAN_TURN_DEG) .set_name("turn2") .set_parent(self)
        c2 = self.Check() .set_name("c2") .set_parent(self)
        turn3 = Turn(SCAN_TURN_DEG) .set_name("turn3") .set_parent(self)
        c3 = self.Check() .set_name("c3") .set_parent(self)
        turn4 = Turn(SCAN_TURN_DEG) .set_name("turn4") .set_parent(self)
        c4 = self.Check() .set_name("c4") .set_parent(self)
        turn5 = Turn(SCAN_TURN_DEG) .set_name("turn5") .set_parent(self)
        c5 = self.Check() .set_name("c5") .set_parent(self)
        turn6 = Turn(SCAN_TURN_DEG) .set_name("turn6") .set_parent(self)
        c6 = self.Check() .set_name("c6") .set_parent(self)
        turn7 = Turn(SCAN_TURN_DEG) .set_name("turn7") .set_parent(self)
        c7 = self.Check() .set_name("c7") .set_parent(self)
        turn8 = Turn(SCAN_TURN_DEG) .set_name("turn8") .set_parent(self)
        c8 = self.Check() .set_name("c8") .set_parent(self)
        turn9 = Turn(SCAN_TURN_DEG) .set_name("turn9") .set_parent(self)
        c9 = self.Check() .set_name("c9") .set_parent(self)
        turn10 = Turn(SCAN_TURN_DEG) .set_name("turn10") .set_parent(self)
        c10 = self.Check() .set_name("c10") .set_parent(self)
        turn11 = Turn(SCAN_TURN_DEG) .set_name("turn11") .set_parent(self)
        c11 = self.Check() .set_name("c11") .set_parent(self)
        parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(c0) .add_destinations(turn1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(turn1) .add_destinations(c1)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(c1) .add_destinations(turn2)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(turn2) .add_destinations(c2)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(c2) .add_destinations(turn3)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(turn3) .add_destinations(c3)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(c3) .add_destinations(turn4)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(turn4) .add_destinations(c4)
        
        failuretrans5 = FailureTrans() .set_name("failuretrans5")
        failuretrans5 .add_sources(c4) .add_destinations(turn5)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(turn5) .add_destinations(c5)
        
        failuretrans6 = FailureTrans() .set_name("failuretrans6")
        failuretrans6 .add_sources(c5) .add_destinations(turn6)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(turn6) .add_destinations(c6)
        
        failuretrans7 = FailureTrans() .set_name("failuretrans7")
        failuretrans7 .add_sources(c6) .add_destinations(turn7)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(turn7) .add_destinations(c7)
        
        failuretrans8 = FailureTrans() .set_name("failuretrans8")
        failuretrans8 .add_sources(c7) .add_destinations(turn8)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(turn8) .add_destinations(c8)
        
        failuretrans9 = FailureTrans() .set_name("failuretrans9")
        failuretrans9 .add_sources(c8) .add_destinations(turn9)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(turn9) .add_destinations(c9)
        
        failuretrans10 = FailureTrans() .set_name("failuretrans10")
        failuretrans10 .add_sources(c9) .add_destinations(turn10)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(turn10) .add_destinations(c10)
        
        failuretrans11 = FailureTrans() .set_name("failuretrans11")
        failuretrans11 .add_sources(c10) .add_destinations(turn11)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(turn11) .add_destinations(c11)
        
        failuretrans12 = FailureTrans() .set_name("failuretrans12")
        failuretrans12 .add_sources(c11) .add_destinations(parentfails1)
        
        return self
