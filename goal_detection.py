from aim_fsm import *
import math
import time
import json

class FieldMemory:
    def __init__(self, stale_timeout=30.0):
        self.team_color = "orange" # "blue"
        self.stale_timeout = stale_timeout
        self.objects = {}
        self.goal_center = None
        self.goal_posts = []
        self.balls = []
        self.goal_width = None

    def update(self, world_map):
        now = time.time()
        for obj_id, obj in world_map.objects.items():
            is_vis = getattr(obj, "is_visible", False)
            entry = {
                "id":         obj_id,
                "type":       self._classify(obj),
                "obj":        obj,
                "x":          getattr(obj, "x", None),
                "y":          getattr(obj, "y", None),
                "distance":   getattr(obj, "sensor_distance", None),
                "heading":    getattr(obj, "sensor_bearing", None),
                "is_visible": is_vis,
                "last_seen":  now if is_vis
                              else self.objects.get(obj_id, {}).get("last_seen", 0),
            }
            self.objects[obj_id] = entry
        self.objects = {
            k: v for k, v in self.objects.items()
            if (now - v["last_seen"]) < (120 if v["type"] == "goalpost" else self.stale_timeout)
        }
        self._rebuild()

    def _classify(self, obj):
        if isinstance(obj, BlueBarrelObj):
            return "blue_goal"
        if isinstance(obj, OrangeBarrelObj):
            return "orange_goal"
        if isinstance(obj, SportsBallObj):
            return "ball"
        if isinstance(obj, WallObj):
            return "wall"
        return "other"

    def _rebuild(self):
        if self.team_color.lower() == "orange":
            self.goal_posts = [v for v in self.objects.values() if v["type"] == "orange_goal"]
        elif self.team_color.lower() == "blue":
            self.goal_posts = [v for v in self.objects.values() if v["type"] == "blue_goal"]
        else:
            print(f"ERROR: '{self.team_color}' is not a valid team color, choose orange or blue.")

        self.balls      = [v for v in self.objects.values() if v["type"] == "ball"]
        if len(self.goal_posts) >= 2:
            posts = sorted(self.goal_posts, key=lambda p: p["distance"] or 9999)[:2]
            x0, y0 = posts[0].get("x"), posts[0].get("y")
            x1, y1 = posts[1].get("x"), posts[1].get("y")
            if None not in (x0, y0, x1, y1):
                self.goal_center = ((x0 + x1) / 2, (y0 + y1) / 2)
                self.goal_width = math.sqrt((x0-x1)**2 + (y0-y1)**2)

    def snapshot_for_ai(self, robot_x=0, robot_y=0, robot_heading=0):
        def r(v):
            return round(v, 1) if v is not None else None
        data = {
            "robot": {"x": r(robot_x), "y": r(robot_y), "heading_deg": r(robot_heading)},
            "goal_center": {"x": r(self.goal_center[0]), "y": r(self.goal_center[1])}
                           if self.goal_center else None,
            "goal_width_mm": r(self.goal_width),
            "goal_posts": [
                {"id": p["id"], "x": r(p["x"]), "y": r(p["y"])}
                for p in self.goal_posts
            ],
            "goal": [
                {"id": b["id"], "x": r(b["x"]), "y": r(b["y"]),
                 "dist": r(b["distance"]), "heading_deg": r(b["heading"]),
                 "visible": b["is_visible"]}
                for b in self.goal_posts
            ],
        }
        return json.dumps(data)

class UpdateMemory(StateNode):
    def start(self, event=None):
        super().start(event)
        self.parent.field_memory.update(robot.world_map)
        mem = self.parent.field_memory
        print(f"[SLAM] Posts:{len(mem.goal_posts)} Goal:{len(mem.goal_posts)} "
              f"Goal:{'YES' if mem.goal_center else 'no'}")
        self.post_completion()


class FindGoal(StateNode):
    """
    Visible goal > post_data (obj).
    Remembered goal > post_success.
    Nothing > post_failure.
    """
    def start(self, event=None):
        super().start(event)
        mem = self.parent.field_memory

        visible = [b for b in mem.goal_posts if b["is_visible"]]
        if visible:
            closest = min(visible, key=lambda b: b["distance"] or 9999)
            self.parent.target_goal = closest
            d = closest.get('distance')
            h = closest.get('heading')
            print(f"[FIND] Goal VISIBLE dist={d if d else '?'} heading={h if h else '?'}")
            self.post_data(closest["obj"])
            return

        remembered = [b for b in mem.goal_posts if b.get("x") is not None]
        if remembered:
            closest = max(remembered, key=lambda b: b["last_seen"])
            self.parent.target_goal = closest
            age = time.time() - closest["last_seen"]
            print(f"[FIND] GOAL REMEMBERED ({closest['x']:.0f},{closest['y']:.0f}) "
                  f"{age:.1f}s ago")
            self.post_success()
            return

        print("[FIND] No goal at all")
        self.post_failure()

class goal_detection(StateNode):
    def start(self, event=None):
        super().start(event)

        mem = self.parent.field_memory

        if mem.goal_center is not None:

            self.parent.goal_center = mem.goal_center
            self.parent.goal_width = mem.goal_width

            print(
                f"[GOAL] Center=({mem.goal_center[0]:.1f}, "
                f"{mem.goal_center[1]:.1f}) "
                f"Width={mem.goal_width:.1f}"
            )

            self.post_success()

        else:

            print(
                f"[GOAL] Need 2 posts, currently have "
                f"{len(mem.goal_posts)}"
            )

            self.post_failure()