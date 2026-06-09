from aim_fsm import *

class CheckForRobot(StateNode):
    def start(self, event=None):
        super().start(event)

        mem = self.parent.field_memory

        closest = None
        min_dist = float('inf')

        for obj in mem.objects.values():
            if isinstance(obj, RobotObj) and obj.is_visible:
                dx = obj.pose.x - self.robot.pose.x
                dy = obj.pose.y - self.robot.pose.y
                dist = (dx**2 + dy**2)**0.5

                if dist < min_dist:
                    min_dist = dist
                    closest = obj

        if closest is None:
            self.post_failure()
            return

        print("Robot detected: ", closest)
        print("Distance: ", min_dist)
        self.robot.nearby_robot = closest

        self.post_success()

class ReplanPath(StateNode):
    def start(self, event=None):
        super().start(event)
    
        print("Robot obstacle detected.")
        print("Robot registered as an RRT obstacle.")
        print("Path plans should route around it.")

        self.post_completion()