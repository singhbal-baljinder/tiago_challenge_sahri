#!/usr/bin/env python3
"""
build_jenga_tower.py (v8)
Cycle sûr « remonte‑charge‑dépose » garanti :
  • **Toujours remonte à HOME_Z** puis revient côté START_SIDE_XY entre CHAQUE
    bloc — pose explicitement publiée, plus de pas “None”.
  • Séquence par bloc (A→H) :
        A. Pose latérale @HOME_Z — gripper OPEN 5 s (chargement humain)
        B. CLOSE 2 s (saisie)
        C. Translation @HOME_Z au‑dessus de la cible 2 s — CLOSE
        D. Descente @z_safe 1 s — CLOSE
        E. Descente @z_release 1 s — CLOSE
        F. OPEN 2 s (dépôt)
        G. Remontée @HOME_Z 2 s — OPEN
        H. Retour latéral @HOME_Z 2 s — OPEN
  • HOME_Z = hauteur finale + 30 cm ≈ 1,33 m.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
TABLE_Z   = 0.35 + 0.21
BLOCK_L   = 0.075
BLOCK_W   = 0.025
BLOCK_H   = 0.015
CENTER_XY = (0.70, 0.15)
SIDE_OFFSET = (-0.50, -0.35)
START_SIDE_XY = (CENTER_XY[0] + SIDE_OFFSET[0], CENTER_XY[1] + SIDE_OFFSET[1])
SAFE_CLEARANCE  = 0.12
RELEASE_OFFSET  = 0.01
LEVELS   = 10
FREQ_HZ  = 10
WAIT_AFTER_STEP_SECS = 0.5  # juste 0,5 s d’intervalle logique
MAX_TOWER_Z = TABLE_Z + (LEVELS - 1) * BLOCK_H + BLOCK_H / 2.0
HOME_Z      = MAX_TOWER_Z + 0.30  # ≈ 1,33 m
OFFSET = (BLOCK_L + BLOCK_W) / 2.0
Q_DOWN         = (0.0,  0.7071068, 0.0, 0.7071068)
Q_DOWN_YAW90   = (-0.5, 0.5, 0.5, 0.5)
def make_pose(x: float, y: float, z: float, q=Q_DOWN) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose
class BuildJenga(Node):
    def __init__(self):
        super().__init__('build_jenga_tower')
        self.pose_pub = self.create_publisher(Pose,   '/target_pose',  10)
        self.grip_pub = self.create_publisher(String, '/gripper_cmd',  10)
        self.seq = self._generate_sequence()
        self.step_idx = 0
        self.tick_in_step = 0
        self.ticks_needed = int(self.seq[0][2] * FREQ_HZ)
        self.waiting = False
        self.wait_ticks = 0
        self.create_timer(1.0 / FREQ_HZ, self.timer_cb)
        self.get_logger().info('Sequence v8 started')
    def _generate_sequence(self):
        seq = []
        cx, cy = CENTER_XY
        sx, sy = START_SIDE_XY
        # Mise en place initiale côté @HOME_Z
        seq.append((make_pose(sx, sy, HOME_Z, Q_DOWN), 'OPEN', 2))
        for level in range(LEVELS):
            z_block   = TABLE_Z + BLOCK_H / 2.0 + level * BLOCK_H
            z_safe    = z_block + SAFE_CLEARANCE
            z_release = z_block + RELEASE_OFFSET
            if level % 2 == 0:
                offsets = [(-OFFSET, 0.0), (OFFSET, 0.0)]
                q_layer = Q_DOWN
            else:
                offsets = [(0.0, -OFFSET), (0.0, OFFSET)]
                q_layer = Q_DOWN_YAW90
            for ox, oy in offsets:
                x = cx + ox
                y = cy + oy
                # A. Attente côté @HOME_Z (OPEN 5 s)
                seq.append((make_pose(sx, sy, HOME_Z, Q_DOWN), 'OPEN', 5))
                # B. Fermeture (CLOSE 2 s)
                seq.append((make_pose(sx, sy, HOME_Z, Q_DOWN), 'CLOSE', 2))
                # C. Translation haute vers cible
                seq.append((make_pose(x, y, HOME_Z, q_layer), 'CLOSE', 2))
                # D. Descente @z_safe
                seq.append((make_pose(x, y, z_safe, q_layer), 'CLOSE', 1))
                # E. Descente @z_release
                seq.append((make_pose(x, y, z_release, q_layer), 'CLOSE', 1))
                # F. Ouverture dépôt (OPEN 2 s)
                seq.append((None, 'OPEN', 2))
                # G. Remontée @HOME_Z
                seq.append((make_pose(x, y, HOME_Z, q_layer), 'OPEN', 2))
                # H. Retour latéral @HOME_Z (OPEN 2 s)
                seq.append((make_pose(sx, sy, HOME_Z, Q_DOWN), 'OPEN', 2))
            # Pause symbolique entre étages
            seq.append((make_pose(sx, sy, HOME_Z, Q_DOWN), 'OPEN', 1))
        # Fin — bras reste côté @HOME_Z 3 s
        seq.append((make_pose(sx, sy, HOME_Z, Q_DOWN), 'OPEN', 3))
        return seq
    def timer_cb(self):
        if self.step_idx >= len(self.seq):
            self.get_logger().info('Tower completed :white_check_mark:')
            rclpy.shutdown()
            return
        # Attente inter‑étape
        if self.waiting:
            self.wait_ticks += 1
            if self.wait_ticks >= int(WAIT_AFTER_STEP_SECS * FREQ_HZ):
                self.waiting = False
                self.tick_in_step = 0
            else:
                return
        pose, cmd, duration = self.seq[self.step_idx]
        if pose is not None:
            self.pose_pub.publish(pose)
        self.grip_pub.publish(String(data=cmd))
        if self.tick_in_step == 0:
            self.ticks_needed = int(duration * FREQ_HZ)
            self.get_logger().info(f"Step {self.step_idx+1}/{len(self.seq)} — {cmd} ({duration}s)")
        self.tick_in_step += 1
        if self.tick_in_step >= self.ticks_needed:
            self.waiting = True
            self.wait_ticks = 0
            self.step_idx += 1
def main():
    rclpy.init()
    node = BuildJenga()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
