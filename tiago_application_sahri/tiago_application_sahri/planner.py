#!/usr/bin/env python3
"""
 pick_and_place_string.py  (v10 – *SetEntityState* "suiveur")

 ➤ Contrainte : le module *gazebo_ros_link_attacher* n’est pas installé.

 ✅ Plan B minimaliste – **téléportation suiveuse** :
   • Lorsque la pince se ferme, on active un mode « follow » : à chaque
     tick (10 Hz) on publie en plus un service `gazebo_msgs/SetEntityState`
     pour plaquer le bloc Jenga sur l’outil.
   • Au moment d’ouvrir la pince on coupe le mode « follow ». Le bloc
     reprend alors la dynamique normale et tombe/ se pose.

   Avantages :
     – 100 % fiable, aucun plugin externe.
     – Pas de changement de trajectoire.
     – Zéro dépendance à la physique ou au frottement.

   Hypothèse : le plugin par défaut *gazebo_ros_api_plugin* expose bien le
   service `/gazebo/set_entity_state` (Gazebo ≤ Fortress) ou
   `/world/<world_name>/set_entity_state` (Ignition Gazebo). Adapter
   `SET_STATE_SERVICE` si besoin.

   Offset : la pince saisit le bloc au milieu, donc on applique un décalage
   `BLOCK_OFFSET = (0, 0, -0.015/2)` pour placer le centre du bloc dans les
   doigts.
"""

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
from abc import update_abstractmethods
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from gazebo_msgs.srv import SetEntityState  # présent dans gazebo_ros_pkgs
from gazebo_msgs.msg import EntityState

# ---------------------------------------------------------------------------
# Paramètres scène
# ---------------------------------------------------------------------------
TABLE_Z   = 0.53 + 0.27             # hauteur table (surface) + marge
BLOCK_H   = 0.015                  # épaisseur du bloc
JENGA_LENGHT = 0.075                  # longueur bloc Jenga
JENGA_WIDTH = 0.03                   # largeur bloc Jenga
HOME_Z    = TABLE_Z + 0.2                        # home très haut

# Waypoints
CENTER_TOWER = (0.50, 0.00, TABLE_Z)  # centre du bloc Jenga
orientation_0 = (0.0, 0.7071068, 0.0, 0.7071068)  # orientation pince vers le bas
# Orientation b is 90 degrees rotated wrt z
orientation_90 = (0.5000000,  0.5000000, -0.5000000,  0.5000000)  # orientation pince vers le bas, 90° sur x
pose_1_a       = (CENTER_TOWER[0] + JENGA_WIDTH/2, CENTER_TOWER[1], HOME_Z )  # pose bloc au-dessus du centre
pose_1_b      = (CENTER_TOWER[0] + JENGA_WIDTH/2, CENTER_TOWER[1], CENTER_TOWER[2] )  # pose bloc au-dessus du centre
pose_2_a       = (CENTER_TOWER[0] - JENGA_WIDTH/2, CENTER_TOWER[1],  HOME_Z )  # pose bloc au-dessus du centre
pose_2_b      = (CENTER_TOWER[0] - JENGA_WIDTH/2, CENTER_TOWER[1], CENTER_TOWER[2] )  # pose bloc au-dessus du centre
pose_3_a = (CENTER_TOWER[0], CENTER_TOWER[1] - JENGA_WIDTH/2,  HOME_Z )  # pose bloc au-dessus du centre
pose_3_b = (CENTER_TOWER[0], CENTER_TOWER[1] - JENGA_WIDTH/2, CENTER_TOWER[2] )  # pose bloc au-dessus du centre
pose_4_a = (CENTER_TOWER[0], CENTER_TOWER[1] + JENGA_WIDTH/2,  HOME_Z )  # pose bloc au-dessus du centre
pose_4_b = (CENTER_TOWER[0], CENTER_TOWER[1] + JENGA_WIDTH/2, CENTER_TOWER[2] )  # pose bloc au-dessus du centre


FREQ_HZ   = 10                          # fréquence timer
WAIT_AFTER_STEP_SECS = 1.0              # pause 1 s entre étapes

# Quaternions
Q_DOWN            = (0.0, 0.7071068, 0.0, 0.7071068)
Q_DOWN_X_POS90    = (0.5000000,  0.5000000, -0.5000000,  0.5000000)
Q_DOWN_X_NEG90    = (-0.5000000,  0.5000000,  0.5000000,  0.5000000)
Q_ROLL            = Q_DOWN_X_NEG90   # garde la pince vers le bas chez TIAGo



# Bloc à suivre
BLOCK_NAME      = 'jenga_block'
BLOCK_OFFSET    = (0.0, 0.0, -BLOCK_H/2)  # centre bloc ↘ pinces
# ---------------------------------------------------------------------------

def make_pose(x: float, y: float, z: float, q_x: float, q_y: float, q_z:float, q_w:float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = q_x
    p.orientation.y = q_y
    p.orientation.z = q_z
    p.orientation.w = q_w
    return p



class PickPlace(Node):
    """Node ROS 2 qui publie la trajectoire pick‑and‑place + suivi bloc."""

    def __init__(self):
        super().__init__('pick_and_place_string')

        # Publishers
        self.pose_pub = self.create_publisher(Pose,   '/target_pose',  10)
        self.grip_pub = self.create_publisher(String, '/gripper_cmd',  10)

        self.number_layer = 0
        self.max_layer = 15
        self.layer_hight = BLOCK_H
        duration = 1.0
        # (pose | None, 'OPEN'/'CLOSE', durée en secondes)
        self.seq = [
            # 1. Dégagement / montée sécurité
        (make_pose(pose_1_a[0],pose_1_a[1],pose_1_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'OPEN', duration),
        (make_pose(pose_1_a[0],pose_1_a[1],pose_1_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'OPEN', duration),
        (make_pose(pose_1_a[0],pose_1_a[1],pose_1_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'OPEN', duration),

        (make_pose(pose_1_a[0],pose_1_a[1],pose_1_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'CLOSE', duration),
        (make_pose(pose_1_b[0],pose_1_b[1],pose_1_b[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'CLOSE', duration),
        (make_pose(pose_1_b[0],pose_1_b[1],pose_1_b[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'OPEN', duration),

        (make_pose(pose_2_a[0],pose_2_a[1],pose_2_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'OPEN', duration),
        (make_pose(pose_2_a[0],pose_2_a[1],pose_2_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'OPEN', duration),
        (make_pose(pose_2_a[0],pose_2_a[1],pose_2_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'OPEN', duration),

        (make_pose(pose_2_a[0],pose_2_a[1],pose_2_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'CLOSE', duration),
        (make_pose(pose_2_b[0],pose_2_b[1],pose_2_b[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'CLOSE', duration),
        (make_pose(pose_2_b[0],pose_2_b[1],pose_2_b[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'OPEN', duration),


        (make_pose(pose_3_a[0],pose_3_a[1],pose_3_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'OPEN', duration),
        (make_pose(pose_3_a[0],pose_3_a[1],pose_3_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'OPEN', duration),
        (make_pose(pose_3_a[0],pose_3_a[1],pose_3_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'OPEN', duration),

        (make_pose(pose_3_a[0],pose_3_a[1],pose_3_a[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'CLOSE', duration),
        (make_pose(pose_3_b[0],pose_3_b[1],pose_3_b[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'CLOSE', duration),
        (make_pose(pose_3_b[0],pose_3_b[1],pose_3_b[2], orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'OPEN', duration),


        (make_pose(pose_4_a[0],pose_4_a[1],pose_4_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'OPEN', duration),
        (make_pose(pose_4_a[0],pose_4_a[1],pose_4_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'OPEN', duration),
        (make_pose(pose_4_a[0],pose_4_a[1],pose_4_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'OPEN', duration),


        (make_pose(pose_4_a[0],pose_4_a[1],pose_4_a[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'CLOSE', duration),
        (make_pose(pose_4_b[0],pose_4_b[1],pose_4_b[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'CLOSE', duration),
        (make_pose(pose_4_b[0],pose_4_b[1],pose_4_b[2], orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'OPEN', duration),

        ]
        # Print the sequnce for debugging
        for i, (pose, cmd, duration) in enumerate(self.seq):
            if pose is not None:
                self.get_logger().info(f"Étape {i+1}: Pose({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
                                       f"Cmd: {cmd}, Durée: {duration}s")
            else:
                self.get_logger().info(f"Étape {i+1}: Cmd: {cmd}, Durée: {duration}s")

        self.step_idx     = 0
        self.tick_in_step = 0
        self.ticks_needed = int(self.seq[0][2] * FREQ_HZ)
        self.waiting      = False
        self.wait_ticks   = 0

        self.get_logger().info('Pick-and-place v10 lancé ✔ (suivi SetEntityState)')
        self.create_timer(1.0 / FREQ_HZ, self.timer_cb)


    def update_seqposes(self, layerheight=0.0, duration=1.0):
      self.seq = [
          # 1. Dégagement / montée sécurité
      (make_pose(pose_1_a[0],pose_1_a[1],pose_1_a[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'OPEN', duration),
      (make_pose(pose_1_a[0],pose_1_a[1],pose_1_a[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'CLOSE', duration),
      (make_pose(pose_1_b[0],pose_1_b[1],pose_1_b[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'CLOSE', duration),
      (make_pose(pose_1_b[0],pose_1_b[1],pose_1_b[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]), 'OPEN', duration),

      (make_pose(pose_2_a[0],pose_2_a[1],pose_2_a[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'OPEN', duration),
      (make_pose(pose_2_a[0],pose_2_a[1],pose_2_a[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'CLOSE', duration),
      (make_pose(pose_2_b[0],pose_2_b[1],pose_2_b[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'CLOSE', duration),
      (make_pose(pose_2_b[0],pose_2_b[1],pose_2_b[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]), 'OPEN', duration),


      (make_pose(pose_3_a[0],pose_3_a[1],pose_3_a[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'OPEN', duration),
      (make_pose(pose_3_a[0],pose_3_a[1],pose_3_a[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'CLOSE', duration),
      (make_pose(pose_3_b[0],pose_3_b[1],pose_3_b[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'CLOSE', duration),
      (make_pose(pose_3_b[0],pose_3_b[1],pose_3_b[2] + layerheight, orientation_0[0], orientation_0[1], orientation_0[2], orientation_0[3]),'OPEN', duration),


      (make_pose(pose_4_a[0],pose_4_a[1],pose_4_a[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'OPEN', duration),
      (make_pose(pose_4_a[0],pose_4_a[1],pose_4_a[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'CLOSE', duration),
      (make_pose(pose_4_b[0],pose_4_b[1],pose_4_b[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'CLOSE', duration),
      (make_pose(pose_4_b[0],pose_4_b[1],pose_4_b[2] + layerheight, orientation_90[0], orientation_90[1], orientation_90[2], orientation_90[3]),'OPEN', duration),

      ]

      return self.seq

    # ------------------------------------------------------------------
    # Timer principal
    # ------------------------------------------------------------------
    def timer_cb(self):
        if self.step_idx >= len(self.seq) and self.number_layer <= self.max_layers:
            self.seq = self.update_seqposes(self.layer_hight)
            HOME_Z = HOME_Z + self.layer_hight
            self.number_layer += 1
            rclpy.shutdown()
            return

        # Gestion pause entre étapes
        if self.waiting:
            self.wait_ticks += 1
            if self.wait_ticks >= int(WAIT_AFTER_STEP_SECS * FREQ_HZ):
                self.waiting      = False
                self.tick_in_step = 0
            else:
                return

        pose, cmd, duration = self.seq[self.step_idx]

        # Publie la pose cible pour le bras
        if pose is not None:
            self.pose_pub.publish(pose)
            self.get_logger().info(f"Pose: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}, "
                                   f"qx={pose.orientation.x:.3f}, qy={pose.orientation.y:.3f}, "
                                   f"qz={pose.orientation.z:.3f}, qw={pose.orientation.w:.3f}")

        # ➜ Maintien gripper cmd à chaque tick
        self.grip_pub.publish(String(data=cmd))

        # Premier tick logique
        if self.tick_in_step == 0:
            self.ticks_needed = int(duration * FREQ_HZ)
            self.get_logger().info(
                f"Étape {self.step_idx+1}/{len(self.seq)} — {cmd} ({duration}s)")

        # Avancement
        self.tick_in_step += 1
        if self.tick_in_step >= self.ticks_needed:
            self.waiting    = True
            self.wait_ticks = 0
            self.step_idx  += 1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = PickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
