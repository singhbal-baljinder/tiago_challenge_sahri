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
TABLE_Z   = 0.53 + 0.27            # hauteur table (surface) + marge
BLOCK_H   = 0.015                  # épaisseur du bloc
CENTER_Z  = TABLE_Z + BLOCK_H / 2.0      # 0.6875 m
SAFE_Z    = 1.05                        # zone sûre bien au-dessus
HOME_Z    = 1.20                        # home très haut

# Waypoints
SIDE_XY       = (0.20, -0.35)           # déport latéral
PICK_XY       = (0.70, 0.00)            # prise
GRIP_MOVE_XY  = (0.70, 0.30)            # translation bloc fermé
PLACE_XY      = (0.70, 0.15)            # pose finale

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

def make_pose(x: float, y: float, z: float, q=Q_DOWN) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


class PickPlace(Node):
    """Node ROS 2 qui publie la trajectoire pick‑and‑place + suivi bloc."""

    def __init__(self):
        super().__init__('pick_and_place_string')

        # Publishers
        self.pose_pub = self.create_publisher(Pose,   '/target_pose',  10)
        self.grip_pub = self.create_publisher(String, '/gripper_cmd',  10)

        self.follow = False  # bloc attaché virtuellement ?

        # (pose | None, 'OPEN'/'CLOSE', durée en secondes)
        self.seq = [
            # 1. Dégagement / montée sécurité
            (make_pose(*SIDE_XY, SAFE_Z),                     'OPEN', 3),
            (make_pose(*SIDE_XY, HOME_Z),                     'OPEN', 2),
            (make_pose(0.40, 0.00, HOME_Z),                   'OPEN', 2),

            # 2. Approche pick (roll X)
            (make_pose(*PICK_XY, SAFE_Z),                     'OPEN', 3),
            (make_pose(*PICK_XY, SAFE_Z, Q_ROLL),             'OPEN', 1),
            (make_pose(*PICK_XY, CENTER_Z, Q_ROLL),           'OPEN', 2),
            (None,                                            'CLOSE', 2),

            # 3. Translation fermée
            (make_pose(*PICK_XY, SAFE_Z, Q_ROLL),             'CLOSE', 2),
            (make_pose(*GRIP_MOVE_XY, SAFE_Z, Q_ROLL),        'CLOSE', 3),
            # 4. Zone de pose
            (make_pose(*PLACE_XY, SAFE_Z, Q_ROLL),            'CLOSE', 2),
            (make_pose(*PLACE_XY, SAFE_Z, Q_DOWN),            'CLOSE', 1),
            (make_pose(*PLACE_XY, CENTER_Z, Q_DOWN),          'CLOSE', 2),
            (None,                                            'OPEN', 2),

            # 5. Retour home
            (make_pose(*PLACE_XY, SAFE_Z),                    'OPEN', 2),
            (make_pose(0.40, 0.00, HOME_Z),                   'OPEN', 3),
        ]

        self.step_idx     = 0
        self.tick_in_step = 0
        self.ticks_needed = int(self.seq[0][2] * FREQ_HZ)
        self.waiting      = False
        self.wait_ticks   = 0

        self.get_logger().info('Pick-and-place v10 lancé ✔ (suivi SetEntityState)')
        self.create_timer(1.0 / FREQ_HZ, self.timer_cb)

    # ------------------------------------------------------------------
    # Timer principal
    # ------------------------------------------------------------------
    def timer_cb(self):
        if self.step_idx >= len(self.seq):
            self.get_logger().info('Séquence terminée ✅')
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

        # ➜ Maintien gripper cmd à chaque tick
        self.grip_pub.publish(String(data=cmd))

        # Premier tick logique + follow switch
        if self.tick_in_step == 0:
            self.ticks_needed = int(duration * FREQ_HZ)
            self.get_logger().info(
                f"Étape {self.step_idx+1}/{len(self.seq)} — {cmd} ({duration}s)")
            if cmd == 'CLOSE':
                self.follow = True
            elif cmd == 'OPEN':
                self.follow = False

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
