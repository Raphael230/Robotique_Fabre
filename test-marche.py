import math
import time
from kinematics import computeIK, trianglePoints, triangle
import pypot.dynamixel

# Groupes de pattes pour la marche en tripode
legs_ids = [
    [11, 12, 13],  # 0 - bas D
    [21, 22, 23],  # 1 - bas G
    [31, 32, 33],  # 2 - mid G
    [41, 42, 43],  # 3 - high G
    [51, 52, 53],  # 4 - high D
    [61, 62, 63],  # 5 - mid D
]

# Pattes du groupe A (bougent ensemble), groupe B (support)
group_A = [0, 2, 4]
group_B = [1, 5, 3]

PORT = "COM8"                     

# Positions types (mm)
STEP_HEIGHT = 30
STEP_LENGTH = 40
Z_DEFAULT = -80  # hauteur par défaut du pied

def foot_trajectory(t, T, lift):
    """ Renvoie la position (x, y, z) du pied selon la phase t dans le cycle """
    phase = (t % T) / T
    x = STEP_LENGTH * (phase - 0.5)
    y = 0
    z = Z_DEFAULT + (lift * STEP_HEIGHT * math.sin(math.pi * phase)) if lift else Z_DEFAULT
    return (x, y, z)

if __name__ == '__main__':
    dxl_io = pypot.dynamixel.DxlIO(PORT, baudrate=1000000)
    print("Moteurs détectés :", dxl_io.scan())
    time.sleep(2)

    CYCLE = 1.5  # durée d’un cycle complet (en secondes)

    while True:
        now = time.time()  # Récupère le temps actuel
        cmds = {}
        print("Start while true")
        time.sleep(5)

        for i, leg in enumerate(group_A):
            cycle_time = (now / CYCLE) % 3  # Calcul du temps dans le cycle
            angles = triangle(X, Z, H, W, cycle_time)  # Appel à la fonction triangle
            dxl_io.set_goal_position(dict(zip(leg, angles)))  # Applique les angles aux moteurs de la patte
            time.sleep(2) # Réduit le temps de pause pour une exécution plus fluide

        for i, leg in enumerate(group_B):
            cycle_time = ((now + CYCLE / 2) / CYCLE) % 3  # Décalage temporel pour le groupe B
            angles = triangle(X, Z, H, W, cycle_time)  # Appel à la fonction triangle
            dxl_io.set_goal_position(dict(zip(leg, angles)))  # Applique les angles aux moteurs de la patte
            print("boucle for group B")
            time.sleep(2) # Réduit le temps de pause pour une exécution plus fluide

            try:
                # Calcule les coordonnées (x, y, z) pour la patte
                x, y, z = foot_trajectory(now, T, lift=True)  # Utilise foot_trajectory pour les coordonnées
                a1, a2, a3 = computeIK(x, y, z)  # Calcul des angles des moteurs
                cmds[leg[0]] = a1  # Assignation des angles aux moteurs
                cmds[leg[1]] = a2
                cmds[leg[2]] = a3
                print("boucle try")
                time.sleep(2) # Réduit le temps de pause pour une exécution plus fluide
            except Exception as e:
                print(f"Erreur IK pour la patte {i} : {e}")

        dxl_io.set_goal_position(cmds)  # Applique les commandes finales aux moteurs
        print("Fin")
        time.sleep(5)  # Pause pour attendre avant de recommencer

# import math
# import time
# import numpy as np
# from kinematics import computeIK, trianglePoints, triangle
# import pypot.dynamixel

# # Groupes de pattes
# legs_ids = [
#     [11, 12, 13],  # 0 - bas D
#     [21, 22, 23],  # 1 - bas G
#     [31, 32, 33],  # 2 - mid G
#     [41, 42, 43],  # 3 - high G
#     [51, 52, 53],  # 4 - high D
#     [61, 62, 63],  # 5 - mid D
# ]

# group_A = [0, 2, 4]
# group_B = [1, 5, 3]

# # Paramètres du pas
# X = -20         # Position de base
# Z = -80         # Hauteur
# H = 30          # Hauteur du triangle
# W = 40          # Longueur du triangle
# CYCLE = 2.0     # Durée du cycle en secondes

# PORT = "COM8"
# print("COM8")

# def trianglePoints(x, z, h, w):
#     return [
#         (x, 0, z),
#         (x + w/2, 0, z - h),
#         (x + w, 0, z)
#     ]

# def triangle(x, z, h, w, t):
#     points = trianglePoints(x, z, h, w)
#     P1 = np.array(points[int(t) % 3])
#     P2 = np.array(points[(int(t) + 1) % 3])
#     T = math.fmod(t, 1)
#     pos = P2 * T + (1 - T) * P1
#     return computeIK(pos[0], pos[1], pos[2])

# # --- Initialisation ---
# dxl_io = pypot.dynamixel.DxlIO(PORT, baudrate=1000000)
# print("Moteurs détectés :", dxl_io.scan())

# time.sleep(5)
# print("Début de la boucle")

# while True:
#     now = time.time()
#     cmds = {}

#     for i in group_A:
#         print("boucle for A")
#         leg = legs_ids[i]
#         t = (now / CYCLE) % 3
#         angles = triangle(X, Z, H, W, t)
#         cmds[leg[0]] = angles[0]
#         cmds[leg[1]] = angles[1]
#         cmds[leg[2]] = angles[2]
#         time.sleep(2)

#     for i in group_B:
#         print("boucle for B")
#         leg = legs_ids[i]
#         t = ((now + CYCLE / 2) / CYCLE) % 3  # décalé
#         angles = triangle(X, Z, H, W, t)
#         cmds[leg[0]] = angles[0]
#         cmds[leg[1]] = angles[1]
#         cmds[leg[2]] = angles[2]
#         time.sleep(2)

#     dxl_io.set_goal_position(cmds)
#     time.sleep(5)# Fréquence de mise à jour fluide (~50Hz)

