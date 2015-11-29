
from src.importer import

def main():
    baxter.start(False)

    joints = baxter._arms[Defines.LEFT].get_joint_angle()

    target = robotlib.GST(joints)
    target = target[7]
    pos = target[0:3,3]

    angle = mathlib.Quaternion.from_matrix(target[0:3,0:3])
    angle = angle.to_euler()

    print pos, angle

    th, dist = robotlib.inv_kin("left", joints, [0.5, -0.4, 0.2], [0, 0, 0])
    print dist

    baxter._arms[Defines.LEFT].set_joint_angle(th)

if __name__ == "__main__":
    main()