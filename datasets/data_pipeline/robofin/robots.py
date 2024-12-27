from pathlib import Path
from geometrout.transform import SE3, SO3
import importlib
from ikfast_franka_panda import get_fk, get_ik
# importlib.import_module("ikfast_panda_arm")
# print('\nikfast module {} imported successful'.format("ikfast_panda_arm"))
# from ikfast_panda_arm import get_fk, get_ik
# ikfast_panda_arm
# import sys
# import os
# import time
# from pybullet_tools.ikfast.compile import compile_ikfast
# robot_name = 'panda_arm'
# compile_ikfast(module_name='ikfast_{}'.format(robot_name),
#                    cpp_filename='ikfast_{}.cpp'.format(robot_name))
import numpy as np


class FrankaRobot():
    # TODO remove this after making this more general
    JOINT_LIMITS = np.array(
        [
            (-2.8973, 2.8973),
            (-1.7628, 1.7628),
            (-2.8973, 2.8973),
            (-3.0718, -0.0698),
            (-2.8973, 2.8973),
            (-0.0175, 3.7525),
            (-2.8973, 2.8973),
        ]
    )

    VELOCITY_LIMIT = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
    ACCELERATION_LIMIT = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
    DOF = 7
    EFF_LIST = set(["panda_link8", "right_gripper", "panda_grasptarget"])
    EFF_T_LIST = {
        ("panda_link8", "panda_hand"): SE3(
            xyz=[0, 0, 0],
            quaternion=[0.9238795325113726, 0.0, 0.0, -0.3826834323648827],
        ),
        ("panda_link8", "right_gripper"): SE3(
            xyz=[0, 0, 0.1], rpy=[0, 0, 2.35619449019]
        ),
        ("panda_link8", "panda_grasptarget"): SE3(
            xyz=[0, 0, 0.105], rpy=[0, 0, -0.785398163397]
        ),
        ("panda_hand", "right_gripper"): SE3(
            xyz=[0.0, 0.0, 0.1], quaternion=[0.0, 0.0, 0.0, 1.0]
        ),
    }
    # These are strings because that's needed for Bullet
    urdf = str(Path(__file__).parent / "urdf" / "robots" / "panda_arm.urdf")#yanshi JIQIR 
    hd_urdf = str(Path(__file__).parent / "urdf" / "franka_panda" / "hd_panda.urdf")#ACT ROBOT
    # This can be a Path because it's only ever used from Python
    pointcloud_cache = Path(__file__).parent / "pointcloud" / "cache" / "franka"
    NEUTRAL = np.array(
        [
            -0.017792060227770554,
            -0.7601235411041661,
            0.019782607023391807,
            -2.342050140544315,
            0.029840531355804868,
            1.5411935298621688,
            0.7534486589746342,
        ]
    )
    # Tuples of radius in meters and the corresponding links, values are centers on that link
    SPHERES = [
        (0.08, {"panda_link0": [[0.0, 0.0, 0.05]]}),
        (
            0.06,
            {
                "panda_link1": [
                    [0.0, -0.08, 0.0],
                    [0.0, -0.03, 0.0],
                    [0.0, 0.0, -0.12],
                    [0.0, 0.0, -0.17],
                ],
                "panda_link2": [
                    [0.0, 0.0, 0.03],
                    [0.0, 0.0, 0.08],
                    [0.0, -0.12, 0.0],
                    [0.0, -0.17, 0.0],
                ],
                "panda_link3": [[0.0, 0.0, -0.1]],
                "panda_link4": [[-0.08, 0.095, 0.0]],
                "panda_link5": [
                    [0.0, 0.055, 0.0],
                    [0.0, 0.075, 0.0],
                    [0.0, 0.0, -0.22],
                ],
            },
        ),
        (
            0.05,
            {
                "panda_link3": [[0.0, 0.0, -0.06]],
                "panda_link5": [[0.0, 0.05, -0.18]],
                "panda_link6": [[0.0, 0.0, 0.0], [0.08, -0.01, 0.0]],
                "panda_link7": [[0.0, 0.0, 0.07]],
            },
        ),
        (
            0.055,
            {
                "panda_link3": [[0.08, 0.06, 0.0], [0.08, 0.02, 0.0]],
                "panda_link4": [
                    [0.0, 0.0, 0.02],
                    [0.0, 0.0, 0.06],
                    [-0.08, 0.06, 0.0],
                ],
            },
        ),
        (
            0.025,
            {
                "panda_link5": [
                    [0.01, 0.08, -0.14],
                    [0.01, 0.085, -0.11],
                    [0.01, 0.09, -0.08],
                    [0.01, 0.095, -0.05],
                    [-0.01, 0.08, -0.14],
                    [-0.01, 0.085, -0.11],
                    [-0.01, 0.09, -0.08],
                    [-0.01, 0.095, -0.05],
                ],
                "panda_link7": [[0.02, 0.04, 0.08], [0.04, 0.02, 0.08]],
            },
        ),
        (0.052, {"panda_link6": [[0.08, 0.035, 0.0]]}),
        (0.02, {"panda_link7": [[0.04, 0.06, 0.085], [0.06, 0.04, 0.085]]}),
        (
            0.028,
            {
                "panda_hand": [
                    [0.0, -0.075, 0.01],
                    [0.0, -0.045, 0.01],
                    [0.0, -0.015, 0.01],
                    [0.0, 0.015, 0.01],
                    [0.0, 0.045, 0.01],
                    [0.0, 0.075, 0.01],
                ]
            },
        ),
        (
            0.026,
            {
                "panda_hand": [
                    [0.0, -0.075, 0.03],
                    [0.0, -0.045, 0.03],
                    [0.0, -0.015, 0.03],
                    [0.0, 0.015, 0.03],
                    [0.0, 0.045, 0.03],
                    [0.0, 0.075, 0.03],
                ]
            },
        ),
        (
            0.024,
            {
                "panda_hand": [
                    [0.0, -0.075, 0.05],
                    [0.0, -0.045, 0.05],
                    [0.0, -0.015, 0.05],
                    [0.0, 0.015, 0.05],
                    [0.0, 0.045, 0.05],
                    [0.0, 0.075, 0.05],
                ]
            },
        ),
        (
            0.012,
            {
                "panda_leftfinger": [
                    [0, 0.015, 0.022],
                    [0, 0.008, 0.044],
                ],
                "panda_rightfinger": [
                    [0, -0.015, 0.022],
                    [0, -0.008, 0.044],
                ],
            },
        ),
    ]

    @staticmethod
    def within_limits(config):
        # We have to add a small buffer because of float math
        return np.all(config >= FrankaRobot.JOINT_LIMITS[:, 0] - 1e-5) and np.all(
            config <= FrankaRobot.JOINT_LIMITS[:, 1] + 1e-5
        )

    @staticmethod
    def random_neutral(method="normal"):
        if method == "normal":
            return np.clip(
                FrankaRobot.NEUTRAL + np.random.normal(0, 0.25, 7),
                FrankaRobot.JOINT_LIMITS[:, 0],
                FrankaRobot.JOINT_LIMITS[:, 1],
            )
        if method == "uniform":
            # No need to clip this because it's always within range
            return FrankaRobot.NEUTRAL + np.random.uniform(0, 0.25, 7)
        assert False, "method must be either normal or uniform"

    @staticmethod
    def fk(config, eff_frame="right_gripper"):
        """
        Returns the SE3 frame of the end effector
        """
        assert (
            eff_frame in FrankaRobot.EFF_LIST
        ), "Default FK only calculated for a valid end effector frame"
        pos, rot = get_fk(config)
        mat = np.eye(4)
        mat[:3, :3] = np.asarray(rot)
        mat[:3, 3] = np.asarray(pos)
        if eff_frame == "panda_link8":
            return SE3(matrix=mat)
        elif eff_frame == "right_gripper":
            return (
                SE3(matrix=mat)
                @ FrankaRobot.EFF_T_LIST[("panda_link8", "right_gripper")]
            )
        else:
            return (
                SE3(matrix=mat)
                @ FrankaRobot.EFF_T_LIST[("panda_link8", "panda_grasptarget")]
            )

    @staticmethod
    def ik(pose, panda_link7, eff_frame="right_gripper"):
        """
        :param pose: SE3 pose expressed in specified end effector frame
        :param panda_link7: Value for the joint panda_link7, other IK can be calculated with this joint value set.
            Must be within joint range
        :param eff_frame: Desired end effector frame, must be among [panda_link8, right_gripper, panda_grasptarget]
        :return: Typically 4 solutions to IK
        """
        assert (
            eff_frame in FrankaRobot.EFF_LIST
        ), "IK only calculated for a valid end effector frame"
        if eff_frame == "right_gripper":
            pose = (
                pose @ FrankaRobot.EFF_T_LIST[("panda_link8", "right_gripper")].inverse
            )
        elif eff_frame == "panda_grasptarget":
            pose = (
                pose
                @ FrankaRobot.EFF_T_LIST[("panda_link8", "panda_grasptarget")].inverse
            )
        rot = pose.so3.matrix.tolist()
        pos = pose.xyz
        assert (
            panda_link7 >= FrankaRobot.JOINT_LIMITS[-1, 0]
            and panda_link7 <= FrankaRobot.JOINT_LIMITS[-1, 1]
        ), f"Value for floating joint must be within range {FrankaRobot.JOINT_LIMITS[-1, :].tolist()}"
        solutions = [np.asarray(s) for s in get_ik(pos, rot, [panda_link7])]
        return [
            s
            for s in solutions
            if (
                np.all(s >= FrankaRobot.JOINT_LIMITS[:, 0])
                and np.all(s <= FrankaRobot.JOINT_LIMITS[:, 1])
            )
        ]

    @staticmethod
    def random_configuration():
        limits = FrankaRobot.JOINT_LIMITS
        return (limits[:, 1] - limits[:, 0]) * (np.random.rand(7)) + limits[:, 0]

    @staticmethod
    def random_ik(pose, eff_frame="right_gripper"):
        config = FrankaRobot.random_configuration()
        try:
            return FrankaRobot.ik(pose, config[-1], eff_frame)
        except:
            raise Exception(f"IK failed with {pose}")

    @staticmethod
    def collision_free_ik(sim, sim_franka, pose, frame="right_gripper", retries=1000):
        for i in range(retries + 1):
            samples = FrankaRobot.random_ik(pose, "right_gripper")
            for sample in samples:
                sim_franka.marionette(sample)
                if not sim.in_collision(sim_franka, check_self=True):
                    return sample
        return None


class FrankaRealRobot(FrankaRobot):
    JOINT_LIMITS = np.array(
        [
            (-2.8973, 2.8973),
            (-1.7628, 1.7628),
            (-2.8973, 2.8973),
            (-3.0718, -0.0698),
            (-2.8973, 2.8973),
            (0.05, 3.75),
            (-2.8973, 2.8973),
        ]
    )

    @staticmethod
    def within_limits(config):
        # We have to add a small buffer because of float math
        return np.all(config >= FrankaRealRobot.JOINT_LIMITS[:, 0] - 1e-5) and np.all(
            config <= FrankaRealRobot.JOINT_LIMITS[:, 1] + 1e-5
        )

    @staticmethod
    def random_neutral(method="normal"):
        if method == "normal":
            return np.clip(
                FrankaRealRobot.NEUTRAL + np.random.normal(0, 0.25, 7),
                FrankaRealRobot.JOINT_LIMITS[:, 0],
                FrankaRealRobot.JOINT_LIMITS[:, 1],
            )
        if method == "uniform":
            # No need to clip this because it's always within range
            return FrankaRealRobot.NEUTRAL + np.random.uniform(0, 0.25, 7)
        assert False, "method must be either normal or uniform"

    @staticmethod
    def fk(config, eff_frame="right_gripper"):
        """
        Returns the SE3 frame of the end effector
        """
        assert (
            eff_frame in FrankaRealRobot.EFF_LIST
        ), "Default FK only calculated for a valid end effector frame"
        pos, rot = get_fk(config)
        mat = np.eye(4)
        mat[:3, :3] = np.asarray(rot)
        mat[:3, 3] = np.asarray(pos)
        if eff_frame == "panda_link8":
            return SE3(matrix=mat)
        elif eff_frame == "right_gripper":
            return (
                SE3(matrix=mat)
                @ FrankaRealRobot.EFF_T_LIST[("panda_link8", "right_gripper")]
            )
        else:
            return (
                SE3(matrix=mat)
                @ FrankaRealRobot.EFF_T_LIST[("panda_link8", "panda_grasptarget")]
            )

    @staticmethod
    def ik(pose, panda_link7, eff_frame="right_gripper"):
        """
        :param pose: SE3 pose expressed in specified end effector frame
        :param panda_link7: Value for the joint panda_link7, other IK can be calculated with this joint value set.
            Must be within joint range
        :param eff_frame: Desired end effector frame, must be among [panda_link8, right_gripper, panda_grasptarget]
        :return: Typically 4 solutions to IK
        """
        assert (
            eff_frame in FrankaRealRobot.EFF_LIST
        ), "IK only calculated for a valid end effector frame"
        if eff_frame == "right_gripper":
            pose = (
                pose
                @ FrankaRealRobot.EFF_T_LIST[("panda_link8", "right_gripper")].inverse
            )
        elif eff_frame == "panda_grasptarget":
            pose = (
                pose
                @ FrankaRealRobot.EFF_T_LIST[
                    ("panda_link8", "panda_grasptarget")
                ].inverse
            )
        rot = pose.so3.matrix.tolist()
        pos = pose.xyz
        assert (
            panda_link7 >= FrankaRealRobot.JOINT_LIMITS[-1, 0]
            and panda_link7 <= FrankaRealRobot.JOINT_LIMITS[-1, 1]
        ), f"Value for floating joint must be within range {FrankaRealRobot.JOINT_LIMITS[-1, :].tolist()}"
        solutions = [np.asarray(s) for s in get_ik(pos, rot, [panda_link7])]
        return [
            s
            for s in solutions
            if (
                np.all(s >= FrankaRealRobot.JOINT_LIMITS[:, 0])
                and np.all(s <= FrankaRealRobot.JOINT_LIMITS[:, 1])
            )
        ]

    @staticmethod
    def random_configuration():
        limits = FrankaRealRobot.JOINT_LIMITS
        return (limits[:, 1] - limits[:, 0]) * (np.random.rand(7)) + limits[:, 0]

    @staticmethod
    def random_ik(pose, eff_frame="right_gripper"):
        config = FrankaRealRobot.random_configuration()
        try:
            return FrankaRealRobot.ik(pose, config[-1], eff_frame)
        except:
            raise Exception(f"IK failed with {pose}")

    @staticmethod
    def collision_free_ik(
        sim, sim_franka, selfcc, pose, frame="right_gripper", retries=1000
    ):
        for i in range(retries + 1):
            samples = FrankaRealRobot.random_ik(pose, "right_gripper")
            for sample in samples:
                sim_franka.marionette(sample)
                if not (
                    sim.in_collision(sim_franka, check_self=True)
                    or selfcc.has_self_collision(sample)
                ):
                    return sample
        return None


class FrankaGripper:
    JOINT_LIMITS = None
    DOF = 6
    urdf = str(Path(__file__).parent / "urdf" / "panda_hand" / "panda.urdf")

    @staticmethod
    def random_configuration():
        raise NotImplementedError(
            "Random configuration not implemented for Franka Hand"
        )
