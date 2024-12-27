"""
Not sure where to put this function yet, but need it quickly so implementing it here
Sphere model comes from STORM: https://github.com/NVlabs/storm/blob/e53556b64ca532e836f6bfd50893967f8224980e/content/configs/robot/franka_real_robot.yml
"""
from urchin import URDF
from robofin.robots import FrankaRobot
from robofin.pointcloud.numpy import transform_pointcloud
from geometrout.primitive import Sphere
import logging
import numpy as np

SELF_COLLISION_SPHERES = [
    ("panda_link0", [-0.08, 0.0, 0.05], 0.06),
    ("panda_link0", [-0.0, 0.0, 0.05], 0.08),
    ("panda_link1", [0.0, -0.08, 0.0], 0.1),
    ("panda_link1", [0.0, -0.03, 0.0], 0.1),
    ("panda_link1", [0.0, 0.0, -0.12], 0.06),
    ("panda_link1", [0.0, 0.0, -0.17], 0.06),
    ("panda_link2", [0.0, 0.0, 0.03], 0.1),
    ("panda_link2", [0.0, 0.0, 0.08], 0.1),
    ("panda_link2", [0.0, -0.12, 0.0], 0.06),
    ("panda_link2", [0.0, -0.17, 0.0], 0.06),
    ("panda_link3", [0.0, 0.0, -0.06], 0.05),
    ("panda_link3", [0.0, 0.0, -0.1], 0.06),
    ("panda_link3", [0.08, 0.06, 0.0], 0.055),
    ("panda_link3", [0.08, 0.02, 0.0], 0.055),
    ("panda_link4", [0.0, 0.0, 0.02], 0.055),
    ("panda_link4", [0.0, 0.0, 0.06], 0.055),
    ("panda_link4", [-0.08, 0.095, 0.0], 0.06),
    ("panda_link4", [-0.08, 0.06, 0.0], 0.055),
    ("panda_link5", [0.0, 0.055, 0.0], 0.05),
    ("panda_link5", [0.0, 0.085, 0.0], 0.055),
    ("panda_link5", [0.0, 0.0, -0.22], 0.05),
    ("panda_link5", [0.0, 0.05, -0.18], 0.045),
    ("panda_link5", [0.015, 0.08, -0.14], 0.03),
    ("panda_link5", [0.015, 0.085, -0.11], 0.03),
    ("panda_link5", [0.015, 0.09, -0.08], 0.03),
    ("panda_link5", [0.015, 0.095, -0.05], 0.03),
    ("panda_link5", [-0.015, 0.08, -0.14], 0.03),
    ("panda_link5", [-0.015, 0.085, -0.11], 0.03),
    ("panda_link5", [-0.015, 0.09, -0.08], 0.03),
    ("panda_link5", [-0.015, 0.095, -0.05], 0.03),
    ("panda_link6", [0.0, 0.0, 0.0], 0.05),
    ("panda_link6", [0.08, 0.035, 0.0], 0.052),
    ("panda_link6", [0.08, -0.01, 0.0], 0.05),
    ("panda_link7", [0.0, 0.0, 0.07], 0.05),
    ("panda_link7", [0.02, 0.04, 0.08], 0.025),
    ("panda_link7", [0.04, 0.02, 0.08], 0.025),
    ("panda_link7", [0.04, 0.06, 0.085], 0.02),
    ("panda_link7", [0.06, 0.04, 0.085], 0.02),
    # ("panda_leftfinger", [0.0, 0.01, 0.034], 0.02),
    # ("panda_rightfinger", [0.0, -0.01, 0.034], 0.02),
]


class FrankaSelfCollisionChecker:
    def __init__(
        self,
        default_prismatic_value=0.025,
    ):
        logging.getLogger("trimesh").setLevel("ERROR")

        self.default_prismatic_value = default_prismatic_value
        self.robot = URDF.load(FrankaRobot.urdf, lazy_load_meshes=True)
        # Set up the center points for calculating the FK position
        link_names = []
        centers = {}
        for s in SELF_COLLISION_SPHERES:
            if s[0] not in centers:
                link_names.append(s[0])
                centers[s[0]] = [s[1]]
            else:
                centers[s[0]].append(s[1])
        self.points = [(name, np.asarray(centers[name])) for name in link_names]

        self.collision_matrix = -np.inf * np.ones(
            (len(SELF_COLLISION_SPHERES), len(SELF_COLLISION_SPHERES))
        )

        link_ids = {link_name: idx for idx, link_name in enumerate(link_names)}
        # Set up the self collision distance matrix
        for idx1, (link_name1, center1, radius1) in enumerate(SELF_COLLISION_SPHERES):
            for idx2, (link_name2, center2, radius2) in enumerate(
                SELF_COLLISION_SPHERES
            ):
                # Ignore all sphere pairs on the same link or adjacent links
                if abs(link_ids[link_name1] - link_ids[link_name2]) < 2:
                    continue
                self.collision_matrix[idx1, idx2] = radius1 + radius2

    def spheres(self, config):
        cfg = np.ones(8)
        cfg[:7] = config
        cfg[-1] = self.default_prismatic_value
        fk = self.robot.link_fk(cfg, use_names=True)
        spheres = []
        for link_name, center, radius in SELF_COLLISION_SPHERES:
            spheres.append(Sphere((fk[link_name] @ np.array([*center, 1]))[:3], radius))
        return spheres

    def has_self_collision(self, config):
        # Cfg should have 8 dof because the two fingers mirror each other in
        # this urdf
        cfg = np.ones(7)
        cfg[:7] = config
        fk = self.robot.link_fk(cfg, use_names=True)
        fk_points = []
        # TODO this is where you left off
        for link_name, centers in self.points:
            pc = transform_pointcloud(centers, fk[link_name], in_place=False)
            fk_points.append(pc)
        transformed_centers = np.concatenate(fk_points, axis=0)
        points_matrix = np.tile(
            transformed_centers, (transformed_centers.shape[0], 1, 1)
        )
        distances = np.linalg.norm(
            points_matrix - points_matrix.transpose((1, 0, 2)), axis=2
        )
        return np.any(distances < self.collision_matrix)


class FrankaSelfCollisionSampler(FrankaSelfCollisionChecker):
    def __init__(self, default_prismatic_value=0.025):
        super().__init__(default_prismatic_value)
        self.link_points = {}
        total_points = 10000
        surface_scalar_sum = sum(
            [radius ** 2 for (_, _, radius) in SELF_COLLISION_SPHERES]
        )
        surface_scalar = total_points / surface_scalar_sum

        for idx1, (link_name, center, radius) in enumerate(SELF_COLLISION_SPHERES):
            sphere = Sphere(center, radius)
            if link_name in self.link_points:
                self.link_points[link_name] = np.concatenate(
                    (
                        self.link_points[link_name],
                        sphere.sample_surface(int(surface_scalar * radius ** 2)),
                    ),
                    axis=0,
                )
            else:
                self.link_points[link_name] = sphere.sample_surface(
                    int(surface_scalar * radius ** 2)
                )

    def sample(self, config, n):
        cfg = np.ones(8)
        cfg[:7] = config
        cfg[-1] = self.default_prismatic_value
        fk = self.robot.link_fk(cfg, use_names=True)
        pointcloud = []
        for link_name, centers in self.points:
            pc = transform_pointcloud(
                self.link_points[link_name], fk[link_name], in_place=False
            )
            pointcloud.append(pc)
        pointcloud = np.concatenate(pointcloud, axis=0)
        mask = np.random.choice(np.arange(len(pointcloud)), n, replace=False)
        return pointcloud[mask]
