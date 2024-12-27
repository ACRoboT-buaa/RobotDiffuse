from __future__ import print_function
import pybullet as p
import numpy as np
import pickle
from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
     get_holding_motion_gen
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_user, disconnect, DRAKE_IIWA_URDF, user_input, update_state, disable_real_time, \
    get_movable_joints, end_effector_from_body, set_joint_positions, inverse_kinematics, pairwise_collision, get_sample_fn, \
    link_from_name, Euler, plan_joint_motion, enable_real_time, joint_controller, step_simulation, \
    refine_path, plan_multi_joint_motion
from tqdm import tqdm
import time
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
import random
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--visualize", help="choose visualize",action="store_true")
args = parser.parse_args()
DEBUG_FAILURE = False

class BodyPath(object):
    def __init__(self, body, path, joints=None, attachments=[]):
        if joints is None:
            joints = get_movable_joints(body)
        self.body = body
        self.path = path
        self.joints = joints
        self.attachments = attachments
    def bodies(self):
        return set([self.body] + [attachment.body for attachment in self.attachments])
    def iterator(self):
        for i, configuration in enumerate(self.path):
            set_joint_positions(self.body, self.joints, configuration)
            for grasp in self.attachments:
                grasp.assign()
            yield i
    def control(self, real_time=False, dt=0):
        if real_time:
            enable_real_time()
        else:
            disable_real_time()
        for values in self.path:
            for _ in joint_controller(self.body, self.joints, values):
                enable_gravity()
                if not real_time:
                    step_simulation()
                time.sleep(dt)

    def refine(self, num_steps=0):
        return self.__class__(self.body, refine_path(self.body, self.joints, self.path, num_steps), self.joints, self.attachments)
    def reverse(self):
        return self.__class__(self.body, self.path[::-1], self.joints, self.attachments)
    def __repr__(self):
        return '{}({},{},{},{})'.format(self.__class__.__name__, self.body, len(self.joints), len(self.path), len(self.attachments))

def assign_fluent_state(fluents):
    obstacles = []
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        if name == 'atpose':
            o, p = args
            obstacles.append(o)
            p.assign()
        else:
            raise ValueError(name)
    return obstacles

def get_free_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2s, fluents=[]):
        def mapconf(e):
            return e.configuration
        conf1.assign()
        obstacles = fixed + assign_fluent_state(fluents)
        conf2=list(map(mapconf, conf2s))
        j=conf2s[0].joints
        paths, _ , cost= plan_multi_joint_motion(robot,j , conf2, obstacles=obstacles,self_collisions=self_collisions,iter=1000)
        res=[]
        if paths is None:
            return None,None
        for path in paths:
            if path is not None:
                command = Command([BodyPath(robot, path, joints=conf2s[0].joints[0:7])])
                res.append(command)
            else:
                res.append(None)
        return res, cost
    return fn

def get_ik_fn(robot, fixed=[], num_attempts=10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)

    tool_link = link_from_name(robot, 'panda_link8')
    def fn(body, pose):
        obstacles = body + fixed
        for _ in range(num_attempts):
            set_joint_positions(robot, movable_joints, sample_fn())  # Random seed
            q_approach = inverse_kinematics(robot, tool_link, pose)
            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue
            conf = BodyConf(robot, q_approach)

            conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))

            return conf
        return None
    return fn

def plan(robot, block, fixed, endpoint, teleport,k):
    x0=random.uniform(-1,1)
    y0= random.uniform(-1,1)
    z0 = random.uniform(0.1, 1)
    x1 = random.uniform(-1, 1)
    y1 = random.uniform(-1, 1)
    z1 = random.uniform(0.1, 1)
    set_pose(block0, Pose(Point(x=x0, y=y0, z=z0), Euler()))
    set_pose(block1, Pose(Point(x=x1, y=y1, z=z1), Euler()))
    ik_fn2 = get_ik_fn(robot, fixed=block, num_attempts=100)
    ik_fn = get_ik_fn(robot, fixed=block, num_attempts=10000)
    free_motion_fn = get_free_motion_gen(robot, fixed=block + fixed, teleport=teleport)
    for i in range(10):
        flag=0
        x2 = random.uniform(-0.6, 0.6)
        y2 = random.uniform(-0.6, 0.6)
        z2 = random.uniform(0.2, 0.8)
        x3 = random.uniform(-0.6, 0.6)
        y3 = random.uniform(-0.6, 0.6)
        z3 = random.uniform(0.2, 0.8)
        pose1 = (Point(x=x2, y=y2, z=z2), None)
        pose2 = (Point(x=x3, y=y3, z=z3), None)
        saved_world = WorldSaver()

        saved_world.restore()
        conf0 = ik_fn2(block, pose1)
        if conf0 is None:
            continue
        conf1 = ik_fn2(block, pose2)
        if conf1 is None:
            continue
        conf1s = [ik_fn(block, pose2) for i in range(k)]
        if conf1s is None:
            continue
        for j in range(k):
            if conf1s[j] is None:
                continue
            if conf1s[j].configuration is None:
                continue
        set_pose(endpoint, Pose(Point(x=x3, y=y3, z=z3), Euler()))
        flag=1
        result2, cost = free_motion_fn(conf0, conf1s)
        break
    if flag==0 or result2 is None:
        return None,None,None,None,None
    def tocommand(e):
        if e is not None:
            return Command(e.body_paths)
        else:
            return None
    return list(map(tocommand,result2)),cost,(x0,y0,z0),(x1,y1,z1),(x3,y3,z3)


connect(use_gui=args.visualize)
disable_real_time()
robot = load_model("robofin/urdf/robots/panda_arm.urdf", fixed_base=True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
floor = load_model('models/short_floor.urdf', fixed_base=True)
block0 = load_model("models/medium_sphere.urdf", fixed_base=True)
block1 = load_model("models/medium_sphere.urdf", fixed_base=True)
endpoint = load_model("models/marker.urdf", fixed_base=True)
set_pose(block0, Pose(Point(x=0.5,y=0.3, z=0.6),Euler()))
set_pose(block1, Pose(Point(x=-0.1,y=-0.1, z=0.8),Euler()))
set_pose(endpoint, Pose(Point(x=0.5, y=0.3, z=0.8), Euler()))
set_default_camera()
dump_world()
k=10

saved_world = WorldSaver()
## 比原版正确
for epoch in tqdm(range(0,100)):
    t=0
    save=[]
    while t<1000:
        command, cost, b0, b1, p = plan(robot, [block0,block1],[floor],endpoint, teleport=False, k=k)

        max=999999
        pos=-1
        if command is None:
            continue
        for i,e in enumerate(command):
            if e is not None:
                if cost[i]<max:
                    max=cost[i]
                    pos=i
        if pos==-1:
            continue
        t+=1

        res=np.array(command[pos].body_paths[0].path)
        save.append([res,b0,b1,p])
    filename = '{}.dat'.format(epoch)
    outfile = open(filename,'w+b')
    pickle.dump(save,outfile)
    outfile.close()
    #saved_world.restore()





disconnect()
