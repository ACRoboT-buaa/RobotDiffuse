from __future__ import print_function
from robofin.pointcloud.torch import FrankaSampler

import torch
from utils import normalization
import numpy as np
from datasets.utils import shuffle_in_unison_scary
from geometrout.primitive import Sphere
from robofin.robots import FrankaRealRobot
from utils.geometry import construct_mixed_point_cloud
class datamanager(object):
    def __init__(self, datapath, num_obstacle_points=1024,random_scale=0.015,train_ratio=None, fold_k=None, norm=None, expand_dim=None, seed=233,sorted=False):
        self.seed = seed
        self.sorted=sorted
        data = np.load(datapath)
#        self.data = np.concatenate([data["x_train"], data["x_test"]])
#        self.labels = np.concatenate([data["y_train"], data["y_test"]])
        self.data = data["x_train"]
        self.labels = data["y_train"]
        self.num_robot_points=1024#1024  384
        self.class_num = 23
        self.num_obstacle_points=num_obstacle_points
        self.random_scale=random_scale
        self.num_target_points=1024#128
        self.fk_sampler = FrankaSampler(
                "cpu",
                use_cache=True,#是否使用缓存 ，因为我用的urdf文件变了
                with_base_link=False,  # 这个需要去除吗Remove base link because this isn't controllable anyway
            )
#        self.labels = one_hot_encode(self.labels, self.class_num)
        del data
        
        self.divide_train_test(train_ratio, fold_k)
        
        if norm is not None:
            self.data = self.data / norm # [0,1]
        
#        self.data = self.data[:,:,:2]
        
        self.train_cur_pos, self.test_cur_pos = 0, 0

        self.expand_dim = expand_dim

    def get_train_test_id(self, train_ratio, fold_k, seed=None):
        self.train_id, self.test_id = [], []
        # normal
        if train_ratio and not fold_k:
            for v in self.dict_by_class:
                self.train_id += v[:int(train_ratio * len(v))]
                self.test_id += v[int(train_ratio * len(v)):]
        # cross validation
        if fold_k and not train_ratio:
            for i in range(10):
                self.test_id += list(self.dict_by_class[i][self.test_fold_id])
                for j in range(fold_k):
                    if j != self.test_fold_id:
                        self.train_id += list(self.dict_by_class[i][j])
        self.train_id = np.array(self.train_id)
        self.test_id = np.array(self.test_id)

        shuffle_in_unison_scary(self.train_id, self.test_id, seed=(seed or self.seed))
        self.train_num, self.test_num = len(self.train_id), len(self.test_id)

    def divide_train_test(self, train_ratio, fold_k, seed=None):
        self.dict_by_class = [[] for i in range(self.class_num)]

        for i, key in enumerate(np.argmax(self.labels, axis=1)):
            self.dict_by_class[key].append(i)
        for i in range(self.class_num):
            np.random.seed(i)
            np.random.shuffle(self.dict_by_class[i])
        
        if fold_k and not train_ratio:
            # only for cross validation
            print ("[{} folds cross validation]".format(fold_k))
            for i in range(self.class_num):
                np.random.seed(i)
                np.random.shuffle(self.dict_by_class[i])
                self.dict_by_class[i] = np.array_split(self.dict_by_class[i], fold_k)
                np.random.seed(i)
                np.random.shuffle(self.dict_by_class[i])
            self.test_fold_id = 0
        self.get_train_test_id(train_ratio, fold_k, seed)
    
    def shuffle_train(self, seed=None):
        np.random.seed(seed)
        np.random.shuffle(self.train_id)
    
    def get_cur_pos(self, cur_pos, full_num, batch_size):
        get_pos = range(cur_pos, cur_pos + batch_size)
        if cur_pos + batch_size <= full_num:
            cur_pos += batch_size
        else:
            rest = cur_pos + batch_size - full_num
            get_pos = list(range(cur_pos, full_num)) +list(range(rest))
            cur_pos = rest
        return cur_pos, get_pos

    @staticmethod
    def normalize(configuration_tensor: torch.Tensor):
        """
        Normalizes the joints between -1 and 1 according the the joint limits

        :param configuration_tensor torch.Tensor: The input tensor. Has dim [7]
        """
        return utils.normalize_franka_joints(configuration_tensor)

    @staticmethod
    def unnormalize(configuration_tensor: torch.Tensor):
        """
        Normalizes the joints between -1 and 1 according the the joint limits

        :param configuration_tensor torch.Tensor: The input tensor. Has dim [7]
        """
        return utils.unnormalize_franka_joints(configuration_tensor)

    def __call__(self, batch_size, phase='train', maxlen=None, var_list=[]):
        if phase == 'train':
            self.train_cur_pos, get_pos = self.get_cur_pos(self.train_cur_pos, self.train_num, batch_size)
            cur_id = self.train_id[get_pos]
        elif phase == 'test':
            self.test_cur_pos, get_pos = self.get_cur_pos(self.test_cur_pos, self.test_num, batch_size)
            cur_id = self.test_id[get_pos]
        num_robot_points=self.num_robot_points
        res = {
            "data":None,
            "configuration":None,
            "obstacle_points":None,
            "all_points":None,
            "sphere_centers":None,
            "labels":None,
        }
        for i in cur_id:
            config=self.__dict__["data"][i]#(254,7)
            config_tensor = torch.as_tensor(config).float()
            config_tensor=self.normalize(config_tensor)#(254,7)
            config_tensor=config_tensor.unsqueeze(-1).permute(1,2,0)#(254,7，1)  to [njoints, nfeats, max_frames]
            if phase== 'train':
                # Add slight random noise to the joints
                randomized = (
                    self.random_scale * torch.randn(config_tensor.shape) + config_tensor
                )
                config_tensor = torch.clamp(randomized,-1,1)
            temp=config_tensor.permute(2,1,0).reshape(254,7)  #  [njoints, nfeats, max_frames] to  max_frames,njoints
            print('====')
            robot_points = self.fk_sampler.sample(self.unnormalize(temp[0,:]), self.num_robot_points) #F,7   穿的是正则化后的数据
            target_points=self.fk_sampler.sample(self.unnormalize(temp[-1,:]), self.num_target_points)
            # target_pose = FrankaRealRobot.fk(#机器人的目标笛卡尔位置
            #     temp[-1, :].tolist()
            # )
            # target_points = self.fk_sampler.sample_end_effector(#机器人目标笛卡尔位置的采样
            #     torch.as_tensor(target_pose.matrix).float(),
            #     num_points=self.num_target_points,
            # )
            if res["configuration"] is None:
                res["configuration"]=config_tensor.unsqueeze(0)#[batch_size, njoints, nfeats, max_frames]
            else:
                res["configuration"]=torch.cat((res["configuration"],config_tensor.unsqueeze(0)),0)

            laebl_tensor=torch.as_tensor(np.expand_dims(self.labels[i],0)).float()
            if res["labels"] is None:
                res["labels"]=laebl_tensor
            else:
                res["labels"]=torch.cat((res["labels"],laebl_tensor),0)
            
            # elif flag=="target_pose":
            #     res = self.__dict__[flag][cur_id]
            #     target_pose=FrankaRealRobot.fk(
            #     res[:, -1, ::]
            # )
            #     return torch.as_tensor(target_pose.xyz).float()
            #==
            obstacle_centers1= torch.FloatTensor(np.expand_dims(self.labels[i][14:17],0))#(1,3)
            obstacle_centers2=torch.FloatTensor(np.expand_dims(self.labels[i][17:20],0))#(1,3)
            obstacle_centers=torch.cat((obstacle_centers1,obstacle_centers2),0)#(2,3)
            spheres=[
                Sphere(c, 0.2)
                for c in obstacle_centers
            ]
            res["obstacles"]=spheres
            obstacle_points = torch.FloatTensor(construct_mixed_point_cloud(
                spheres, self.num_obstacle_points
            )) #[num_obstacle_points,(x,y,z,1)]
            if self.sorted:
                print(obstacle_points.shape,self.num_obstacle_points)  
                obstacle_points=obstacle_points.reshape(self.num_obstacle_points,1,1,1,1)
                obstacle_points=torch.sort(obstacle_points,1)
                obstacle_points=torch.sort(obstacle_points,2)
                obstacle_points=torch.sort(obstacle_points,3)
                obstacle_points.reshape(-1,4)
            obstacle_points = obstacle_points.unsqueeze(0)
            if res["obstacle_points"] is None:
                res["obstacle_points"]=obstacle_points
            else:
                res["obstacle_points"]=torch.cat((res["obstacle_points"],obstacle_points),0)

            all_points=torch.cat(
                (
                    torch.zeros(self.num_robot_points, 4),#机器人表面采样情况0
                    torch.ones(self.num_obstacle_points, 4),#障碍物表面采样情况1
                    2 * torch.ones(self.num_target_points, 4),#机器人末端采样情况2
                ),
                dim=0,
            )
            all_points[: self.num_robot_points, :3] = robot_points[0,:,:3].float()
            all_points[
                self.num_robot_points : self.num_robot_points
                + self.num_obstacle_points,
                :3,
            ] = torch.as_tensor(obstacle_points[0,:, :3]).float()
            all_points[
                self.num_robot_points + self.num_obstacle_points :,
                :3,
            ] = target_points[0,:,:3].float()
            if res["all_points"] is None:
                res["all_points"]=all_points.unsqueeze(0)
            else:
                res["all_points"]=torch.cat((res["all_points"],all_points.unsqueeze(0)),0)
            #==
            if res["sphere_centers"] is None:
                res["sphere_centers"]=obstacle_centers.unsqueeze(0)
            else:
                res["sphere_centers"]=torch.cat((res["sphere_centers"],obstacle_centers.unsqueeze(0)),0)
            # obstacles={}
            # obstacles["sphere_centers"]=torch.from_numpy(np.array(obstacle_centers))
            # # obstacles["sphere_radii"] =torch.from_numpy(np.array([0.2,0.2]))
            # res["obstacles"].append(obstacles)
        
        return res


# data = datamanager("data/CharacterTrajectories/CharacterTrajectories.npz",
#                    train_ratio=0.8, fold_k=None, norm=None, expand_dim=False, seed=0)
# X = data(64, var_list=['data', 'labels'])
# print X['data'].shape
# print X['labels'].shape
# seq = X['data'][3]
# print np.argmax(X['labels'][3])
#
# import matplotlib
# matplotlib.use("Agg")
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import pyplot as plt
#
#
# # fig = plt.figure()
# # ax = fig.gca(projection='3d')
# # ax.plot3D(seq[:, 0], seq[:, 1], seq[:, 2])
#
# plt.plot(seq[:,0], seq[:,1])
#
#
# plt.savefig('figs/test/tmp.png')