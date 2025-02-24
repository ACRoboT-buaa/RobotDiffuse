# RobotDiffuse: Motion Planning for Redundant Manipulator based on Diffusion Model
This repository provides the implementation of the _RobotDiffuse: Motion Planning for Redundant Manipulator based on Diffusion Model_ method, called _RobotDiffuse_ below. 

## Abstract
> Redundant manipulators, with their higher Degrees of Freedom (DOFs), offer enhanced kinematic performance and versatility, making them suitable for applications like manufacturing, surgical robotics, and human-robot collaboration. However, motion planning for these manipulators is challenging due to increased DOFs and complex, dynamic environments. While traditional motion planning algorithms struggle with high-dimensional spaces, deep learning-based methods often face instability and inefficiency in complex tasks.
> This paper introduces RobotDiffuse, a diffusion model-based approach for motion planning in redundant manipulators. By integrating physical constraints with a point cloud encoder and replacing the U-Net structure with an encoder-only transformer, RobotDiffuse improves the model's ability to capture temporal dependencies and generate smoother, more coherent motion plans. We validate the approach using a complex simulator, and release a new dataset with 35M robot poses and 0.14M obstacle avoidance scenarios. Experimental results demonstrate the effectiveness of RobotDiffuse and the promise of diffusion models for motion planning tasks.



## Citation
Link to our paper [here](https://arxiv.org/abs/2412.19500).
If you use this code for your research, please cite our paper:

```
@article{zhang2024robotdiffuse,
  title={RobotDiffuse: Motion Planning for Redundant Manipulator based on Diffusion Model},
  author={Zhang, Xiaohan and Mou, Xudong and Wang, Rui and Wo, Tianyu and Gu, Ningbo and Wang, Tiejun and Xu, Cangbai and Liu, Xudong},
  journal={arXiv preprint arXiv:2412.19500},
  year={2024}
}
```

## Installation
This code is based on `Python 3.7`, all requirements are written in `requirements.txt`. 

```
pip install -r requirements.txt
```

## Dataset
To verify our approach in more complex obstacle avoidance scenarios, we specifically made a dataset pipeline and produced a dataset Robot-obtalcles-panda (ROP) using Pybullet.
This repository also includes ROP's data loading code and data generation code.

### Data generation
If you want to generate the data yourself, we provide scripts for generating datasets which are designed to maintain an even distribution of data in the dataset. To ensure uniformity, you will first generate data for each individual problem type and then merge them together.

You can use `datagen.py` to generate data for a single type of environment and between two task-oriented poses .  

```bash
cd datasets\data_pipeline
python3 datagen.py
```

To visualize how the trajectories all start or end with a collision-free robot pose, you can use

```bash
cd datasets\data_pipeline
python3 datagen.py --visualize
```

### Data Cleaning

After generating the data, you will need to clean it and merge it into a single dataset.

```bash
cd datasets\data_pipeline
python3 datamerge.py
```

We provide a script, `datamerge.py`, that takes the output of the `datagen.py` and cleans it up. When you run `datamerge.py` in full pipe mode, it will generate a file named `robot.npz` in the specified directory. This is all the data for a multi-process run, and it has data in a single environment class where each trajectory starts or ends in a collision-free pose.

### Data Loading

After merging the data, you will need to load it and train the model. We provide a script, `datamanager.py`, that processes the content of `robot.npz`, splits the training set and the validation set, and returns the processing content

```python
from datamanager import datamanager
dm = datamanager(
    os.path.join(os.getcwd(),"datasets","robot.npz"),
    train_ratio=0.9,
    random_scale=0, 
    fold_k=None, 
    norm=None, 
    expand_dim=3, 
    seed=0,
    num_obstacle_points=1400)
#loading training set
data = dm(args.num_samples)
#loading validation set
#data = dm(args.num_samples,phase="test")
configuration = data["configuration"].to(dist_util.dev())
sphere_centers =data["sphere_centers"].to(dist_util.dev())
obstacle_points=data["obstacle_points"].to(dist_util.dev())
labels=torch.tensor(data["labels"]).float().to(dist_util.dev())
all_points=torch.tensor(data["all_points"]).float().to(dist_util.dev())
obstacles=data["obstacles"]
```

## Visualize

For the display of motion planning results, this paper provides a more lightweight visualization scheme `visualize\index.html`, which is based on the Web to set the information of obstacle avoidance tasks and visualize the robot's motion process, which can provide users with convenient and efficient interaction capabilities and can get started at a low cost.

For obstacles, it sets the corresponding size of blue geometry to render and uses green geometry to wrap to represent the safe distance.

H5 performs animation display by reading local generated files.

## Repository Structure

### `dataset`
datasets Code and for the data generation pipeline.

### `visualize`
Directory where the visualize code on h5 is saved.

### `Utils`

Directory where the utils is saved.

## Baselines
RRT*, MPNet, MPINet, MPD

We reiterate that in addition to our method, the source code of other baselines is based on the GitHub source code 
provided by their papers. For reproducibility, we changed the source code of their models as little as possible. 
We are grateful for the work on these papers.

We consult the GitHub source code of the paper corresponding to the baseline and then reproduce it. 
For baselines , we use their own recommended hyperparameters. 

### Acknowledgements
Part of the code, especially the baseline code and ablation code, is based on the following source code.
1. [MPNet](https://github.com/ahq1993/MPNet)

2. [MPINet](https://github.com/nvlabs/motion-policy-networks)

3. [motion-diffusion-model](https://github.com/guytevet/motion-diffusion-model)

4. [robofin](https://github.com/fishbotics/robofin)

5. [ikfast_pybind](https://github.com/yijiangh/ikfast_pybind)

6. [DGCNN](https://github.com/WangYueFt/dgcnn/tree/master/pytorch)

7. [DDPM]([hojonathanho/diffusion: Denoising Diffusion Probabilistic Models](https://github.com/hojonathanho/diffusion))

8. [PointNet]([charlesq34/pointnet: PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation](https://github.com/charlesq34/pointnet))

![output3](https://github.com/user-attachments/assets/76cf5875-3436-4eaa-92aa-69201c18ba3c)

![output4](https://github.com/user-attachments/assets/91f26298-6e19-4a8c-b1c7-d803571eba87)

![output8](https://github.com/user-attachments/assets/46924c8e-d223-4588-998a-323be7767327)

![output15](https://github.com/user-attachments/assets/63ae1a2d-668d-4366-b31d-a90e345e08a0)

![output19](https://github.com/user-attachments/assets/b1d4b791-81ca-4305-8a2b-7bb42a2c8762)


   
