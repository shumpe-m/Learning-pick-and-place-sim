import os
from pyrep import PyRep
from pyrep.robots.arms.arm import Arm
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.const import PrimitiveShape
from pyrep.objects.vision_sensor import VisionSensor
import numpy as np
import seaborn as sns
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import quaternion

import cv2


class PyrepEnv:
    def __init__(self, scene_file: str, headless=True):
        self.pr = PyRep()
        self.pr.launch(scene_file, headless)
        self.dim_action, self.dim_state = None, None
        # self.pr.start()

    def _get_state(self):
        raise NotImplementedError()

    def reset(self):
        raise NotImplementedError()

    def step(self, action):
        raise NotImplementedError()

    def shutdown(self):
        self.pr.stop()
        self.pr.shutdown()


class Pick_Place_Env(PyrepEnv):
    def __init__(self, scene_file: str, headless: bool):
        super(Pick_Place_Env, self).__init__(scene_file, headless)
        self.ee_target = Dummy('/Panda/Panda_target') # グリッパーの位置
        self.box1_target = Dummy('/Panda/Box1_target') # 固定 visionを取るための位置: pick 
        self.box1_upper = np.array(self.box1_target.get_pose())
        self.box2_target = Dummy('/Panda/Box2_target') # 固定 visionを取るための位置: place
        self.box2_upper = np.array(self.box2_target.get_pose())
        self.gripper_ctrl = Dummy('/Panda/OpenCloseControl')
        self.box_list = [Shape(f'Box{i}') for i in range(1,3)]
        self.box_size = np.array([0.281, 0.172, 0.075]) # scale x,y,z
        self.box_pos = np.array([[0.3, 0.15, 0.09], [0.3, -0.15, 0.09]]) # position x,y,z [Box1][Box2]
        #self.dish = Shape('Dish')
        self.vision_sensor = VisionSensor('/Panda/Panda_joint1/Panda_link1_respondable/Panda_joint2/Panda_link2_respondable/Panda_joint3/Panda_link3_respondable/Panda_joint4/Panda_link4_respondable/Panda_joint5/Panda_link5_respondable/Panda_joint6/Panda_link6_respondable/Panda_joint7/Panda_link7_respondable/Panda_attachment/Panda_gripper/Vision_sensor')
        #self.obj_length = 0.05
        self.dim_action, self.dim_state = 4, 3

        self.pr.start()
        #self.box_position = np.array([box.get_position() for box in self.box_list])
        self.num_arrange = 0
        # x,yアクションスペース
        self.min_action_space = self.box_pos[:, 0:2] - self.box_size[0:2] / 2
        self.max_action_space = self.box_pos[:, 0:2] + self.box_size[0:2] / 2
        self.bias = self.box_size[0:2]
        self.z_offset = 0.06 # boxの底の相対位置
        
        
        #self.common_pose = self.box_position[0]
        #self.common_pose[1] = (self.box_position[0, 1] - self.box_position[1, 1])/2

    def __open_gripper(self):
        self.gripper_ctrl.set_pose([0, 0, 1, 0, 0, 0, 1])
        [self.pr.step() for _ in range(40)]

    def __close_gripper(self):
        self.gripper_ctrl.set_pose([0, 0, -1, 0, 0, 0, 1])
        [self.pr.step() for _ in range(40)]

    def _get_state(self):
        return np.array([o.get_position() for o in self.obj_list])
    
    def _get_depth(self):
        [self.pr.step() for _ in range(10)]
        return self.vision_sensor.capture_depth()
    
    def _gripper_pos(self, pos_type = "", pick_place_pose = None):
        # target_name:ダミーオブジェクト名 
        if pos_type == "box1":
            self.ee_target.set_pose(self.box1_upper)
        elif pos_type == "box2":
            self.ee_target.set_pose(self.box2_upper)
        else:
            self.ee_target.set_pose(pick_place_pose)

        [self.pr.step() for _ in range(40)]
    
    def _pick(self, pose, z_arg):
        quat = np.roll(quaternion.as_float_array(quaternion.from_rotation_vector([0, 0, z_arg])), -1)
        # 物体の掴むべき点の直上に移動
        #ee_target_pose = self.box1_upper.copy()
        target_x = pose[0] + self.min_action_space[1, 0]
        target_y = pose[1] + self.min_action_space[1, 1]
        target_z = pose[2] + self.z_offset
        self._gripper_pos(pick_place_pose = np.concatenate([[target_x, target_y, 0.3], self.box2_upper[3:]]))
        self._gripper_pos(pick_place_pose = np.concatenate([[target_x, target_y, 0.3], quat]))
        
        self._gripper_pos(pick_place_pose = np.concatenate([[target_x, target_y, target_z], quat]))

        self.__close_gripper()

        self._gripper_pos(pick_place_pose = np.concatenate([[target_x, target_y, 0.3], quat]))
        self._gripper_pos(pick_place_pose = np.concatenate([[target_x, target_y, 0.3], self.box2_upper[3:]]))
        
        self._gripper_pos(pos_type = 'box2')

    def _place(self, pose, z_arg):
        return 0
        # go to above the dish with object
        quat = np.roll(quaternion.as_float_array(quaternion.from_rotation_vector([0, 0, pose[2]])), -1)
        target_x = pose[0] * np.cos(pose[1]) + self.dish_position[0]
        target_y = pose[0] * np.sin(pose[1]) + self.dish_position[1]

        self.ee_target.set_pose(np.concatenate([[target_x, target_y, 0.25], quat]))
        [self.pr.step() for _ in range(30)]

        self.ee_target.set_pose(np.concatenate([[target_x, target_y, self.target_z], quat]))
        [self.pr.step() for _ in range(30)]

        self.__open_gripper()
        [self.pr.step() for _ in range(10)]

        self.ee_target.set_pose(np.concatenate([[target_x, target_y, 0.25], quat]))
        [self.pr.step() for _ in range(10)]
        
    def _create_object(self, pos = None, ori = None, color = [0.1, 1, 0.1], static = False, num = '', box = '1'):
        box_num = 0 if box == "1" else 1
        pos[0:2] = pos[0:2] * (self.bias - 0.1) + self.min_action_space[box_num, 0:2] + 0.1
        pos[2] = pos[2] * 0.13 + self.z_offset
        obj = Shape.create(type = PrimitiveShape.CUBOID,
                                       mass = 0.01,
                                       size = [0.02, 0.02, 0.035],
                                       color = color,
                                       static = static, respondable=True)
        obj.set_name('Obj' + str(num))
        obj.set_bullet_friction(1.)
        obj.set_position(pos)
        obj.set_orientation(ori)

    def reset(self):
        self.num_arrange = 0
        self.pr.stop()
        self.pr.start()
        self.num_obj = np.random.randint(1, 5) # randint()の２つ目の引数でオブジェクト数の最大値変更
        # 初期配置 複数可 後々、Box2に関係ないobjを追加
        for i in range(self.num_obj):
            self.goal_pose = np.random.rand(3)
            self.goal_ori = np.random.rand(3)
            self.initial_pose = np.random.rand(3)
            self.initial_ori = np.random.rand(3)
            self._create_object(pos = self.goal_pose,
                                ori = self.goal_ori,
                                color = [0.1, 1, 0.1],
                                static = False,
                                num = i,
                                box = '1')
            self._create_object(pos = self.initial_pose,
                                ori = self.initial_ori,
                                color = [0.1, 1, 0.1],
                                static = False,
                                num = self.num_obj + i,
                                box = '2')
        # ゴールstateを取得
        self._gripper_pos(pos_type = "box1")
        goal_state = self._get_depth()
        # Box1のオブジェクトを消去
        for i in range(self.num_obj):
            Object.remove(Shape('Obj' + str(i)))
        # 初期state取得
        self._gripper_pos(pos_type = "box2")
        state = self._get_depth()
        return state, goal_state, self.num_obj

    def reward_function(self, state):
        reward = 0
        
        return reward

    def __norm_to_unnorm(self, action):
        # 次の値を使う場合、各Boxのmin_action_spaceを足す必要あり
        place_x = self.bias[0] * action[0]
        place_y = self.bias[1] * action[1]
        place_z = self.box_size[2] * action[2]
        z_arg = -action[3] * np.pi / 4
        return np.array([place_x, place_y, place_z]), z_arg

    def step(self, pick_norm_action):
        # アクション
        # 正規化された行動から環境用の行動に
        unnrom_action, z_arg = self.__norm_to_unnorm(pick_norm_action)
        # オブジェクトの把持
        self._pick(unnrom_action, z_arg)
        # オブジェクトの配置
        result_state = self._place(unnrom_action, z_arg)
        reward = self.reward_function(result_state)
        # Box2のstateを取得
        self._gripper_pos("box2")
        next_state = self._get_depth()
        return reward, next_state


class Agent:
    def __init__(self, dim_action: int):
        self.dim_action = dim_action

    def act(self, state):
        raise NotImplementedError()

    def learn(self, replay_buffer):
        raise NotImplementedError()


class RandomAgent(Agent):
    def __init__(self, dim_action: int):
        super(RandomAgent, self).__init__(dim_action)

    def act(self, state):
        norm_action = np.random.rand(self.dim_action)
        return norm_action

    def learn(self, replay_buffer):
        # do something
        pass
    
class Get_image:
    def __init__(self):
        #self.depth_min = 0.22
        #self.depth_max = 0.44
        self.depth_min = 0.35
        self.depth_max = 0.40
        now = time.ctime()
        cnvtime = time.strptime(now)
        self.dir = 'data/images/' + str(time.strftime("%Y_%m_%d_%H_%M", cnvtime))
        os.mkdir(self.dir)

    def image_save(self, state, episode = 0, step = 0):
        # 正規化
        state = (state - self.depth_min) / (self.depth_max - self.depth_min) * 255
        #plt.figure()
        #plt.axis("off")
        #plt.imshow(state, cmap = 'gray', vmin = 0, vmax = 1)
        #sns.heatmap(state)
        path = str(self.dir) + '/' + str(episode) + '_' + str(step) + '.png'
        #plt.savefig(path)
        #plt.close('all')
        
        cv2.imwrite(path, state)
        
        


if __name__ == '__main__':
    scene = 'assets/test_env_v3.ttt'
    abs_scene = osp.join(osp.dirname(osp.abspath(__file__)), scene)

    env = Pick_Place_Env(scene_file=abs_scene, headless=False)
    agent = RandomAgent(env.dim_action)
    
    image = Get_image()

    num_episodes = 3
    replay_buffer = []

    for e in range(num_episodes):
        state, goal_state, step = env.reset()
        image.image_save(state = goal_state, episode = e)
        print(state.shape)
        for i in range(step):
            image.image_save(state = state, episode = e, step = i+1)
            action = agent.act(state)
            reward, next_state = env.step(action)
            print(i, reward)
            replay_buffer.append((state, action, reward, next_state))
            state = next_state
            agent.learn(replay_buffer)

    env.shutdown()
