"""
Note: This is a updated version from my previous code,
for the target network, I use moving average to soft replace target parameters instead using assign function.
By doing this, it has 20% speed up on my machine (CPU).

Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
Pendulum example.

View more on my tutorial page: https://morvanzhou.github.io/tutorials/

Using:
tensorflow 1.0
gym 0.8.0
"""

import tensorflow as tf
import numpy as np
import time
import _thread
import pandas as pd


class DDPG(object):
    def __init__(self):
        # 为了在一个类中实现，构造函数不能再传参了，参数全部在run()函数中修改

        # 自己加的参数
        self.admittance = 10.0
        self.is_episode_done = False
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0
        self.episode_number = 0
        self.episode_reward = 0.0
        self.cpp_online = False
        self.online = False
        self.velocity_expect = 0.55  # 目标速度
        self.is_run_done = 0
        self.step_number = 0

        self.MAX_EPISODES = 15  # 总训练次数
        self.MAX_EP_STEPS = 120  # 单次训练最长步数  （*0.03）
        self.LR_A = 0.001  # learning rate for actor
        self.LR_C = 0.002  # learning rate for critic
        self.GAMMA = 0.9  # reward discount
        self.TAU = 0.01  # soft replacement
        self.MEMORY_CAPACITY = 10000
        self.BATCH_SIZE = 32

        self.a_dim = 1  # action的范围是20
        self.s_dim = 2
        self.a_bound = 6.0
        self.memory = np.zeros((self.MEMORY_CAPACITY, self.s_dim * 2 + self.a_dim + 1), dtype=np.float32)
        self.pointer = 0
        self.sess = tf.Session()

        self.S = tf.placeholder(tf.float32, [None, self.s_dim], 's')
        self.S_ = tf.placeholder(tf.float32, [None, self.s_dim], 's_')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')
        self.a = self._build_a(self.S)
        q = self._build_c(self.S, self.a, )
        a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Actor')
        c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Critic')
        ema = tf.train.ExponentialMovingAverage(decay=1 - self.TAU)  # soft replacement

        def ema_getter(getter, name, *args, **kwargs):
            return ema.average(getter(name, *args, **kwargs))

        target_update = [ema.apply(a_params), ema.apply(c_params)]  # soft update operation
        a_ = self._build_a(self.S_, reuse=True, custom_getter=ema_getter)  # replaced target parameters
        q_ = self._build_c(self.S_, a_, reuse=True, custom_getter=ema_getter)

        a_loss = - tf.reduce_mean(q)  # maximize the q
        self.atrain = tf.train.AdamOptimizer(self.LR_A).minimize(a_loss, var_list=a_params)

        with tf.control_dependencies(target_update):  # soft replacement happened at here
            q_target = self.R + self.GAMMA * q_
            td_error = tf.losses.mean_squared_error(labels=q_target, predictions=q)
            self.ctrain = tf.train.AdamOptimizer(self.LR_C).minimize(td_error, var_list=c_params)

        self.sess.run(tf.global_variables_initializer())

    def choose_action(self, s):
        return self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]

    def learn(self):
        indices = np.random.choice(self.MEMORY_CAPACITY, size=self.BATCH_SIZE)
        bt = self.memory[indices, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]
        bs_ = bt[:, -self.s_dim:]

        self.sess.run(self.atrain, {self.S: bs})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % self.MEMORY_CAPACITY  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.pointer += 1

    def _build_a(self, s, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Actor', reuse=reuse, custom_getter=custom_getter):
            net = tf.layers.dense(s, 30, activation=tf.nn.relu, name='l1', trainable=trainable)
            a = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            return tf.multiply(a, self.a_bound, name='scaled_a')

    def _build_c(self, s, a, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Critic', reuse=reuse, custom_getter=custom_getter):
            n_l1 = 30
            w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            net = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            return tf.layers.dense(net, 1, trainable=trainable)  # Q(s,a)

    def step(self, velocity, acceleration, jerk):
        """
        更新step参数
        :param velocity: 状态：速度
        :param acceleration: 状态：加速度
        :param jerk: 奖励：加加速度
        :return: 动作和 episode结束标志：返回string，第一位是0/1，标志episode是否结束，其余表示导纳值
        """
        step_time = time.time()
        self.velocity = velocity
        self.acceleration = acceleration
        self.jerk = jerk
        self.cpp_online = True
        #        time.sleep(2)
        #        print("我把状态更新等了2s")
        #        print("v,", self.velocity, "a,", self.acceleration, "B:" ,self.admittance ,"jerk,", self.jerk)
        #        print("in step selfonline before:", self.cpp_online)
        while self.online:
            time.sleep(0.001)
        #            time.sleep(0.1)
        #             print("step睡了0.1，因为动作选择还没结束")
        #        print("in step selfonline after:", self.cpp_online)
        #        print("本次step要输出的动作是", self.admittance)

        if self.is_episode_done:
            #            print("B--," , -self.admittance)
            self.admittance = -self.admittance

#        print("python step time:", time.time() * 1000 - step_time * 1000)
        return self.admittance

    def run(self):
        _thread.start_new_thread(self.run_inner, ())

    def run_inner(self):
        """
        开始训练
        :return:
        """
        self.is_run_done = 0
        # 这里需要先获得状态，取代
        var = 1  # control exploration
        t1 = time.time()
        #        print("run 开始")
        for i in range(self.MAX_EPISODES):
            # TODO c++中每个 episode 自动reset
            self.cpp_online = False
            #            print("in run selfonline:", self.cpp_online)
            while not self.cpp_online:
                time.sleep(0.001)
            #                time.sleep(0.5)
            #                 print("run等待第一次无用的状态更新0.5s")
            # s = np.array([0, 0])
            ep_reward = 0
            step_number = 1
            self.is_episode_done = False
            last_s = np.array([0.0, 0.0])
            last_a = 0.0
            self.admittance = 10.0
            self.step_number = 0
            self.episode_number += 1
            #            print("---------------------------------一次episode开始----------------------------------------------------", self.episode_number)
            my_time = time.time()
            while not self.is_episode_done:
#                print("step real time",time.time() - my_time)
                my_time = time.time()
                self.cpp_online = False
                #                print("-------------------走进了一次step--------------------------", step_number)
                while not self.cpp_online:
                    time.sleep(0.001)
                #                    time.sleep(0.5)
                #                     print("run等待状态更新0.5s")

                self.online = True
                action_select_time = time.time() * 1000
                step_number += 1
                #                print("step number = ", step_number)
                s = np.array([self.velocity, self.acceleration])
                a = self.choose_action(s)
                #                print("网络里的a:", a)

                # 测试用 todo
                #                time.sleep(2)

                # Add exploration noise
                # 神经网络中训练的是-20~20，这里传输给c++的时候加上25就变成了5~45
                a = np.clip(np.random.normal(a, var) + 10.0, 4.0, 16.0)
                #                print("加上偏置和噪声的a:", a)

                # add randomness to action selection for exploration
                #                while (last_a == a).all() and (last_s == s).all():
                #                    a = self.choose_action(s)
                #                    a = np.clip(np.random.normal(a, var) + 25, 5, 45)

                self.admittance = a
                #                print("将self的导纳值赋值为神经网络的a")
                self.online = False
#                print("action select time:",time.time() * 1000 - action_select_time)

                last_a = a
                last_s = s
                r = -self.jerk

                self.store_transition(s, a, r / 10, s)

                if self.pointer > self.MEMORY_CAPACITY:
                    var *= .9995  # decay the action randomness
                    self.learn()

                ep_reward += r
                # 下面是episode结束的条件
                #                print("episode结束？？", self.is_episode_done)
                if (abs((self.velocity - self.velocity_expect)/self.velocity_expect) <= 0.05 and abs(self.acceleration/self.velocity_expect) <= 0.1):
                    # if abs(self.velocity - self.velocity_expect) <= 0.05:
                    #                    print("velocity", self.velocity)
                    #                    print("velocity_expect", self.velocity_expect)
                    self.episode_reward = ep_reward
                    self.is_episode_done = True
                    #                    print("episode done 置为True")
                    print("----------------------11111一次episode结束因为速度符合条件----------------------------------")

                elif step_number >= self.MAX_EP_STEPS:
                    #                    print("step_number", step_number)
                    #                    print("MAX_EP_STEPS", self.MAX_EP_STEPS)
                    self.episode_reward = ep_reward
                    self.is_episode_done = True
                    print("----------------------22222一次episode结束因为step 100次----------------------------------")

        # print('Running time: ', time.time() - t1)
        # self.is_run_done = 1
        # return self.is_run_done

    def show_episode_result(self):
        """
        每个episode结束时，可以调用此函数显示信息
        :return: 本次 episode 信息
        """
        res = "episode" + str(self.episode_number) + "reward:" + str(self.episode_reward)
        return res

    def set_velocity_expect(self, velocity):
        self.velocity_expect = velocity

    def setter_online(self, a):
        self.online = a

    @staticmethod
    def test_fun():
        return 100
