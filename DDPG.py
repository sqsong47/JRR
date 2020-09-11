import tensorflow as tf
import numpy as np
import time
import _thread


class DDPG(object):
    def __init__(self):
        self.admittance = 10.0
        self.is_episode_done = False
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0
        self.episode_number = 0
        self.episode_reward = 0.0
        self.cpp_online = False
        self.python_online = False
        self.velocity_expect = 0.30  # target velocity
        self.is_run_done = 0
        self.step_number = 0

        # parameter adjustable
        self.velocity_normalization = 1 / self.velocity_expect
        self.acceleration_normalization = 1 / 5
        self.jerk_normalization = 1 / 300
        self.MAX_EPISODES = 30  # max times of episodes
        self.MAX_EP_STEPS = 50  # max number of steps per episode  （*0.05）
        self.a_bound = 7.5      # range of action

        self.LR_A = 0.001  # learning rate for actor
        self.LR_C = 0.002  # learning rate for critic
        self.GAMMA = 0.9  # reward discount
        self.TAU = 0.01  # soft replacement
        self.MEMORY_CAPACITY = 1000
        self.BATCH_SIZE = 2

        self.a_dim = 1
        self.s_dim = 2
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
        self.velocity = velocity * self.velocity_normalization
        self.acceleration = acceleration * self.acceleration_normalization
        self.jerk = jerk * self.jerk_normalization
        self.cpp_online = True
        while self.python_online:
            time.sleep(0.001)

        if self.is_episode_done:
            self.admittance = -abs(self.episode_reward)
        return self.admittance

    def run(self):
        _thread.start_new_thread(self.run_inner, ())

    def run_inner(self):
        self.is_run_done = 0
        var = 1  # control exploration
        for i in range(self.MAX_EPISODES):
            self.cpp_online = False
            # waiting for step() called to wait for robot back to zero
            while not self.cpp_online:
                time.sleep(0.001)
            # reset parameters
            step_number = 1
            self.episode_reward = 0.0
            self.is_episode_done = False
            self.admittance = 15
            self.step_number = 0

            self.episode_number += 1
            # circulation of steps per episode
            while not self.is_episode_done:
                self.cpp_online = False
                while not self.cpp_online:
                    time.sleep(0.001)

                self.python_online = True
                step_number += 1
                s = np.array([self.velocity, self.acceleration])
                if self.velocity == 404:
                    print("3333333 episode finished because robot close to the limits")
                    self.velocity = 0
                    step_number = 50

                a = self.choose_action(s)

                # 神经网络中训练的是-20~20，这里传输给c++的时候加上25就变成了5~45,同时增加一个方差为1的噪声
                a = np.clip(np.random.normal(a, var) + 17.5, 10, 25)

                self.admittance = a
                self.python_online = False
                r = -self.jerk
                self.store_transition(s, a, r / 10, s)

                if self.pointer > self.MEMORY_CAPACITY:
                    var *= .9995  # decay the action randomness
                    self.learn()

                self.episode_reward += r
                # 下面是episode结束的条件
                if abs((self.velocity / self.velocity_normalization - self.velocity_expect) / self.velocity_expect) <= 0.075 and abs(
                        self.acceleration / self.acceleration_normalization / self.velocity_expect) <= 5:
                    self.is_episode_done = True
                    print("----------------------11111一次episode结束因为速度符合条件-------------------------episode",
                          self.episode_number)

                elif step_number >= self.MAX_EP_STEPS:
                    self.is_episode_done = True
                    print("----------------------22222一次episode结束因为step 50次--------------------------episode",
                          self.episode_number)

    def show_episode_result(self):
        """
        每个episode结束时，可以调用此函数显示信息
        :return: 本次 episode 信息
        """
        res = "episode" + str(self.episode_number) + "reward:" + str(self.episode_reward)
        return res
