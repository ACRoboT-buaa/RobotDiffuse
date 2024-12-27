import numpy as np

def shuffle_in_unison_scary(*args, **kwargs):
	rng_state = np.random.get_state()
	for i in range(len(args)):
		np.random.shuffle(args[i])#训练深度学习模型时，可以用它来打乱训练样本的顺序，以保证模型不会受到数据顺序的影响
		np.random.set_state(rng_state)