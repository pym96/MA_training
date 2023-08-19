'''
    Author: Pan Jiaxiang
    Date: 2023 / 8 / 11
    Where: On the going home train
'''
import numpy as np
import random

'''
    @param sizes: Standing neuron amount of different layers
    @param bias: Standing for bias, initialized with random array
    @param weight: Standing for weight, initailized with random array
    @Tips: Both bias and weigths will be fine-tuned by back-proporgation
'''
class Network(object):
    def __init__(self,sizes):
        self.num_layers = len(sizes)
        self.sizes = sizes
        self.bias =  [np.random.randn(y,1) for y in sizes[1:]]
        self.weights = [np.random.randn(y,x)
                        for x,y in zip(sizes[:-1],sizes[1:])]

    def sigmoid(z):
        return 1.0 / (1.0 + np.exp(-z))

    def feedback(self,a):
        '''Return the output of the network of if "a" is the input'''
        for b, w in zip(self.bias, self.weights):
            a = sigmoid(np.dot(w,a) + b)
        return a

    def SGD(self, training_data, epochs, mini_batch_size, eta, test_data=None):
        '''
            Training the neural network using mini-batch stochastic
            gradient descent. The "training_data" is a list of tuples
            of "(x,y)" representing the training inputs and the desired 
            output. The other non-optional parameters are 
                self-explanator. If the "test_data' is provided then the epoch,
                and the partial progress will be printed out. This is useful when
                tracking progress but slow things down substantially. 
        '''
        if test_data: n_test = len(test_data)
        n = len(training_data)
        
        for j in xrange(epochs):
            random.shuffle(training_data)
            mini_batches = [
                training_data[k:k+mini_batch_size]
                for k in xrange(0, n, mini_batch_size)]

            for mini_batch in mini_batches:
                self.update_mini_batch(mini_batch, eta)
            if test_data:
                print "Epoch {0}: {1} / {2}".format(
                    j, self.evaluate(test_data), n_test
                )
            else:
                print "Epoch {0} complete".format(j)


        def update_mini_batch(self, mini_batch, eta):
            '''
                Update network's biaes and weights by applying gradient descent
                using backpropagation to a single mini batch
                the "mini_batch" is a list of tuple "(x,y)", and "eta" is the 
                learning rate
            '''
            nabla_b = [np.zeros(b.shape) for b in self.biaes]
            nabla_w = [np.zeros(w.shape) for w in self.weigths]

            for x,y in mini_batch:
                delta_nabla_b, delta_nabla_w = self.backprop(x, y)
                nabla_b = [nb + dnb for nb, dnb in zip(nabla_b, delta_nabla_b)]
                nabla_w = [nb + dnw for nw, dnw in zip(nabla_w, delta_nabla_w)]
                self.weights = [w - (eta / len(mini_batch) * nw
                                for w, nw in zip(self.weights, nabla_w))]
                self.biaes = [b - (eta / len(mini_batch) * nb)
                                for b, nb in zip(self.biaes, nabla_b)]


''''
Build a neural network
    Input layer: 2 neurons
    Concealed layer: 1 neurons
    Output layer: 3 neurons
'''
net = Network([2,1,3])


