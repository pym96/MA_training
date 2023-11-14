'''
    Author: Pan Jiaxiang
    Date: 2023 / 8 / 11
    Where: On the going home train
'''
import numpy as np
import random



#### Miscellaneous functions
def sigmoid(z):
    """The sigmoid function."""
    return 1.0/(1.0+np.exp(-z)) 

def sigmoid_prime(z):
    """Derivative of the sigmoid function."""
    return sigmoid(z)*(1-sigmoid(z))

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

    def feedback(self,a):
        '''Return the output of the network of if "a" is the input'''
        for b, w in zip(self.bias, self.weights):
            a = sigmoid(np.dot(w,a) + b)
        return a
    
    
    # For example, update your SGD method:
    def SGD(self, trainloader, epochs, mini_batch_size, eta, testloader=None):
        if testloader:
            n_test = len(testloader.dataset)
        
        n = len(trainloader.dataset)

        for j in range(epochs):
            for i, data in enumerate(trainloader, 0):
                inputs, labels = data
                inputs = inputs.view(inputs.shape[0], -1).numpy()
                labels = labels.numpy().reshape((-1, 1))

                mini_batch = list(zip(inputs, labels))
                self.update_mini_batch(mini_batch, eta)

            if testloader:
                print("Epoch {0}: {1} / {2}".format(
                    j, self.evaluate(testloader), n_test
                ))
            else:
                print("Epoch {0} complete".format(j))

    # Update the evaluate method:
    def evaluate(self, testloader):
        test_results = []
        for data in testloader:
            inputs, labels = data
            inputs = inputs.view(inputs.shape[0], -1).numpy()
            labels = labels.numpy()

            test_results.extend([(np.argmax(self.feedforward(x)), y) for (x, y) in zip(inputs, labels)])

        return sum(int(x == y) for (x, y) in test_results)


    def update_mini_batch(self, mini_batch, eta):
        """Update the network's weights and biases by applying
        gradient descent using backpropagation to a single mini batch.
        The ``mini_batch`` is a list of tuples ``(x, y)``, and ``eta``
        is the learning rate."""
        nabla_b = [np.zeros(b.shape) for b in self.bias]
        nabla_w = [np.zeros(w.shape) for w in self.weights]
        for x, y in mini_batch:
            delta_nabla_b, delta_nabla_w = self.backprop(x, y)
            nabla_b = [nb+dnb for nb, dnb in zip(nabla_b, delta_nabla_b)]
            nabla_w = [nw+dnw for nw, dnw in zip(nabla_w, delta_nabla_w)]
        self.weights = [w-(eta/len(mini_batch))*nw
                        for w, nw in zip(self.weights, nabla_w)]
        self.bias = [b-(eta/len(mini_batch))*nb
                    for b, nb in zip(self.bias, nabla_b)]


    def backprop(self, x, y):
            """Return a tuple ``(nabla_b, nabla_w)`` representing the
            gradient for the cost function C_x.  ``nabla_b`` and
            ``nabla_w`` are layer-by-layer lists of numpy arrays, similar
            to ``self.biases`` and ``self.weights``."""
            nabla_b = [np.zeros(b.shape) for b in self.bias]
            nabla_w = [np.zeros(w.shape) for w in self.weights]
            # feedforward
            activation = x
            activations = [x] # list to store all the activations, layer by layer
            zs = [] # list to store all the z vectors, layer by layer
            for b, w in zip(self.bias, self.weights):
                z = np.dot(w, activation)+b
                zs.append(z)
                activation = sigmoid(z)
                activations.append(activation)
            # backward pass
            delta = self.cost_derivative(activations[-1], y) * \
                sigmoid_prime(zs[-1])
            nabla_b[-1] = delta
            nabla_w[-1] = np.dot(delta, activations[-2].transpose())
            # Note that the variable l in the loop below is used a little
            # differently to the notation in Chapter 2 of the book.  Here,
            # l = 1 means the last layer of neurons, l = 2 is the
            # second-last layer, and so on.  It's a renumbering of the
            # scheme in the book, used here to take advantage of the fact
            # that Python can use negative indices in lists.
            for l in range(2, self.num_layers):
                z = zs[-l]
                sp = sigmoid_prime(z)
                delta = np.dot(self.weights[-l+1].transpose(), delta) * sp
                nabla_b[-l] = delta
                nabla_w[-l] = np.dot(delta, activations[-l-1].transpose())
            return (nabla_b, nabla_w)
            
            
    def cost_derivative(self, output_activations, y):
        """Return the vector of partial derivatives \partial C_x /
        \partial a for the output activations."""
        return (output_activations-y)



''''
Build a neural network
    Input layer: 784 neurons
    Concealed layer: 128 neurons + 64 neurons
    Output layer: 10 neurons
'''
net = Network([784,128,64,10])

print(net.sizes)


import torch
import torchvision
import torchvision.transforms as transforms
import numpy as np

transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.5,), (0.5,))])

trainset = torchvision.datasets.MNIST(root='./data', train=True, download=True, transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=4, shuffle=True)

testset = torchvision.datasets.MNIST(root='./data', train=False, download=True, transform=transform)
testloader = torch.utils.data.DataLoader(testset, batch_size=4, shuffle=False)

# Update the main part of your code to use trainloader and testloader:
net.SGD(trainloader, epochs=5, mini_batch_size=64, eta=0.1, testloader=testloader)