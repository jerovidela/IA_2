import numpy as np

class NeuralNetwork:
    def __init__(self, input_nodes=6, hidden_nodes=6, output_nodes=3):
        self.input_nodes = input_nodes
        self.hidden_nodes = hidden_nodes
        self.output_nodes = output_nodes
        self.initialize()

        # ======================== INITIALIZE NETWORK WEIGTHS AND BIASES =============================
    def initialize(self):

            self.W1 = np.random.uniform(-1, 1, (self.input_nodes, self.hidden_nodes))
            self.b1 = np.random.uniform(-1, 1, (self.hidden_nodes,))
            
            self.W2 = np.random.uniform(-1, 1, (self.hidden_nodes, self.output_nodes))
            self.b2 = np.random.uniform(-1, 1, (self.output_nodes,))


        # ============================================================================================

    def think(self, inputs):
            """
            3.1.2: Propagación hacia adelante con ReLU y Softmax.
            """
            x = np.array(inputs)
            
            # Capa oculta: Activación ReLU
            z1 = np.dot(x, self.W1) + self.b1
            a1 = np.maximum(0, z1) 
            
            # Capa de salida: Activación Softmax
            z2 = np.dot(a1, self.W2) + self.b2
            
            # Truco de estabilidad numérica para Softmax (evita desbordamientos exponenciales)
            exp_scores = np.exp(z2 - np.max(z2)) 
            a2 = exp_scores / np.sum(exp_scores)
            
            return a2

    def act(self, inputs):
            """
            3.1.3: Toma de decisión basada en la probabilidad máxima.
            """
            outputs = self.think(inputs)
            
            # argmax devuelve el índice con la probabilidad más alta (0, 1 o 2)
            action_index = np.argmax(outputs)
            
            if action_index == 0:
                return "JUMP"
            elif action_index == 1:
                return "DUCK"
            else:
                return "RUN"
