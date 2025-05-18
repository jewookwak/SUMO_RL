# train/embedding.py
import tensorflow as tf
import numpy as np

class Embedding(tf.keras.Model):
    def __init__(self, embedding_dim):
        super(Embedding, self).__init__()
        self.layer = tf.keras.layers.Dense(embedding_dim, activation='relu')
        self.embedding_dim = embedding_dim

    def call(self, batch_size, num_quantile, tau_min, tau_max):
        sample = tf.random.uniform(
            [batch_size * num_quantile, 1],
            minval=tau_min, maxval=tau_max, dtype=tf.float32)
        sample_tile = tf.tile(sample, [1, self.embedding_dim])
        embedding = tf.cos(
            tf.cast(tf.range(0, self.embedding_dim, 1), tf.float32) * np.pi * sample_tile)
        embedding_out = self.layer(embedding)
        return embedding_out, sample