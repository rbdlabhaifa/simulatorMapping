import os
import argparse
import pickle
from typing import Dict, Any, List, Callable
import numpy as np
from dataclasses import dataclass
from sklearn.neighbors import KNeighborsRegressor
from tensorflow import keras
from sklearn.decomposition import PCA



@dataclass
class Dataset:
    X: List[List[int]]
    Y: List[float]

def load_leaves_to_weights(leaves_to_weights_path):
    leaves_to_weights = None
    with open(leaves_to_weights_path, 'rb') as handle:
        leaves_to_weights = pickle.load(handle)
    assert leaves_to_weights is not None, f'failed to load {leaves_to_weights_path}'
    return leaves_to_weights


def convert_single_vector_to_binary(v: str):
    numbers_list = v.split()
    binary_list = [bin(int(x))[2:].zfill(8) for x in numbers_list]
    concat_binary = ''.join(binary_list)
    binary_vector = [int(x) for x in concat_binary]
    return binary_vector


def convert_vectors_to_binary(leaves_to_weights: Dict[str, str], size=5000):
    vectors = []
    labels = []
    c = 0
    for decimal_vector, weight in leaves_to_weights.items():
        if c == size:
            break
        else:
            c += 1
        binary_vector = convert_single_vector_to_binary(decimal_vector)
        vectors.append(binary_vector)
        labels.append(float(weight))
    return Dataset(vectors, labels)


# Guassian


@dataclass
class NormalDistribution:
    mean: float
    std: float


def gaussian_likelihood(leaves_to_weights: Dict[str, str]):
    samples = [float(x) for x in leaves_to_weights.values()]
    loc = np.mean(samples)
    scale = np.std(samples)
    return NormalDistribution(loc, scale)


def get_normal_ditribution(leaves_to_weights_path) -> NormalDistribution:
    leaves_to_weights = load_leaves_to_weights(leaves_to_weights_path)
    distribution = gaussian_likelihood(leaves_to_weights)
    return distribution


def sample_from_distribution(normal_distribution: NormalDistribution, *args, **kwargs):
    sample = np.random.normal(normal_distribution.mean, normal_distribution.std)
    return sample


# KNN

    
def prepare_knn_model(leaves_to_weights_path):
    leaves_to_weights = load_leaves_to_weights(leaves_to_weights_path)
    dataset: Dataset = convert_vectors_to_binary(leaves_to_weights)
    neigh = KNeighborsRegressor(n_neighbors=3)
    neigh.fit(dataset.X[:5000], dataset.Y[:5000])
    return neigh


def predict_knn_regressor(neigh: KNeighborsRegressor, test_sample: List[List[int]], *args, **kwargs):
    test_sample = convert_single_vector_to_binary(test_sample)
    pred = neigh.predict([test_sample])
    return pred[0]


# Regression


def get_linear_model():
    print("building linear model")
    x = keras.layers.Input(shape=(256,))
    y = keras.layers.Dense(units=16, activation='relu', kernel_regularizer=keras.regularizers.L2(0.1))(x)
    y = keras.layers.Dense(units=8, activation='relu', kernel_regularizer=keras.regularizers.L2(0.5))(y)
    y = keras.layers.Dense(units=1)(y)

    model = keras.Model(x, y)
    compile_kwargs = {
        "loss": keras.losses.mean_squared_error,
        "optimizer": keras.optimizers.Adam(),
        "metrics": keras.metrics.MAE
    }
    print(f"compiling model with {compile_kwargs}")
    model.compile(**compile_kwargs)
    model.summary(line_length=150)
    return model

def get_data(size: int,leaves_to_weights_path):
    print("building dataset")
    leaves_to_weigths: Dict[str, str] = load_leaves_to_weights(leaves_to_weights_path)
    print("converting to binary")
    dataset: Dataset = convert_vectors_to_binary(leaves_to_weigths, size=size)
    x = np.array(dataset.X)
    y = np.array(dataset.Y)

    val_percent = 0.3
    x_train = x[:int(len(x) * (1 - val_percent))]
    y_train = y[:int(len(y) * (1 - val_percent))]

    x_val = x[int(len(x) * (1 - val_percent)):]
    y_val = y[int(len(y) * (1 - val_percent)):]

    return x_train, y_train, x_val, y_val

def train_model(model: keras.Model, x_train: np.array, y_train: np.array, x_val: np.array, y_val: np.array, train_steps: int, val_steps: int):
    print("starting model fit")
    model.fit(
        x_train,
        y_train,
        batch_size=4,
        epochs=8,
        steps_per_epoch=train_steps,
        shuffle=True,
        validation_data=(x_val, y_val),
        validation_steps=val_steps,
        validation_batch_size=4,
        workers=1
    )

    print("evaluatong the model")
    model.evaluate(x_val, y_val, batch_size=4)
    return model

def build_model(size, train_steps, val_steps,leaves_to_weights_path):
    model = get_linear_model()
    x_train, y_train, x_val, y_val = get_data(size,leaves_to_weights_path)
    trained_model = train_model(model, x_train, y_train, x_val, y_val, train_steps, val_steps)
    return trained_model


def get_model_for_predict():
    model = keras.saving.load_model("/home/pi/orb_slam3/model_dense16_dense8_reg_weak/")
    return model


def predict_with_linear_model(model: keras.Model, test_sample: List[List[int]], *args, **kwargs):
    x = convert_single_vector_to_binary(test_sample)
    x = np.array(x)
    x = x.reshape((1, 256))
    pred = model.predict(x)[0][0]
    return pred

# PCA

def apply_pca():
    x_train, _, _, _ = get_data(size=50000)
    pca = PCA(n_components=2)
    X_train = pca.fit_transform(x_train)

    import pandas as pd
    pca_df = pd.DataFrame(data=X_train, columns=["PC1", "PC2"])

    import seaborn as sns
    import matplotlib.pyplot as plt

    sns.set()
    sns.lmplot(
        x="PC1",
        y = "PC2",
        data=pca_df
    )
    plt.title("")
    plt.show()

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output_path', help="path to dump model", required=True)
    args = parser.parse_args()
    output_path = args.output_path
    assert not os.path.exists(output_path)


    size = 200000
    train_steps = 4000
    val_steps = 100

    trained_model = build_model(size, train_steps, val_steps)
    os.makedirs(output_path)
    keras.saving.save_model(trained_model, output_path)