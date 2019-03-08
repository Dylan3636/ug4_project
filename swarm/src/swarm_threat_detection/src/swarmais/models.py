import numpy as np
import pandas as pd
from functools import reduce
from keras.models import Sequential
from keras.layers import Dense, LSTM, InputLayer
from keras.optimizers import Adam, SGD
from keras.regularizers import l2
from keras.wrappers.scikit_learn import KerasClassifier
from keras.callbacks import TensorBoard, ModelCheckpoint, LearningRateScheduler
from tensorboardcolab import *
from keras import backend as K
from tensorflow import set_random_seed, Session

from sklearn.model_selection import ParameterGrid, train_test_split
from sklearn.metrics import mean_squared_error, r2_score
from swarmais.schedulers import *
from swarmais.preprocessing import *


# Set TensorFlow Session
# sess = Session()
# K.set_session(sess)


def coeff_determination(y_true, y_pred):
    """R^2 Score function"""
    SS_res = K.sum(K.square(y_true-y_pred))
    SS_tot = K.sum(K.square(y_true - K.mean(y_true)))
    return 1 - SS_res/(SS_tot + K.epsilon())


def get_callbacks(tensorboardfp, modelcheckptfp, colab=False, **schedulerkwparams):
    """Gets Callbacks to pass to model fit function"""
    if colab:
        tbc = TensorBoardColab(graph_path=tensorboardfp)
        tensorboardcb = TensorBoardColabCallback(tbc)
    else:
        tensorboardcb = TensorBoard(log_dir=tensorboardfp, histogram_freq=0,
                                write_graph=True, write_images=True)
    modelcheckptcb = ModelCheckpoint(modelcheckptfp, monitor='val_loss', save_best_only=True, verbose=0)
    scheduler = CosineAnnealingWithWarmRestarts(**schedulerkwparams)
    schedulercb = LearningRateScheduler(schedule=scheduler.update_learning_rule)
    return [tensorboardcb, modelcheckptcb, schedulercb]


def nn_holdout_score(nnreg, trainX, trainY, valX, valY, num_epochs, batch_size, callbacks):
    history = nnreg.fit(trainX,
                        trainY,
                        validation_data=(valX, valY),
                        epochs=num_epochs,
                        batch_size=batch_size,
                        callbacks=callbacks,
                        verbose=1
                        )
    train_pred = nnreg.predict(trainX)
    val_pred = nnreg.predict(valX)

    train_r2 = r2_score(trainY, train_pred)
    val_r2 = r2_score(valY, val_pred)
    return train_r2, val_r2, history


def create_ffnn_model(layers: list, optimizer: str):
    """Feed-Forward Model Constructor"""
    model = Sequential()
    model.add(Dense(layers[0], input_dim=2, activation='relu'))
    for num_neurons in layers[1::]:
        model.add(Dense(num_neurons, activation='relu'))
    model.add(Dense(2, activation = 'linear'))
    model.compile(loss='mse', optimizer=optimizer, metrics=[coeff_determination])
    return model


def create_rnn_model(seq_length: int, num_lstm_units: int, optimizer: str):
    model = Sequential()
    model.add(InputLayer((seq_length, 2)))
    model.add(LSTM(num_lstm_units))
    model.add(Dense(2, activation='linear'))
    model.compile(loss='mse', optimizer=optimizer, metrics=[coeff_determination])
    return model


def train_ffnn(datapath, graphdir, modelpath,
               layers, optimizer,
               total_iters_per_period,
               batch_size, num_epochs=1000, seed=0, colab=False):

    set_random_seed(seed)

    if type(layers) == int:
        layers = [layers]

    # Model Name
    modelname = "ffnn_{}".format(batch_size)+'_'
    modelname += reduce(lambda x, y: x + y, ['{}_'.format(layer) for layer in layers])
    modelname += "{}_{}_{}".format(optimizer, total_iters_per_period, seed)

    # Set up callbacks
    callbacks = get_callbacks(graphdir+"/"+modelname,
                              modelpath+"/"+modelname+".hd5",
                              colab=colab,
                              total_iters_per_period=total_iters_per_period)
    # Construct model
    ffnnreg = create_ffnn_model(layers, optimizer)

    # Load parsed data
    data = pd.read_csv(datapath)

    # Split data
    result = train_val_test_split(data, seed)
    train_indicies, X_train, y_train = result['train']
    val_indicies, X_val, y_val = result['val']
    test_indicies, X_test, y_test = result['test']

    # Normalize data
    result = normalize_data(X_train, y_train)
    x_scaler, y_scaler = result['scalers']
    X_train_norm, y_train_norm = result['data']

    result = normalize_data(X_val, y_val, x_scaler, y_scaler)
    X_val_norm, y_val_norm = result['data']

    result = normalize_data(X_test, y_test, x_scaler, y_scaler)
    X_test_norm, y_test_norm = result['data']

    results = nn_holdout_score(ffnnreg,
                               X_train_norm,
                               y_train_norm,
                               X_val_norm,
                               y_val_norm,
                               num_epochs, batch_size, callbacks)
    print("Completed Training!")
    df = pd.DataFrame(index=["Train", "Val", "Test"], columns=["Loss", "R2_Score"])
    print("Evaluating Network")
    df.loc["Train", :] = ffnnreg.evaluate(X_train_norm, y_train_norm)
    df.loc["Val", :] = ffnnreg.evaluate(X_val_norm, y_val_norm)
    df.loc["Test", :] = ffnnreg.evaluate(X_test_norm, y_test_norm)

    csv_file = modelpath+"/"+modelname+".csv"
    df.to_csv(csv_file)
    print("Saving evaluation results to {}".format(csv_file))

    return ffnnreg, results


def train_rnn(datapath,
              graphdir='graphlogs',
              modelpath='models',
              seq_length=10,
              periods=5,
              num_lstm_units=64,
              optimizer='adam',
              total_iters_per_period=100,
              batch_size=32,
              num_epochs=1000,
              seed=0):

    # Load parsed data
    data = pd.read_csv(datapath)
    data = data.sort_values(by="BaseDateTime")

    print("Constructing sequences")
    X_rec, y_rec = construct_timeseries_data(data, seq_length, periods)

    train_indicies_rec, test_indicies_rec = train_test_split(range(np.size(X_rec, 0)),
                                                             random_state=seed,
                                                             shuffle=True,
                                                             train_size=0.7,
                                                             test_size=0.3)
    train_indicies_rec, val_indicies_rec = train_test_split(train_indicies_rec,
                                                            random_state=seed,
                                                            shuffle=True,
                                                            train_size=0.7,
                                                            test_size=0.3)

    X_train_rec = X_rec[train_indicies_rec, :, :]
    X_val_rec = X_rec[val_indicies_rec, :, :]
    X_test_rec = X_rec[test_indicies_rec, :, :]

    y_train_rec = y_rec[train_indicies_rec, :]
    y_val_rec = y_rec[val_indicies_rec, :]
    y_test_rec = y_rec[test_indicies_rec, :]


    # Normalize data
    result = normalize_data(X_train_rec.reshape(-1, 2), y_train_rec)
    x_scaler, y_scaler = result['scalers']
    X_train_norm_rec = result['data'][0].reshape(-1, seq_length, 2)
    y_train_norm_rec = result['data'][1]

    result = normalize_data(X_val_rec.reshape(-1, 2), y_val_rec, x_scaler, y_scaler)
    X_val_norm_rec = result['data'][0].reshape(-1, seq_length, 2)
    y_val_norm_rec = result['data'][1]

    result = normalize_data(X_test_rec.reshape(-1, 2), y_test_rec, x_scaler, y_scaler)
    X_test_norm_rec = result['data'][0].reshape(-1, seq_length, 2)
    y_test_norm_rec = result['data'][1]

    # Model name
    modelname = 'rnn_{}_{}_{}_{}_{}'.format(batch_size,
                                            num_lstm_units,
                                            optimizer,
                                            total_iters_per_period,
                                            seed)

    # Set up callbacks
    callbacks = get_callbacks(graphdir+"/"+modelname,
                              modelpath+"/"+modelname+'.hd5',
                              total_iters_per_period=total_iters_per_period)

    print(seq_length, num_lstm_units, optimizer)
    rnnreg = create_rnn_model(seq_length, num_lstm_units, optimizer)
    rnnreg.summary()
    results = nn_holdout_score(rnnreg,
                               X_train_norm_rec,
                               y_train_norm_rec,
                               X_val_norm_rec,
                               y_val_norm_rec,
                               num_epochs,
                               batch_size,
                               callbacks)

    print("Completed Training!")
    df = pd.DataFrame(index=["Train", "Val", "Test"], columns=["Loss", "R2_Score"])
    print("Evaluating Network")
    df.loc["Train", :] = rnnreg.evaluate(X_train_norm_rec, y_train_norm_rec)
    df.loc["Val", :] = rnnreg.evaluate(X_val_norm_rec, y_val_norm_rec)
    df.loc["Test", :] = rnnreg.evaluate(X_test_norm_rec, y_test_norm_rec)

    csv_file = modelpath+"/"+modelname+".csv"
    df.to_csv(csv_file)
    print("Saving evaluation results to {}".format(csv_file))

    return rnnreg, results
