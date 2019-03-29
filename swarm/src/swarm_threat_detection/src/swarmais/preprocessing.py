from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tqdm import tqdm
import numpy as np
import pandas as pd


def train_val_test_split(data, seed=0):
    """ Split parsed AIS Data into train-val-test split"""
    mask = data.SmoothedVectorYcoord >= 0
    subdata = data.loc[mask, :]

    # Split Data
    train_indicies, test_indicies = train_test_split(subdata.index, random_state=seed, shuffle=True, train_size=0.7, test_size=0.3)
    train_indicies, val_indicies = train_test_split(train_indicies, random_state=seed, shuffle=True, train_size=0.7, test_size=0.3)
    train_data = data.loc[train_indicies, :]
    val_data = data.loc[val_indicies, :]
    test_data = data.loc[test_indicies, :]

    X_train = train_data.loc[:, ['Xcoord', 'Ycoord']]
    X_val = val_data.loc[:, ['Xcoord', 'Ycoord']]
    X_test = test_data.loc[:, ['Xcoord', 'Ycoord']]

    y_train = train_data.loc[:, ['SmoothedVectorXcoord', 'SmoothedVectorYcoord']]
    y_val = val_data.loc[:, ['SmoothedVectorXcoord', 'SmoothedVectorYcoord']]
    y_test = test_data.loc[:, ['SmoothedVectorXcoord', 'SmoothedVectorYcoord']]
    return {'train': (train_indicies, X_train, y_train),
            'val': (val_indicies, X_val, y_val),
            'test': (test_indicies, X_test, y_test)}


def normalize_data(X, y, xscaler=None, yscaler=None):
    """Normalize data"""
    xnone = False
    ynone = False
    if xscaler is None:
        xscaler = StandardScaler()
        xnone = True
    if yscaler is None:
        yscaler = StandardScaler()
        ynone = True

    if type(X) == pd.DataFrame:
        X_norm = X.copy()
        X_norm.loc[:, :] = xscaler.fit_transform(X) if xnone else xscaler.transform(X)

        y_norm = y.copy()
        y_norm.loc[:, :] = yscaler.fit_transform(y) if ynone else yscaler.transform(y)
    else:
        X_norm = xscaler.fit_transform(X) if xnone else xscaler.transform(X)
        y_norm = yscaler.fit_transform(y) if ynone else yscaler.transform(y)

    return {'scalers': (xscaler, yscaler), 'data': (X_norm, y_norm)}


def construct_vessel_timeseries_data(vessel, seq_length, periods):
    X = [vessel.loc[vessel.index[i*periods:i*periods+seq_length], ['Xcoord', 'Ycoord']].values
         for i in range(int((len(vessel)-seq_length)/periods))]
    y = [vessel.loc[vessel.index[i*periods+seq_length-1], ['SmoothedVectorXcoord', 'SmoothedVectorYcoord']].values
         for i in range(int((len(vessel)-seq_length)/periods))]
    return X, y


def construct_timeseries_data(data, seq_length, periods):
    X=[]
    y=[]
    for mmsi, vessel in tqdm(data.groupby("MMSI")):
        vesselX, vesselY = construct_vessel_timeseries_data(vessel, seq_length, periods)
        X += vesselX
        y += vesselY
    return np.reshape(X, (-1, seq_length, 2)), np.reshape(y, (-1, 2))
