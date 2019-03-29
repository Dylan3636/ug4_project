#!/home/dylan/miniconda3/envs/mlp/bin/python

import numpy as np
import pandas as pd
from swarmais.analysisutils import *
from tqdm import tqdm


def drop_lt(df, col, min_val):
    return df[df.loc[:, col]>min_val]


def drop_gt(df, col, max_val):
    return df[df.loc[:, col]<max_val]


def drop_eq(df, col, eq_val):
    return df[df.loc[:, col]!=eq_val]


def drop_neq(df, col, eq_val):
    return df[df.loc[:, col]==eq_val]


def drop_cond(df, col, cond_func):
    return df.drop(df[df.loc[:, col].map(cond_func)].index)


def load_dataset(dataset_path, num_chunks=1000, chunk_size=100000, save=True, save_path=None):
    count = 0
    data = pd.DataFrame()
    # dice_file_loc="AIS_ASCII_by_UTM_Month/2017_v2/AIS_2017_01_Zone18.csv"
    # file_loc = "/media/dylan/1A46509946507809/AIS_ASCII_by_UTM_Month/2017_v2/AIS_2017_01_Zone18.csv"
    mid_lat = 39.248405
    mid_lon = -74.81905
    pbar = tqdm(total=num_chunks)
    column_names = ['MMSI', 'LAT', 'LON', 'SOG', 'COG', 'Heading', 'VesselType', 'BaseDateTime']

    for chunk in pd.read_csv(dataset_path, chunksize=chunk_size, usecols=column_names):

        # Drop Nans
        chunk=chunk.dropna()

        # Drop stationary points
        chunk = drop_lt(chunk, 'SOG', 0.5)

        # Drop points with nonsensical heading
        chunk = drop_gt(chunk, 'Heading', 500)

        # Pick tracks coming from three specific vessel types
        chunk.VesselType = chunk.VesselType.map(int)
        chunk = drop_cond(chunk, 'VesselType', lambda vessel: vessel not in [1004, 1025, 1024])

        # Feature Construction
        chunk.loc[:, 'Xcoord'] = geo2coord(mid_lon, mid_lat, chunk.LON, chunk.LAT, False)
        chunk.loc[:, 'Ycoord'] = geo2coord(mid_lon, mid_lat, chunk.LON, chunk.LAT, True)

        chunk.loc[:, 'BaseDateTime'] = pd.to_datetime(chunk['BaseDateTime'])

        # Append chunk
        data = pd.concat([data, chunk], axis=0, ignore_index=True)
        pbar.update(1)
        pbar.set_description("{:.3f} MB".format(np.sum(data.memory_usage())/1024.0**2))
        pbar.refresh()
        if count>num_chunks:
            break
        count+=1

    # Remove short tracks
    data = data.groupby("MMSI", sort=False).filter(func=lambda grp : len(grp)>500)

    # Sort tracks
    data.sort_values(by='BaseDateTime', inplace=True)

    # Calculate and smooth pointwise heading
    data = data.groupby("MMSI", sort=False).apply(smooth_vessel)
    data = data.drop(columns='MMSI').reset_index(0)
    data.info()

    if save:
        assert save_path, "File path cannot be none when saving file"
        data.to_csv(save_path)
    return data
