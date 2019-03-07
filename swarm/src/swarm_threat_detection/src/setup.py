from distutils.core import setup

requirements = ['argparse', 'tqdm', 'pandas', 'sklearn', 'tensorflow', 'tensorboardcolab']
setup(
    name="swarmais",
    version="0.0.1",
    packages=["swarmais"],
    scripts=['scripts/ffnn_grid_search',
             'scripts/evaluate_ffnn_model',
             'scripts/parse_ais_dataset'],
   install_requires=requirements
)
