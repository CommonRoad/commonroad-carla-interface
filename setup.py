import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="commonroad-carla-interface",
    version="0.0.1",
    author="Cyber-Physical Systems Group, Technical University of Munich",
    author_email="commonroad@lists.lrz.de",
    description="It's pip... with git.",
    long_description=long_description,
    url="https://gitlab.lrz.de/mpfav-ss21-driving-simulator/commonroad-carla-interface.git",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=[
        "numpy>=1.2",
        "pygame>=2.1.2",
        "commonroad-io>=2021.4",
        "imageio~=2.9.0",
        "scipy>=1.2.0",
        "PyQt5>=5.12.2",
        "carla>=0.9.13",
        "typer >= 0.4.0",
        "moviepy >= 1.0.3"
    ]

)