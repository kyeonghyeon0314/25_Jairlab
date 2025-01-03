# syntax=docker/dockerfile:1

	# Build
# $ docker build -t cascio99/test:06 .
	# Run
# $ docker run -it --gpus all --mount type=bind,src="$(pwd)",target=/dst cascio99/test:06 bash

# Use an official NVIDIA CUDA base image with Ubuntu 20.04 and CUDA 11.0.3 & cuDNN 8.0.5
FROM ubuntu:20.04

# Set environment variables
ENV PYTHON_VERSION=3.8
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        build-essential \
        curl \
        git \
        wget \
        ca-certificates

# Add the deadsnakes PPA to get Python 3.8
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update

# Install Python 3.8 and development headers
RUN apt-get install -y --no-install-recommends \
        python${PYTHON_VERSION} \
        python${PYTHON_VERSION}-dev \
        python${PYTHON_VERSION}-distutils

# Update alternatives to use Python 3.8 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHON_VERSION} 1 && \
    update-alternatives --config python3

# Install the latest pip
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.8

# Ensure 'python' command is available via python-is-python3(ft. need for 'bash setup.sh')
RUN apt-get install -y python-is-python3

# # Install specific PyTorch version with CUDA 11.0 support
# RUN pip install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

# ENV CUDAVER=cuda-11.0
# ENV PATH=/usr/local/$CUDAVER/bin:$PATH
# ENV LD_LIBRARY_PATH=/usr/local/$CUDAVER/lib:$LD_LIBRARY_PATH
# ENV LD_LIBRARY_PATH=/usr/local/$CUDAVER/lib64:$LD_LIBRARY_PATH
# ENV CUDA_PATH=/usr/local/$CUDAVER
# ENV CUDA_ROOT=/usr/local/$CUDAVER
# ENV CUDA_HOME=/usr/local/$CUDAVER
# ENV CUDA_HOST_COMPILER=/usr/bin/gcc-9.4

# Clean up temporary files to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Set working directory inside the container
WORKDIR /dst

###############################################################################################
					# TODO inside the container
    # cuda11.3.0
# $ wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda_11.3.0_465.19.01_linux.run
# docker cp ...
# sh cuda_11.3.0_465.19.01_linux.run
# for verification, `nvcc --version`
    # cudnn8.2.0
# download cuDNN Library for Linux (x86_64) from https://developer.nvidia.com/rdp/cudnn-archive
# docker cp ...
# tar -xzvf cudnn-11.3-linux-x64-v8.2.0.53.tgz
# cp cuda/include/cudnn* /usr/local/cuda/include
# cp cuda/lib64/libcudnn* /usr/local/cuda/lib64
# chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
# for verification, `cat /usr/local/cuda/include/cudnn_version.h | grep -n CUDNN_MAJOR -A 2`
    # pytorch1.10.0
# pip install torch==1.10.0+cu113 torchvision==0.11.0+cu113 torchaudio==0.10.0 -f https://download.pytorch.org/whl/torch_stable.html

    # Installation
# Step 4: Install MMDetection3D
# Step 4.1: Install MMEngine, MMCV, and MMDetection using MIM
# pip install -U openmim
# mim install mmengine
# mim install 'mmcv>=2.0.0rc4'
# mim install 'mmdet>=3.0.0'
# Step 4.2: Install MMDetection3D
# git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x
# cd mmdetection3d
# pip install -v -e .
# Step 5: Install Sparse Convolution Backend
# Step 5.1: Install SPConv
# pip install cumm-cu113 && pip install spconv-cu113
# Step 5.2: Install TorchSparse
# apt-get install libsparsehash-dev
# pip install --upgrade git+https://github.com/mit-han-lab/torchsparse.git@v1.4.0
# Step 5.3: Install Minkowski Engine (Optional) -> skip(failed)
# apt install build-essential python3-dev libopenblas-dev
# pip install -U git+https://github.com/NVIDIA/MinkowskiEngine --no-deps
# ERROR: Failed building wheel for MinkowskiEngine
#   Running setup.py clean for MinkowskiEngine
# Failed to build MinkowskiEngine
# ERROR: ERROR: Failed to build installable wheels for some pyproject.toml based projects (MinkowskiEngine)
# Step 6: Install nuScenes Devkit
# pip install nuscenes-devkit
    


# ********************
	# Prepare data for SemanticKitti

# git clone https://github.com/ldkong1205/LaserMix.git
# $ docker cp data/ $(container_name):/dst/LaserMix/
# https://github.com/ldkong1205/LaserMix/blob/main/docs/DATA_PREPARE.md#memo-create-semantickitti-dataset
# reorganize semantickitti dataset
# python ./tools/create_data.py semantickitti --root-path ./data/semantickitti --out-dir ./data/semantickitti --extra-tag semantickitti

                            # Train with a single GPU
# cd /dst/LaserMix
# python tools/train.py configs/lasermix/lasermix_cy3d_semi_semantickitti_10.py

        # Troubleshooting
#prob1
# Traceback (most recent call last):
#     File "tools/train.py", line 10, in <module>
#       from mmengine.runner import Runner
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/__init__.py", line 2, in <module>
#       from ._flexible_runner import FlexibleRunner
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/_flexible_runner.py", line 20, in <module>
#       from mmengine.hooks import Hook
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/hooks/__init__.py", line 9, in <module>
#       from .naive_visualization_hook import NaiveVisualizationHook
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/hooks/naive_visualization_hook.py", line 5, in <module>
#       import cv2
#     File "/usr/local/lib/python3.8/dist-packages/cv2/__init__.py", line 181, in <module>
#       bootstrap()
#     File "/usr/local/lib/python3.8/dist-packages/cv2/__init__.py", line 153, in bootstrap
#       native_module = importlib.import_module("cv2")
#     File "/usr/lib/python3.8/importlib/__init__.py", line 127, in import_module
#       return _bootstrap._gcd_import(name[level:], package, level)
#   ImportError: libGL.so.1: cannot open shared object file: No such file or directory
#sol for prob1
# apt-get update && apt-get install libgl1 ffmpeg libsm6 libxext6  -y
# install only 'libgl1' would be fine

####################################### flag #########################
# docker commit $(container_name) cascio99/test:07
####################################### flag #########################

#prob2
# Traceback (most recent call last):
#     File "tools/train.py", line 12, in <module>
#       from mmdet3d.utils import replace_ceph_backend
#   ModuleNotFoundError: No module named 'mmdet3d'
#sol for prob2
# echo 'export PYTHONPATH="${PYTHONPATH}:/dst/LaserMix/tools/train.py"' >> ~/.bashrc && source ~/.bashrc

#prob3
# Traceback (most recent call last):
#     File "tools/train.py", line 12, in <module>
#       from mmdet3d.utils import replace_ceph_backend
#     File "/dst/LaserMix/mmdet3d/__init__.py", line 3, in <module>
#       import mmdet
#     File "/usr/local/lib/python3.8/dist-packages/mmdet/__init__.py", line 16, in <module>
#       assert (mmcv_version >= digit_version(mmcv_minimum_version)
#   AssertionError: MMCV==2.2.0 is used but incompatible. Please install mmcv>=2.0.0rc4, <2.2.0.
#sol for prob3
# fix line10 in LaserMix/mmdet3d/__init__.py (/usr/local/lib/python3.8/dist-packages/mmdet/__init__.py?)
# mmcv_maximum_version = '2.1.0' ---> mmcv_maximum_version = '2.2.1'
# similar error...
# mmdet_maximum_version = '3.2.0' ---> mmdet_maximum_version = '3.3.1'

#prob4
# Traceback (most recent call last):
#     File "tools/train.py", line 135, in <module>
#       main()
#     File "tools/train.py", line 131, in main
#       runner.train()
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/runner.py", line 1728, in train
#       self._train_loop = self.build_train_loop(
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/runner.py", line 1520, in build_train_loop
#       loop = LOOPS.build(
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/registry/registry.py", line 570, in build
#       return self.build_func(cfg, *args, **kwargs, registry=self)
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/registry/build_functions.py", line 121, in build_from_cfg
#       obj = obj_cls(**args)  # type: ignore
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/loops.py", line 222, in __init__
#       super().__init__(runner, dataloader)
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/base_loop.py", line 26, in __init__
#       self.dataloader = runner.build_dataloader(
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/runner.py", line 1370, in build_dataloader
#       dataset = DATASETS.build(dataset_cfg)
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/registry/registry.py", line 570, in build
#       return self.build_func(cfg, *args, **kwargs, registry=self)
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/registry/build_functions.py", line 121, in build_from_cfg
#       obj = obj_cls(**args)  # type: ignore
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/dataset_wrapper.py", line 47, in __init__
#       self.datasets.append(DATASETS.build(dataset))
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/registry/registry.py", line 570, in build
#       return self.build_func(cfg, *args, **kwargs, registry=self)
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/registry/build_functions.py", line 121, in build_from_cfg
#       obj = obj_cls(**args)  # type: ignore
#     File "/dst/LaserMix/mmdet3d/datasets/semantickitti_dataset.py", line 79, in __init__
#       super().__init__(
#     File "/dst/LaserMix/mmdet3d/datasets/seg3d_dataset.py", line 111, in __init__
#       super().__init__(
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/base_dataset.py", line 247, in __init__
#       self.full_init()
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/base_dataset.py", line 298, in full_init
#       self.data_list = self.load_data_list()
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/base_dataset.py", line 435, in load_data_list
#       annotations = load(self.ann_file)
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/fileio/io.py", line 855, in load
#       with BytesIO(file_backend.get(file)) as f:
#     File "/usr/local/lib/python3.8/dist-packages/mmengine/fileio/backends/local_backend.py", line 33, in get
#       with open(filepath, 'rb') as f:
#   FileNotFoundError: [Errno 2] No such file or directory: 'data/semantickitti/semantickitti_infos_train.10.pkl'
#sol for prob4
# cp /dst/LaserMix/data/sets/semantickitti/semantickitti_infos_train.10.pkl /dst/LaserMix/data/semantickitti/semantickitti_infos_train.10.pkl
# similar...
# cp /dst/LaserMix/data/sets/semantickitti/semantickitti_infos_train.10-unlabeled.pkl /dst/LaserMix/data/semantickitti/semantickitti_infos_train.10-unlabeled.pkl

#prob5
# FileNotFoundError: [Errno 2] No such file or directory: '/data/sets/semantickitti/semantickitti_infos_val.pkl'
#sol for prob5
# mkdir -p /data/sets/semantickitti/ && cp /dst/LaserMix/data/semantickitti/semantickitti_infos_val.pkl /data/sets/semantickitti/semantickitti_infos_val.pkl

#prob6
# /dst/LaserMix/mmdet3d/evaluation/functional/kitti_utils/eval.py:10: NumbaDeprecationWarning: The 'nopython' keyword argument was not supplied to the 'numba.jit' decorator. The implicit default value for this argument is currently False, but it will be changed to True in Numba 0.59.0. See https://numba.readthedocs.io/en/stable/reference/deprecation.html#deprecation-of-object-mode-fall-back-behaviour-when-using-jit for details.
#   def get_thresholds(scores: np.ndarray, num_gt, num_sample_pts=41):
# 01/01 11:48:45 - mmengine - WARNING - The prefix is not set in metric class SegMetric.
# 01/01 11:48:48 - mmengine - WARNING - "FileClient" will be deprecated in future. Please use io functions in https://mmengine.readthedocs.io/en/latest/api/fileio.html#file-io
# 01/01 11:48:48 - mmengine - WARNING - "HardDiskBackend" is the alias of "LocalBackend" and the former will be deprecated in future.
# 01/01 11:48:48 - mmengine - INFO - Checkpoints will be saved to /dst/LaserMix/work_dirs/lasermix_cy3d_semi_semantickitti_10.
# Traceback (most recent call last):
#   File "tools/train.py", line 135, in <module>
#     main()
#   File "tools/train.py", line 131, in main
#     runner.train()
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/runner.py", line 1777, in train
#     model = self.train_loop.run()  # type: ignore
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/loops.py", line 288, in run
#     data_batch = next(self.dataloader_iterator)
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/runner/loops.py", line 167, in __next__
#     data = next(self._iterator)
#   File "/usr/local/lib/python3.8/dist-packages/torch/utils/data/dataloader.py", line 521, in __next__
#     data = self._next_data()
#   File "/usr/local/lib/python3.8/dist-packages/torch/utils/data/dataloader.py", line 1203, in _next_data
#     return self._process_data(data)
#   File "/usr/local/lib/python3.8/dist-packages/torch/utils/data/dataloader.py", line 1229, in _process_data
#     data.reraise()
#   File "/usr/local/lib/python3.8/dist-packages/torch/_utils.py", line 434, in reraise
#     raise exception
# FileNotFoundError: Caught FileNotFoundError in DataLoader worker process 0.
# Original Traceback (most recent call last):
#   File "/usr/local/lib/python3.8/dist-packages/torch/utils/data/_utils/worker.py", line 287, in _worker_loop
#     data = fetcher.fetch(index)
#   File "/usr/local/lib/python3.8/dist-packages/torch/utils/data/_utils/fetch.py", line 49, in fetch
#     data = [self.dataset[idx] for idx in possibly_batched_index]
#   File "/usr/local/lib/python3.8/dist-packages/torch/utils/data/_utils/fetch.py", line 49, in <listcomp>
#     data = [self.dataset[idx] for idx in possibly_batched_index]
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/dataset_wrapper.py", line 171, in __getitem__
#     return self.datasets[dataset_idx][sample_idx]
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/base_dataset.py", line 410, in __getitem__
#     data = self.prepare_data(idx)
#   File "/dst/LaserMix/mmdet3d/datasets/seg3d_dataset.py", line 303, in prepare_data
#     return self.pipeline(data_info)
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/dataset/base_dataset.py", line 60, in __call__
#     data = t(data)
#   File "/usr/local/lib/python3.8/dist-packages/mmcv/transforms/base.py", line 12, in __call__
#     return self.transform(results)
#   File "/dst/LaserMix/mmdet3d/datasets/transforms/loading.py", line 646, in transform
#     points = self._load_points(pts_file_path)
#   File "/dst/LaserMix/mmdet3d/datasets/transforms/loading.py", line 622, in _load_points
#     pts_bytes = get(pts_filename, backend_args=self.backend_args)
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/fileio/io.py", line 181, in get
#     return backend.get(filepath)
#   File "/usr/local/lib/python3.8/dist-packages/mmengine/fileio/backends/local_backend.py", line 33, in get
#     with open(filepath, 'rb') as f:
# FileNotFoundError: [Errno 2] No such file or directory: 'data/semantickitti/sequences/06/velodyne/000129.bin'
#sol for prob6
# change semantickiit dataset structure

