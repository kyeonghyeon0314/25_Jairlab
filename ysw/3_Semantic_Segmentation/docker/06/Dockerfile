# syntax=docker/dockerfile:1

	# Build
# $ docker build -t cascio99/test:06 .
	# Run
# $ docker run -it --gpus all --mount type=bind,src="$(pwd)",target=/dst cascio99/test:06 bash

# Use an official NVIDIA CUDA base image with Ubuntu 20.04 and CUDA 11.0.3 & cuDNN 8.0.5
FROM nvidia/cuda:11.0.3-cudnn8-devel-ubuntu20.04

# Set environment variables
ENV PYTHON_VERSION=3.7
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

# Add the deadsnakes PPA to get Python 3.7
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update

# Install Python 3.7 and development headers
RUN apt-get install -y --no-install-recommends \
        python${PYTHON_VERSION} \
        python${PYTHON_VERSION}-dev \
        python${PYTHON_VERSION}-distutils \
        python3-pip

# Update alternatives to use Python 3.7 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHON_VERSION} 1 && \
    update-alternatives --config python3

# Upgrade pip and setuptools
RUN pip install --upgrade pip setuptools wheel

# Ensure 'python' command is available via python-is-python3(ft. need for 'bash setup.sh')
RUN apt-get install -y python-is-python3

# Install specific PyTorch version with CUDA 11.0 support
RUN pip install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

ENV CUDAVER=cuda-11.0
ENV PATH=/usr/local/$CUDAVER/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/$CUDAVER/lib:$LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/usr/local/$CUDAVER/lib64:$LD_LIBRARY_PATH
ENV CUDA_PATH=/usr/local/$CUDAVER
ENV CUDA_ROOT=/usr/local/$CUDAVER
ENV CUDA_HOME=/usr/local/$CUDAVER
ENV CUDA_HOST_COMPILER=/usr/bin/gcc-9.4

# Clean up temporary files to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Set working directory inside the container
WORKDIR /dst

###############################################################################################
					# TODO inside the container
# apt-get update

	# Basic & Advanced Installation

# cd /dst
# git clone https://github.com/jialeli1/lidarseg3d.git
# cd lidarseg3d
# pip install -r requirements.txt
	# Optional
	# apt-get install vim
# echo 'export PYTHONPATH="${PYTHONPATH}:/dst/lidarseg3d"' >> ~/.bashrc && source ~/.bashrc

# pip install nuscenes-devkit==1.1.6
# echo 'export PYTHONPATH="${PYTHONPATH}:/dst/lidarseg3d/nuscenes-devkit/python-sdk"' >> ~/.bashrc && source ~/.bashrc

# bash setup.sh

# pip install mmcv-full -f https://download.openmmlab.com/mmcv/dist/cu110/torch1.7.1/index.html

# apt-get update
# apt-get install libboost-all-dev
# cd /dst
# git clone -b v1.2.1 https://github.com/traveller59/spconv.git --recursive
# cd spconv/
####################################### flag #########################
# docker commit peaceful_mclean cascio99/test:06


# python setup.py bdist_wheel
# cd ./dist && pip install *

# ********************
	# Prepare data for SemanticKitti
# cd /dst/lidarseg3d && mkdir -p data/SemanticKITTI/
# $ docker cp dataset $(container_name):/dst/lidarseg3d/data/SemanticKITTI/
# $ docker cp workdirs $(container_name):/dst/lidarseg3d/

	# Train & Evaluation
# cd /dst/lidarseg3d/
# python -m torch.distributed.launch --nproc_per_node=1 ./tools/train.py configs/semantickitti/MSeg3D/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py
	# troubleshooting
	# 1. apt-get install python3.7-tk
	# 2. Have some problems...
        
        # ...
        #   File "/dst/lidarseg3d/det3d/datasets/pipelines/loading.py", line 309, in __call__
        #     calib = read_calib_semanticKITTI(calib_path)
        #   File "/dst/lidarseg3d/det3d/datasets/pipelines/loading.py", line 61, in read_calib_semanticKITTI
        #     calib_out['Tr'][:3, :4] = calib_all['Tr'].reshape(3, 4)
        # KeyError: 'Tr'
        # ...
        # subprocess.CalledProcessError: Command '['/usr/bin/python', '-u', './tools/train.py', '--local_rank=0', 'configs/semantickitti/MSeg3D/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py']' returned non-zero exit status 1.
        
    # You should download calib.txt from data_odometry_calib.zip.
    # calib.txt @ data_odometry_color.zip from KITTI(https://www.cvlibs.net/datasets/kitti/eval_odometry.php) doesn't has "Tr: ..." line

    # 3. raise your shared memory limit?
    # 2024-12-27 07:07:32,330   INFO  Start running, host: root@a3b0d5dcc0f7, work_dir: /dst/lidarseg3d/work_dirs/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12
    # 2024-12-27 07:07:32,330   INFO  workflow: [('train', 1)], max: 12 epochs
    # ==> stage 1 frozen for HRNet
    # ==> stage 2 frozen for HRNet
    # ==> stage 3 frozen for HRNet
    # ERROR: Unexpected bus error encountered in worker. This might be caused by insufficient shared memory (shm).
    # Traceback (most recent call last):
    # File "/usr/lib/python3.7/multiprocessing/queues.py", line 236, in _feed
    #     obj = _ForkingPickler.dumps(obj)
    # File "/usr/lib/python3.7/multiprocessing/reduction.py", line 51, in dumps
    #     cls(buf, protocol).dump(obj)
    # File "/usr/local/lib/python3.7/dist-packages/torch/multiprocessing/reductions.py", line 321, in reduce_storage
    #     fd, size = storage._share_fd_()
    # RuntimeError: unable to write to file </torch_1656_4123472588>
    # Traceback (most recent call last):
    # File "/usr/lib/python3.7/multiprocessing/queues.py", line 236, in _feed
    #     obj = _ForkingPickler.dumps(obj)
    # File "/usr/lib/python3.7/multiprocessing/reduction.py", line 51, in dumps
    #     cls(buf, protocol).dump(obj)
    # File "/usr/local/lib/python3.7/dist-packages/torch/multiprocessing/reductions.py", line 321, in reduce_storage
    #     fd, size = storage._share_fd_()
    # RuntimeError: unable to write to file </torch_1652_3512622956>
    # ERROR: Unexpected bus error encountered in worker. This might be caused by insufficient shared memory (shm).
    # ERROR: Unexpected bus error encountered in worker. This might be caused by insufficient shared memory (shm).
    # Traceback (most recent call last):
    # File "/usr/lib/python3.7/multiprocessing/queues.py", line 236, in _feed
    #     obj = _ForkingPickler.dumps(obj)
    # File "/usr/lib/python3.7/multiprocessing/reduction.py", line 51, in dumps
    #     cls(buf, protocol).dump(obj)
    # File "/usr/local/lib/python3.7/dist-packages/torch/multiprocessing/reductions.py", line 321, in reduce_storage
    #     fd, size = storage._share_fd_()
    # RuntimeError: unable to write to file </torch_1652_3878146076>
    # Traceback (most recent call last):
    # File "./tools/train.py", line 171, in <module>
    #     main()
    # File "./tools/train.py", line 166, in main
    #     sync_bn_type=sync_bn_type,
    # File "/dst/lidarseg3d/det3d/torchie/apis/train.py", line 393, in train_detector
    #     trainer.run(data_loaders, cfg.workflow, cfg.total_epochs, local_rank=cfg.local_rank)
    # File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 555, in run
    #     epoch_runner(data_loaders[i], self.epoch, **kwargs)
    # File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 419, in train
    #     self.model, data_batch, train_mode=True, **kwargs
    # File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 377, in batch_processor_inline
    #     losses = model(example, return_loss=True)
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
    #     result = self.forward(*input, **kwargs)
    # File "/dst/lidarseg3d/det3d/models/detectors/seg_mseg3d_net.py", line 106, in forward
    #     data = self.backbone(data)
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
    #     result = self.forward(*input, **kwargs)
    # File "/dst/lidarseg3d/det3d/models/backbones/scn_unet.py", line 229, in forward
    #     x_up2 = self.UR_block_forward(x_conv2, x_up3, self.conv_up_t2, self.conv_up_m2, self.inv_conv2)
    # File "/dst/lidarseg3d/det3d/models/backbones/scn_unet.py", line 167, in UR_block_forward
    #     x_m = conv_m(x)
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
    #     result = self.forward(*input, **kwargs)
    # File "/usr/local/lib/python3.7/dist-packages/spconv/modules.py", line 134, in forward
    #     input = module(input)
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
    #     result = self.forward(*input, **kwargs)
    # File "/usr/local/lib/python3.7/dist-packages/spconv/conv.py", line 199, in forward
    #     outids.shape[0], self.algo)
    # File "/usr/local/lib/python3.7/dist-packages/spconv/functional.py", line 94, in forward
    #     algo=algo)
    # File "/usr/local/lib/python3.7/dist-packages/spconv/ops.py", line 120, in indice_conv
    #     int(inverse), int(subm), algo)
    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/_utils/signal_handling.py", line 66, in handler
    #     _error_if_any_worker_fails()
    # RuntimeError: DataLoader worker (pid 1655) is killed by signal: Bus error. It is possible that dataloader's workers are out of shared memory. Please try to raise your shared memory limit.
    # Traceback (most recent call last):
    # File "/usr/lib/python3.7/runpy.py", line 193, in _run_module_as_main
    #     "__main__", mod_spec)
    # File "/usr/lib/python3.7/runpy.py", line 85, in _run_code
    #     exec(code, run_globals)
    # File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 260, in <module>
    #     main()
    # File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 256, in main
    #     cmd=cmd)
    # subprocess.CalledProcessError: Command '['/usr/bin/python', '-u', './tools/train.py', '--local_rank=0', 'configs/semantickitti/MSeg3D/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py']' returned non-zero exit status 1.

	# ===================================== 2nd try =====================================
	
    # 2024-12-27 07:14:16,818   INFO  Start running, host: root@54463bab4ac5, work_dir: /dst/lidarseg3d/work_dirs/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12
    # 2024-12-27 07:14:16,818   INFO  workflow: [('train', 1)], max: 12 epochs
    # ==> stage 1 frozen for HRNet
    # ==> stage 2 frozen for HRNet
    # ==> stage 3 frozen for HRNet
    # ERROR: Unexpected bus error encountered in worker. This might be caused by insufficient shared memory (shm).
    # ERROR: Unexpected bus error encountered in worker. This might be caused by insufficient shared memory (shm).
    # Traceback (most recent call last):
    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/dataloader.py", line 872, in _try_get_data
    #     data = self._data_queue.get(timeout=timeout)
    # File "/usr/lib/python3.7/multiprocessing/queues.py", line 104, in get
    #     if not self._poll(timeout):
    # File "/usr/lib/python3.7/multiprocessing/connection.py", line 257, in poll
    #     return self._poll(timeout)
    # File "/usr/lib/python3.7/multiprocessing/connection.py", line 414, in _poll
    #     r = wait([self], timeout)
    # File "/usr/lib/python3.7/multiprocessing/connection.py", line 921, in wait
    #     ready = selector.select(timeout)
    # File "/usr/lib/python3.7/selectors.py", line 415, in select
    #     fd_event_list = self._selector.poll(timeout)
    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/_utils/signal_handling.py", line 66, in handler
    #     _error_if_any_worker_fails()
    # RuntimeError: DataLoader worker (pid 84) is killed by signal: Bus error. It is possible that dataloader's workers are out of shared memory. Please try to raise your shared memory limit.

    # The above exception was the direct cause of the following exception:

    # Traceback (most recent call last):
    # File "./tools/train.py", line 171, in <module>
    #     main()
    # File "./tools/train.py", line 166, in main
    #     sync_bn_type=sync_bn_type,
    # File "/dst/lidarseg3d/det3d/torchie/apis/train.py", line 393, in train_detector
    #     trainer.run(data_loaders, cfg.workflow, cfg.total_epochs, local_rank=cfg.local_rank)
    # File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 555, in run
    #     epoch_runner(data_loaders[i], self.epoch, **kwargs)
    # File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 404, in train
    #     for i, data_batch in enumerate(data_loader)

    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/dataloader.py", line 435, in __next__
    #     data = self._next_data()
    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/dataloader.py", line 1068, in _next_data
    #     idx, data = self._get_data()
    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/dataloader.py", line 1034, in _get_data
    #     success, data = self._try_get_data()
    # File "/usr/local/lib/python3.7/dist-packages/torch/utils/data/dataloader.py", line 885, in _try_get_data
    #     raise RuntimeError('DataLoader worker (pid(s) {}) exited unexpectedly'.format(pids_str)) from e
    # RuntimeError: DataLoader worker (pid(s) 84) exited unexpectedly
    # Traceback (most recent call last):
    # File "/usr/lib/python3.7/runpy.py", line 193, in _run_module_as_main
    #     "__main__", mod_spec)
    # File "/usr/lib/python3.7/runpy.py", line 85, in _run_code
    #     exec(code, run_globals)
    # File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 260, in <module>
    #     main()
    # File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 256, in main
    #     cmd=cmd)
    # subprocess.CalledProcessError: Command '['/usr/bin/python', '-u', './tools/train.py', '--local_rank=0', 'configs/semantickitti/MSeg3D/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py']' returned non-zero exit status 1.

# ref for prob3
#https://github.com/pytorch/pytorch/issues/16417

#1st sol for prob3: NOT SOLVED -> set back to default
# $ docker run -it --gpus all --shm-size=2gb --mount type=bind,src="$(pwd)",target=/dst cascio99/test:06 bash

  #  Traceback (most recent call last):
  #   File "./tools/train.py", line 171, in <module>
  #     main()
  #   File "./tools/train.py", line 166, in main
  #     sync_bn_type=sync_bn_type,
  #   File "/dst/lidarseg3d/det3d/torchie/apis/train.py", line 393, in train_detector
  #     trainer.run(data_loaders, cfg.workflow, cfg.total_epochs, local_rank=cfg.local_rank)
  #   File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 555, in run
  #     epoch_runner(data_loaders[i], self.epoch, **kwargs)
  #   File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 419, in train
  #     self.model, data_batch, train_mode=True, **kwargs
  #   File "/dst/lidarseg3d/det3d/torchie/trainer/trainer.py", line 377, in batch_processor_inline
  #     losses = model(example, return_loss=True)
  #   File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
  #     result = self.forward(*input, **kwargs)
  #   File "/dst/lidarseg3d/det3d/models/detectors/seg_mseg3d_net.py", line 123, in forward
  #     data = self.point_head(batch_dict=data, return_loss=return_loss)
  #   File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
  #     result = self.forward(*input, **kwargs)
  #   File "/dst/lidarseg3d/det3d/models/point_heads/point_seg_mseg3d_head.py", line 364, in forward
  #     batch_size=batch_size,
  #   File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
  #     result = self.forward(*input, **kwargs)
  #   File "/dst/lidarseg3d/det3d/models/point_heads/context_module.py", line 111, in forward
  #     tgt=point_feats, memory=sem_embeddings, batch_idx=batch_idx, batch_size=batch_size,
  #   File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
  #     result = self.forward(*input, **kwargs)
  #   File "/dst/lidarseg3d/det3d/models/point_heads/context_module.py", line 164, in forward
  #     pos=pos, query_pos=query_pos, mem_pos=mem_pos)
  #   File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
  #     result = self.forward(*input, **kwargs)
  #   File "/dst/lidarseg3d/det3d/models/point_heads/context_module.py", line 300, in forward
  #     tgt_key_padding_mask, memory_key_padding_mask, pos, query_pos, mem_pos)
  #   File "/dst/lidarseg3d/det3d/models/point_heads/context_module.py", line 242, in forward_post
  #     batch_size=batch_size,
  #   File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
  #     result = self.forward(*input, **kwargs)
  #   File "/dst/lidarseg3d/det3d/models/point_heads/context_module.py", line 365, in forward
  #     cur_atted = torch.bmm(cur_sim_map, cur_v)
  # RuntimeError: CUDA out of memory. Tried to allocate 48.00 MiB (GPU 0; 5.79 GiB total capacity; 4.44 GiB already allocated; 34.06 MiB free; 4.55 GiB reserved in total by PyTorch)
  # Traceback (most recent call last):
  #   File "/usr/lib/python3.7/runpy.py", line 193, in _run_module_as_main
  #     "__main__", mod_spec)
  #   File "/usr/lib/python3.7/runpy.py", line 85, in _run_code
  #     exec(code, run_globals)
  #   File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 260, in <module>
  #     main()
  #   File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 256, in main
  #     cmd=cmd)
  # subprocess.CalledProcessError: Command '['/usr/bin/python', '-u', './tools/train.py', '--local_rank=0', 'configs/semantickitti/MSeg3D/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py']' returned non-zero exit status 1.

#2nd sol for prob3: NOT SOLVED -> set back to default
# lidarseg3d/det3d/torchie/trainer/trainer.py:line531 in run,
# clear gpu memory every epoch starts?

    # import gc
    # gc.collect()
    # torch.cuda.empty_cache()

#3rd sol for prob3: NOT SOLVED -> set back to default
# remove nn.Sequential layers in point_seg_mseg3d_head.py:PoitSegMSeg3DHead(nn.Module)
# file path: lidarseg3d/det3d/models/point_heads/point_seg_mseg3d_head.py
# Line44~65

    # no apex
    # No Tensorflow
    # No Tensorflow
    # /usr/local/lib/python3.7/dist-packages/mmcv/__init__.py:21: UserWarning: On January 1, 2023, MMCV will release v2.0.0, in which it will remove components related to the training process and add a data transformation module. In addition, it will rename the package names mmcv to mmcv-lite and mmcv-full to mmcv. See https://github.com/open-mmlab/mmcv/blob/master/docs/en/compatibility.md for more details.
    # 'On January 1, 2023, MMCV will release v2.0.0, in which it will remove '
    # No APEX!
    # 2024-12-30 05:47:57,693   INFO  Distributed training: False
    # 2024-12-30 05:47:57,693   INFO  torch.backends.cudnn.benchmark: False
    # /dst/lidarseg3d/det3d/models/img_backbones/hrnet.py:329: UserWarning: DeprecationWarning: pretrained is deprecated, please use "init_cfg" instead
    # warnings.warn('DeprecationWarning: pretrained is deprecated, '
    # ==> Loading pretrained weights for HRNet from ./work_dirs/pretrained_models/hrnetv2_w18-00eb2006.pth
    # ==> Loading checkpoint from OrderedDict
    # ==> Done (loaded 1830/1830)
    # ==> stage 1 frozen for HRNet
    # ==> stage 2 frozen for HRNet
    # ==> stage 3 frozen for HRNet
    # Traceback (most recent call last):
    # File "./tools/train.py", line 171, in <module>
    #     main()
    # File "./tools/train.py", line 139, in main
    #     model = build_detector(cfg.model, train_cfg=cfg.train_cfg, test_cfg=cfg.test_cfg)
    # File "/dst/lidarseg3d/det3d/models/builder.py", line 60, in build_detector
    #     return build(cfg, DETECTORS, dict(train_cfg=train_cfg, test_cfg=test_cfg))
    # File "/dst/lidarseg3d/det3d/models/builder.py", line 24, in build
    #     return build_from_cfg(cfg, registry, default_args)
    # File "/dst/lidarseg3d/det3d/utils/registry.py", line 78, in build_from_cfg
    #     return obj_cls(**args)
    # File "/dst/lidarseg3d/det3d/models/detectors/seg_mseg3d_net.py", line 29, in __init__
    #     self.point_head = builder.build_point_head(point_head)
    # File "/dst/lidarseg3d/det3d/models/builder.py", line 64, in build_point_head
    #     return build(cfg, POINT_HEADS)
    # File "/dst/lidarseg3d/det3d/models/builder.py", line 24, in build
    #     return build_from_cfg(cfg, registry, default_args)
    # File "/dst/lidarseg3d/det3d/utils/registry.py", line 78, in build_from_cfg
    #     return obj_cls(**args)
    # File "/dst/lidarseg3d/det3d/models/point_heads/point_seg_mseg3d_head.py", line 53, in __init__
    #     self.gffm_lidar = F.relu(self.norm_lidar(self.lin_lidar))
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 727, in _call_impl
    #     result = self.forward(*input, **kwargs)
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/batchnorm.py", line 98, in forward
    #     self._check_input_dim(input)
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/batchnorm.py", line 207, in _check_input_dim
    #     if input.dim() != 2 and input.dim() != 3:
    # File "/usr/local/lib/python3.7/dist-packages/torch/nn/modules/module.py", line 779, in __getattr__
    #     type(self).__name__, name))
    # torch.nn.modules.module.ModuleAttributeError: 'Linear' object has no attribute 'dim'
    # Traceback (most recent call last):
    # File "/usr/lib/python3.7/runpy.py", line 193, in _run_module_as_main
    #     "__main__", mod_spec)
    # File "/usr/lib/python3.7/runpy.py", line 85, in _run_code
    #     exec(code, run_globals)
    # File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 260, in <module>
    #     main()
    # File "/usr/local/lib/python3.7/dist-packages/torch/distributed/launch.py", line 256, in main
    #     cmd=cmd)
    # subprocess.CalledProcessError: Command '['/usr/bin/python', '-u', './tools/train.py', '--local_rank=0', 'configs/semantickitti/MSeg3D/semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py']' returned non-zero exit status 1.

#4th sol for prob3: NOT SOLVED -> set back to default
# reduce batch_size in semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py
# line227
# samples_per_gpu=2 --> samples_per_gpu=1

    # RuntimeError: CUDA out of memory. Tried to allocate 78.00 MiB (GPU 0; 5.79 GiB total capacity; 4.40 GiB already allocated; 78.44 MiB free; 4.53 GiB reserved in total by PyTorch)

#5th sol for prob3: NOT SOLVED -> set back to defualt
# optimize data loading in semkitti_avgvfe_unetscn3d_hrnetw18_lr1en2_e12.py
# line228
# samples_per_gpu=8 --> samples_per_gpu=1

    # Similar error...


# STOP searching for MSeg3D, try LaserMix for semanticKitti dataset
# "Seems like MSeg3D uses too much gpu memory for me; RTX2060(6gb)"
# git : https://github.com/ldkong1205/LaserMix
# paper : https://arxiv.org/abs/2405.05258
