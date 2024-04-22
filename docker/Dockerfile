# 一个项目一个Docker，到时候别人也可以复现，哈哈
# 下面就是个示例

# Contains CUDA 11.7.1, PyTorch 1.13.0a0+d321be6, APEX, Tensorboard, Torch-TensorRT etc.
# For 3090 it requires NVIDIA Driver release 515 or later.
# For A100 it requires NVIDIA Driver release 450.51 (or later R450), 470.57 (or later R470), or 510.47 (or later R510).
# For detail info please refer to https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html
FROM nvcr.io/nvidia/pytorch:22.08-py3

# Common tools
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    apt install -y nethogs htop iputils-ping zip unzip rsync rclone
RUN apt clean &&\
    rm -rf /var/cache/apt/*

# Install zsh as main shell
RUN apt install -y zsh && zsh --version && chsh -s /usr/bin/zsh
# Install oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Setup workspace
RUN mkdir -p /workspace
WORKDIR /workspace

# Project requirements
COPY ./requirements.txt ./requirements.txt
RUN pip install -r ./requirements.txt
RUN rm /workspace/requirements.txt

# MMCV
RUN pip install mmcv-full==1.7.0 -f https://download.openmmlab.com/mmcv/dist/cu117/torch1.13/index.html

# For dynamic pillar/voxel encoder
RUN FORCE_CUDA=1 pip install --no-cache-dir --verbose torch-scatter==2.1.0
