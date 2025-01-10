from functools import partial
from spconv.pytorch.conv import SparseConv3d, SubMConv3d
from spconv.pytorch.modules import SparseModule
import spconv.pytorch as spconv
import torch
import torch.nn as nn
import numpy as np
from ..registry import BACKBONES
from det3d.core.utils import common_utils


def post_act_block(in_channels, out_channels, kernel_size, indice_key=None, stride=1, padding=0,
                   conv_type='subm', norm_fn=None):

    if conv_type == 'subm':
        conv = spconv.SubMConv3d(in_channels, out_channels, kernel_size, bias=False, indice_key=indice_key)
    elif conv_type == 'spconv':
        conv = spconv.SparseConv3d(in_channels, out_channels, kernel_size, stride=stride, padding=padding,
                                   bias=False, indice_key=indice_key)
    elif conv_type == 'inverseconv':
        conv = spconv.SparseInverseConv3d(in_channels, out_channels, kernel_size, indice_key=indice_key, bias=False)
    else:
        raise NotImplementedError

    m = spconv.SparseSequential(
        conv,
        norm_fn(out_channels),
        nn.ReLU(),
    )

    return m



class SparseBasicBlock(spconv.modules.SparseModule):
    expansion = 1

    def __init__(self, inplanes, planes, stride=1, downsample=None, indice_key=None, norm_fn=None):
        super(SparseBasicBlock, self).__init__()
        self.conv1 = spconv.SubMConv3d(
            inplanes, planes, kernel_size=3, stride=stride, padding=1, bias=False, indice_key=indice_key
        )
        self.bn1 = norm_fn(planes)
        self.relu = nn.ReLU()
        self.conv2 = spconv.SubMConv3d(
            planes, planes, kernel_size=3, stride=1, padding=1, bias=False, indice_key=indice_key
        )
        self.bn2 = norm_fn(planes)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        #identity = x.features
        identity = x

        assert x.features.dim() == 2, 'x.features.dim()=%d' % x.features.dim()

        out = self.conv1(x)
        out = out.replace_feature(self.bn1(out.features))
        out = out.replace_feature(self.relu(out.features))
        #out.features = self.bn1(out.features)
        #out.features = self.relu(out.features)

        out = self.conv2(out)
        out = out.replace_feature(self.bn2(out.features))
        #out.features = self.bn2(out.features)

        if self.downsample is not None:
            identity = self.downsample(x)

        out = out.replace_feature(out.features + identity.features)
        out = out.replace_feature(self.relu(out.features))
        #out.features += identity
        #out.features = self.relu(out.features)

        return out


@BACKBONES.register_module
class UNetSCN3D(nn.Module):
    """
    Res enc + Res dec
    Sparse Convolution based UNet for point-wise feature learning.
    Reference Paper: https://arxiv.org/abs/1907.03670 (Shaoshuai Shi, et. al)
    From Points to Parts: 3D Object Detection from Point Cloud with Part-aware and Part-aggregation Network
    """
    def __init__(self, num_input_features=128, name="UNetSCN3D", voxel_size=[], point_cloud_range=[], model_cfg={},  **kwargs):
        super().__init__()
        self.model_cfg = model_cfg
        self.voxel_size = voxel_size
        self.point_cloud_range = point_cloud_range

        norm_fn = partial(nn.BatchNorm1d, eps=1e-3, momentum=0.01)

        ratio=model_cfg.get("SCALING_RATIO", 1)
        self.conv_input = spconv.SparseSequential(
            spconv.SubMConv3d(num_input_features, 16*ratio, 3, padding=1, bias=False, indice_key='subm1'),
            norm_fn(16*ratio),
            nn.ReLU(),
        )
        block = post_act_block


        # res enc
        self.conv1 = spconv.SparseSequential(
            SparseBasicBlock(16*ratio, 16*ratio, norm_fn=norm_fn, indice_key='subm1'),
            SparseBasicBlock(16*ratio, 16*ratio, norm_fn=norm_fn, indice_key='subm1'),
        )

        # res enc
        self.conv2 = spconv.SparseSequential(
            # [1600, 1408, 41] <- [800, 704, 21]
            block(16*ratio, 32*ratio, 3, norm_fn=norm_fn, stride=2, padding=1, indice_key='spconv2', conv_type='spconv'),
            SparseBasicBlock(32*ratio, 32*ratio, norm_fn=norm_fn, indice_key='subm2'),
            SparseBasicBlock(32*ratio, 32*ratio, norm_fn=norm_fn, indice_key='subm2'),
        )

        self.conv3 = spconv.SparseSequential(
            # [800, 704, 21] <- [400, 352, 11]
            block(32*ratio, 64*ratio, 3, norm_fn=norm_fn, stride=2, padding=1, indice_key='spconv3', conv_type='spconv'),
            SparseBasicBlock(64*ratio, 64*ratio, norm_fn=norm_fn, indice_key='subm3'),
            SparseBasicBlock(64*ratio, 64*ratio, norm_fn=norm_fn, indice_key='subm3'),
        )

        self.conv4 = spconv.SparseSequential(
            # [400, 352, 11] <- [200, 176, 5]
            block(64*ratio, 64*ratio, 3, norm_fn=norm_fn, stride=2, padding=(0, 1, 1), indice_key='spconv4', conv_type='spconv'),
            SparseBasicBlock(64*ratio, 64*ratio, norm_fn=norm_fn, indice_key='subm4'),
            SparseBasicBlock(64*ratio, 64*ratio, norm_fn=norm_fn, indice_key='subm4'),
        )

        if self.model_cfg.get('RETURN_ENCODED_TENSOR', True):
            last_pad = self.model_cfg.get('last_pad', 0)

            self.conv_out = spconv.SparseSequential(
                # [200, 150, 5] -> [200, 150, 2]
                spconv.SparseConv3d(64*ratio, 128, (3, 1, 1), stride=(2, 1, 1), padding=last_pad,
                                    bias=False, indice_key='spconv_down2'),
                norm_fn(128),
                nn.ReLU(),
            )
        else:
            self.conv_out = None

        # decoder
        # [400, 352, 11] <- [200, 176, 5]
        self.conv_up_t4 = SparseBasicBlock(64*ratio, 64*ratio, indice_key='subm4', norm_fn=norm_fn)
        self.conv_up_m4 = block(128*ratio, 64*ratio, 3, norm_fn=norm_fn, padding=1, indice_key='subm4')
        self.inv_conv4 = block(64*ratio, 64*ratio, 3, norm_fn=norm_fn, indice_key='spconv4', conv_type='inverseconv')

        # [800, 704, 21] <- [400, 352, 11]
        self.conv_up_t3 = SparseBasicBlock(64*ratio, 64*ratio, indice_key='subm3', norm_fn=norm_fn)
        self.conv_up_m3 = block(128*ratio, 64*ratio, 3, norm_fn=norm_fn, padding=1, indice_key='subm3')
        self.inv_conv3 = block(64*ratio, 32*ratio, 3, norm_fn=norm_fn, indice_key='spconv3', conv_type='inverseconv')

        # [1600, 1408, 41] <- [800, 704, 21]
        self.conv_up_t2 = SparseBasicBlock(32*ratio, 32*ratio, indice_key='subm2', norm_fn=norm_fn)
        self.conv_up_m2 = block(64*ratio, 32*ratio, 3, norm_fn=norm_fn, indice_key='subm2')
        self.inv_conv2 = block(32*ratio, 16*ratio, 3, norm_fn=norm_fn, indice_key='spconv2', conv_type='inverseconv')

        # [1600, 1408, 41] <- [1600, 1408, 41]
        self.conv_up_t1 = SparseBasicBlock(16*ratio, 16*ratio, indice_key='subm1', norm_fn=norm_fn)
        self.conv_up_m1 = block(32*ratio, 16*ratio, 3, norm_fn=norm_fn, indice_key='subm1')

        self.conv5 = spconv.SparseSequential(
            block(16*ratio, 16*ratio, 3, norm_fn=norm_fn, padding=1, indice_key='subm1')
        )
        self.num_point_features = 16*ratio

    def UR_block_forward(self, x_lateral, x_bottom, conv_t, conv_m, conv_inv):
        x_trans = conv_t(x_lateral)
        x = x_trans.replace_feature(torch.cat((x_bottom.features, x_trans.features), dim=1))
        #x.features = torch.cat((x_bottom.features, x_trans.features), dim=1)
        x_m = conv_m(x)
        x = self.channel_reduction(x, x_m.features.shape[1])
        x = x.replace_feature(x_m.features + x.features)
        #x.features = x_m.features + x.features
        x = conv_inv(x)
        return x

    @staticmethod
    def channel_reduction(x, out_channels):
        """
        Args:
            x: x.features (N, C1)
            out_channels: C2

        Returns:

        """
        features = x.features
        n, in_channels = features.shape
        assert (in_channels % out_channels == 0) and (in_channels >= out_channels)
        reduced_features = features.view(n, out_channels, -1).sum(dim=2)
        #x.features = features.view(n, out_channels, -1).sum(dim=2)
        return x.replace_feature(reduced_features)

    def forward(self, batch_dict):
        """
        Args:
            batch_dict:
                batch_size: int
                voxel_features: (num_voxels, C)
                voxel_coords: (num_voxels, 4), [batch_idx, z_idx, y_idx, x_idx]
        Returns:
            batch_dict:
                encoded_spconv_tensor: sparse tensor
                point_features: (N, C)
        """
        voxel_features, voxel_coords = batch_dict['voxel_features'], batch_dict['voxel_coords']
        batch_size = batch_dict['batch_size']
        sparse_shape = np.array(batch_dict['input_shape'][::-1]) + [1, 0, 0]

        input_sp_tensor = spconv.SparseConvTensor(
            features=voxel_features,
            indices=voxel_coords.int(),
            spatial_shape=sparse_shape,
            batch_size=batch_size
        )
        x = self.conv_input(input_sp_tensor)

        x_conv1 = self.conv1(x)
        x_conv2 = self.conv2(x_conv1)
        x_conv3 = self.conv3(x_conv2)
        x_conv4 = self.conv4(x_conv3)

        if self.conv_out is not None:
            # for detection head only 
            out = self.conv_out(x_conv4)
            batch_dict['encoded_spconv_tensor'] = out
            batch_dict['encoded_spconv_tensor_stride'] = 8

        # [400, 352, 11] <- [200, 176, 5]
        x_up4 = self.UR_block_forward(x_conv4, x_conv4, self.conv_up_t4, self.conv_up_m4, self.inv_conv4)
        # [800, 704, 21] <- [400, 352, 11]
        x_up3 = self.UR_block_forward(x_conv3, x_up4, self.conv_up_t3, self.conv_up_m3, self.inv_conv3)
        # [1600, 1408, 41] <- [800, 704, 21]
        x_up2 = self.UR_block_forward(x_conv2, x_up3, self.conv_up_t2, self.conv_up_m2, self.inv_conv2)
        # [1600, 1408, 41] <- [1600, 1408, 41]
        x_up1 = self.UR_block_forward(x_conv1, x_up2, self.conv_up_t1, self.conv_up_m1, self.conv5)

        batch_dict.update({
            'multi_scale_3d_features': {
                'x_conv1': x_up2,
                'x_conv2': x_up3,
                'x_conv3': x_up4,
                'x_conv4': x_conv4,
            }
        })

        batch_dict['conv_point_features'] = x_up1.features # for seg
        point_coords = common_utils.get_voxel_centers(
            x_up1.indices[:, 1:], downsample_times=1, voxel_size=self.voxel_size,
            point_cloud_range=self.point_cloud_range
        )
        batch_dict['conv_point_coords'] = torch.cat((x_up1.indices[:, 0:1].float(), point_coords), dim=1)

        return batch_dict

