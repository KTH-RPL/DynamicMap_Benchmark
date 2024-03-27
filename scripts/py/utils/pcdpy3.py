'''
# Created: 2023-04-12 15:37
# Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# Author: Kin ZHANG  (https://kin-zhang.github.io/)
# Part of Dynamic Benchmark. Running in Python3.
# Version: 1.0.0 (2023-04-12) Only read.
# Version: 2.0.0 (2023-05-21) Add write with intensity involved.


# This code is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# ---------------------------------------------------------------
# First version: Read and write PCL .pcd files in python.
# dimatura@cmu.edu, 2013-2018 Check https://github.com/dimatura/pypcd
'''

import re
import warnings
import copy
import numpy as np

def load_pcd(path):
    data = PointCloud.from_path(path)
    data.xyzi2np()
    return data

def save_pcd(path, data: np.array, pos=np.array([0,0,0,1,0,0,0]), rgb: np.array = None):
    # need be data: [N,3] or [N,4] [x,y,z,intensity]
    # pos: [x,y,z, qw,qx,qy,qz]
    # assert with print
    assert len(pos) == 7 , "we must have our pose format [x,y,z, qw,qx,qy,qz] in np.array"
    with open(path, 'wb') as fileobj:
        md = {'version': .7,
            'fields': ['x', 'y', 'z'],
            'size': [4, 4, 4],
            'type': ['F', 'F', 'F'],
            'count': [1, 1, 1],
            'width': data.shape[0],
            'height': 1,
            # The viewpoint information is specified as a translation (tx ty tz) + quaternion (qw qx qy qz). The default value is:
            'viewpoint': [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]],
            'points': data.shape[0],
            'data': 'binary'}
        pc_data, pc = None, None

        data_float32 = np.array(data, dtype=np.float32)

        if(data.shape[1] == 4):
            md['fields'].append('intensity')
            md['size'].append(4)
            md['type'].append('F')
            md['count'].append(1)
            pc_data = np.zeros(data_float32.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])
            pc_data['x'] = data_float32[:, 0]
            pc_data['y'] = data_float32[:, 1]
            pc_data['z'] = data_float32[:, 2]
            pc_data['intensity'] = data_float32[:, 3]

        elif(data.shape[1] == 3 and rgb is not None and data.shape[0] == rgb.shape[0]):
            md['fields'].append('rgb')
            md['size'].append(4)
            md['type'].append('F')
            md['count'].append(1)
            pc_data = np.zeros(data_float32.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
            pc_data['x'] = data_float32[:, 0]
            pc_data['y'] = data_float32[:, 1]
            pc_data['z'] = data_float32[:, 2]
            rgb = rgb.astype(np.uint32)
            pc_data['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]

        elif(data.shape[1] == 3):
            pc_data = np.zeros(data_float32.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            pc_data['x'] = data_float32[:, 0]
            pc_data['y'] = data_float32[:, 1]
            pc_data['z'] = data_float32[:, 2]
        
        if pc_data is not None:
            pc = PointCloud(md, pc_data)
            point_cloud_to_fileobj(pc, fileobj)
        else:
            raise ValueError('data shape error, need be [N,3] or [N,4]')
        
class PointCloud(object):
    def __init__(self, metadata, pc_data):
        self.metadata_keys = metadata.keys()
        self.__dict__.update(metadata)
        self.pc_data = pc_data

    def get_metadata(self):
        """ returns copy of metadata """
        metadata = {}
        for k in self.metadata_keys:
            metadata[k] = copy.copy(getattr(self, k))
        return metadata

    def xyzi2np(self):
        x = np.reshape(self.pc_data['x'], (-1, 1))
        y = np.reshape(self.pc_data['y'], (-1, 1))
        z = np.reshape(self.pc_data['z'], (-1, 1))
        if 'intensity' in self.pc_data.dtype.names:
            i = np.reshape(self.pc_data['intensity'], (-1, 1))
            self.np_data = np.concatenate((x, y, z, i), axis=1)
        else:
            self.np_data = np.concatenate((x, y, z), axis=1)

    @staticmethod
    def from_path(filename):
        # note need binary read!!! which is 'rb'
        with open(filename, 'rb') as f:
            pc = point_cloud_from_fileobj(f)
        return pc
    
# Data Type Start --------------------------------------------------------
numpy_pcd_type_mappings = [(np.dtype('float32'), ('F', 4)),
                           (np.dtype('float64'), ('F', 8)),
                           (np.dtype('uint8'), ('U', 1)),
                           (np.dtype('uint16'), ('U', 2)),
                           (np.dtype('uint32'), ('U', 4)),
                           (np.dtype('uint64'), ('U', 8)),
                           (np.dtype('int16'), ('I', 2)),
                           (np.dtype('int32'), ('I', 4)),
                           (np.dtype('int64'), ('I', 8))]
numpy_type_to_pcd_type = dict(numpy_pcd_type_mappings)
pcd_type_to_numpy_type = dict((q, p) for (p, q) in numpy_pcd_type_mappings)

def _build_dtype(metadata):
    dtypes = []
    for f, c, t, s in zip(metadata['fields'],
                          metadata['count'],
                          metadata['type'],
                          metadata['size']):
        np_type = pcd_type_to_numpy_type[(t, s)]

        if c == 1:
            dtypes.append((f, np_type))
        else:
            sub_dtype = (f, np_type, c)
            dtypes.append(sub_dtype)
    return np.dtype(dtypes)
# Data Type End --------------------------------------------------------

# Parse Point Data Start -----------------------------------------------
def parse_ascii_pc_data(f, dtype):
    return np.loadtxt(f, dtype=dtype, delimiter=' ')

def parse_binary_pc_data(f, dtype, metadata):
    rowstep = metadata['points']*dtype.itemsize
    # for some reason pcl adds empty space at the end of files
    buf = f.read(rowstep)
    return np.fromstring(buf, dtype=dtype)
# Parse Point Data End -------------------------------------------------

    
def parse_header(lines):
    """ Parse header of PCD files.
    """
    metadata = {}
    for ln in lines:
        if ln.startswith('#') or len(ln) < 2:
            continue
        match = re.match(r'(\w+)\s+([-\w\s\.]+)', ln)
        if not match:
            warnings.warn("warning: can't understand line: %s" % ln)
            continue
        key, value = match.group(1).lower(), match.group(2)
        if key == 'version':
            metadata[key] = value
        elif key in ('fields', 'type'):
            metadata[key] = value.split()
        elif key in ('size', 'count'):
            metadata[key] = map(int, value.split())
        elif key in ('width', 'height', 'points'):
            metadata[key] = int(value)
        elif key == 'viewpoint':
            metadata[key] = map(float, value.split())
        elif key == 'data':
            metadata[key] = value.strip().lower()
    # add some reasonable by defaults is not exist
    if 'count' not in metadata:
        metadata['count'] = [1]*len(metadata['fields'])
    if 'viewpoint' not in metadata:
        metadata['viewpoint'] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    if 'version' not in metadata:
        metadata['version'] = '.7'
    return metadata

def point_cloud_from_fileobj(f):
    """ Parse pointcloud coming from file object f
    """
    header = []

    while True:
        ln = (f.readline().strip()).decode("utf-8")
        header.append(ln)
        if ln.startswith('DATA'):
            metadata = parse_header(header)
            metadata_temp = copy.deepcopy(metadata)
            dtype = _build_dtype(metadata_temp)
            break

    if metadata['data'] == 'ascii':
        pc_data = parse_ascii_pc_data(f, dtype)
    elif metadata['data'] == 'binary':
        pc_data = parse_binary_pc_data(f, dtype, metadata)
    else:
        print('DATA field is neither "ascii" or "binary" Please save the pcd as binary or ascii')
    return PointCloud(metadata, pc_data)

def write_header(metadata, rename_padding=False):
    """ Given metadata as dictionary, return a string header.
    """
    template = """\
VERSION {version}
FIELDS {fields}
SIZE {size}
TYPE {type}
COUNT {count}
WIDTH {width}
HEIGHT {height}
VIEWPOINT {viewpoint}
POINTS {points}
DATA {data}
"""
    str_metadata = metadata.copy()

    if not rename_padding:
        str_metadata['fields'] = ' '.join(metadata['fields'])
    else:
        new_fields = []
        for f in metadata['fields']:
            if f == '_':
                new_fields.append('padding')
            else:
                new_fields.append(f)
        str_metadata['fields'] = ' '.join(new_fields)
    str_metadata['size'] = ' '.join(map(str, metadata['size']))
    str_metadata['type'] = ' '.join(metadata['type'])
    str_metadata['count'] = ' '.join(map(str, metadata['count']))
    str_metadata['width'] = str(metadata['width'])
    str_metadata['height'] = str(metadata['height'])
    str_metadata['viewpoint'] = ' '.join(map(str, metadata['viewpoint']))
    str_metadata['points'] = str(metadata['points'])
    tmpl = template.format(**str_metadata)
    return tmpl

def point_cloud_to_fileobj(pc, fileobj, data_compression=None):
    """ Write pointcloud as .pcd to fileobj.
    If data_compression is not None it overrides pc.data.
    """
    metadata = pc.get_metadata()
    if data_compression is not None:
        data_compression = data_compression.lower()
        assert(data_compression in ('ascii', 'binary'))
        metadata['data'] = data_compression

    header = write_header(metadata)
    fileobj.write(header.encode('utf-8'))
    if metadata['data'].lower() == 'ascii':
        fmtstr = build_ascii_fmtstr(pc)
        np.savetxt(fileobj, pc.pc_data, fmt=fmtstr)
    elif metadata['data'].lower() == 'binary':
        fileobj.write(pc.pc_data.tostring('C'))
    else:
        raise ValueError('unknown DATA type')
    
def build_ascii_fmtstr(pc):
    """ Make a format string for printing to ascii.
    Note %.8f is minimum for rgb.
    """
    fmtstr = []
    for t, cnt in zip(pc.type, pc.count):
        if t == 'F':
            fmtstr.extend(['%.10f']*cnt)
        elif t == 'I':
            fmtstr.extend(['%d']*cnt)
        elif t == 'U':
            fmtstr.extend(['%u']*cnt)
        else:
            raise ValueError("don't know about type %s" % t)
    return fmtstr


# Other functions --------------------------------------------------------
try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    warnings.warn("scipy not found, some functions may not work")

def xyzqwxyz_to_matrix(xyzqwxyz: list):
    """
    input: xyzqwxyz: [x, y, z, qx, qy, qz, qw] a list of 7 elements
    """
    rotation = R.from_quat([xyzqwxyz[4], xyzqwxyz[5], xyzqwxyz[6], xyzqwxyz[3]]).as_matrix()
    pose = np.eye(4).astype(np.float64)
    pose[:3, :3] = rotation
    pose[:3, 3] = xyzqwxyz[:3]
    return pose
