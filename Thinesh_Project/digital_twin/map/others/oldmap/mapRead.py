# -*- coding: utf-8 -*-
from __future__ import print_function

import cv2
import time
import os
import operator
import numpy as np
import argparse
from PIL import Image

__author__ = 'zj'

image_formats = ['jpg', 'JPG', 'jpeg', 'JPEG', 'png', 'PNG']


def is_pgm_file(in_path):
    print(in_path)
    if not os.path.isfile(in_path):
        return False
    if in_path is not str and not in_path.endswith('.pgm'):
        return False
    return True


def convert_pgm_P5(in_path, out_path):
    """
         Convert pgm files to other image formats
         Read the binary file, read the magic number first, then read the width and height, and the maximum value
         :param in_path: Enter the path to the pgm file
         :param out_path: output file path
    """
    print("convert_pgm_P5")

    with open(in_path, 'rb') as f:
                 # Read two bytes - magic number and decode it into a string
        magic_number = f.readline().strip().decode('utf-8')
        print(magic_number)

                 # 
        width, height = f.readline().strip().decode('utf-8').split(' ')
        width = int(width)
        height = int(height)
        print(width, height )
                 # 
        maxval = f.readline().strip()
                 # Bytes of gray value read each time
        print(maxval)
        if int(maxval) < 256:
            one_reading = 1
        else:
            one_reading = 2
                 # Create a blank image with size (row, column) = (height, width)
        print(one_reading)
        img = np.zeros((height, width))
        img[:, :] = [[ord(f.read(one_reading)) for j in range(width)] for i in range(height)]
        for ii in img:
            print(ii)
        cv2.imwrite(out_path, img)
        print('%s save ok' % out_path)


def convert_pgm_P5_batch(in_dir, out_dir, res_format):
    """
         Batch convert PGM files
         :param in_dir: pgm folder path
         :param out_dir: output folder path
         :param res_format: result image format
    """

    file_list = os.listdir(in_dir)
    for file_name in file_list:
        file_path = os.path.join(in_dir, file_name)
                 #if the pgm file path, then format it
        if is_pgm_file(file_path):
            file_out_path = os.path.join(out_dir, os.path.basename(file_name) + '.' + res_format)
            convert_pgm_P5(file_path, file_out_path)
                 #If it is a directory, create a new result file directory, recursive processing
        elif os.path.isdir(file_path):
            file_out_dir = os.path.join(out_dir, file_name)
            if not os.path.exists(file_out_dir):
                os.mkdir(file_out_dir)
            convert_pgm_P5_batch(file_path, file_out_dir, res_format)
        else:
            pass
    print('batch operation over')


if __name__ == '__main__':
    script_start_time = time.time()

    # print(args)
    in_path =  os.path.join(os.getcwd(),"sample_map.pgm")#3args['input']
    out_path = os.path.join(os.getcwd(),"data.png")#args['output']


    if in_path is not None and out_path is not None:
                 # Convert a single pgm file format
        convert_pgm_P5(in_path, out_path)
        # convert_pgm_by_PIL(in_path, out_path)
    elif isbatch:
                 # Batch conversion
        convert_pgm_P5_batch(in_dir, out_dir, res_format)
    else:
    	Print('Please enter the corresponding parameter')

    print('Script took %s seconds.' % (time.time() - script_start_time,))

