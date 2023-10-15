#!/usr/bin/env python3
import numpy as np
import rospy, cv2
import torch
import os, sys

import argparse
import time
import torch.nn as nn

import signal

from std_msgs.msg import Float32MultiArray        # See https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

from agents.image_attack_agent import ImageAttacker
from setting_params import FREQ_MID_LEVEL, SETTING

import glob

from fastdvdnet.utils import batch_psnr, init_logger_test, \
				variable_to_cv2_image, remove_dataparallel_wrapper, open_sequence_denoiser, close_logger

from basicsr.archs.rrdbnet_arch import RRDBNet
from basicsr.utils.download_util import load_file_from_url

from tangoesrgan import TangoESRGANer

from tangoesrgan.archs.discriminator_arch import UNetDiscriminatorSN
from tangoesrgan.archs.srvgg_arch import SRVGGNetCompact


IMAGE_CLEAN_RECEIVED = None
def fnc_clean_img_callback(msg):
    global IMAGE_CLEAN_RECEIVED
    IMAGE_CLEAN_RECEIVED = msg

IMAGE_RECEIVED = None
def fnc_img_callback(msg):
    global IMAGE_RECEIVED
    IMAGE_RECEIVED = msg

ADV_IMAGE_RECEIVED = None
def fnc_adv_img_callback(msg):
    global ADV_IMAGE_RECEIVED
    ADV_IMAGE_RECEIVED = msg

def get_args():
    """ Get arguments for individual tb3 deployment. """
    parser = argparse.ArgumentParser(
        description="Denoise a sequence with FPRN"
    )
    parser.add_argument('-i', '--input', type=str, default='inputs', help='Input image or folder')
    parser.add_argument(
        '-n',
        '--model_name',
        type=str,
        default='tangoesrgan-general-x4v3',
        help=('Model names: TANGOESRGAN_x4plus | RealESRNet_x4plus | RealESRGAN_x4plus_anime_6B | RealESRGAN_x2plus | '
              'realesr-animevideov3 | realesr-general-x4v3'))
    parser.add_argument('-o', '--output', type=str, default='results', help='Output folder')
    parser.add_argument(
        '-dn',
        '--denoise_strength',
        type=float,
        default=0.5,
        help=('Denoise strength. 0 for weak denoise (keep noise), 1 for strong denoise ability. '
              'Only used for the realesr-general-x4v3 model'))
    parser.add_argument('-s', '--outscale', type=float, default=1, help='The final upsampling scale of the image')
    parser.add_argument(
        '--model_path', type=str, default=None, help='[Option] Model path. Usually, you do not need to specify it')
    parser.add_argument('--suffix', type=str, default='out', help='Suffix of the restored image')
    parser.add_argument('-t', '--tile', type=int, default=0, help='Tile size, 0 for no tile during testing')
    parser.add_argument('--tile_pad', type=int, default=10, help='Tile padding')
    parser.add_argument('--pre_pad', type=int, default=0, help='Pre padding size at each border')
    parser.add_argument('--face_enhance', action='store_true', help='Use GFPGAN to enhance face')
    parser.add_argument(
        '--fp32', action='store_true', help='Use fp32 precision during inference. Default: fp16 (half precision).')
    parser.add_argument(
        '--alpha_upsampler',
        type=str,
        default='realesrgan',
        help='The upsampler for the alpha channels. Options: realesrgan | bicubic')
    parser.add_argument(
        '--ext',
        type=str,
        default='auto',
        help='Image extension. Options: auto | jpg | png, auto means using the same extension as inputs')
    parser.add_argument(
        '-g', '--gpu-id', type=int, default=None, help='gpu device to use (default=None) can be 0,1,2 for multi-gpu')
    parser.add_argument("--gray", 
                        action='store_true',
						help='perform denoising of grayscale images instead of RGB')
    parser.add_argument("--max_num_fr_per_seq", 
                        type=int, 
                        default=5,
					    help='max number of frames to load per sequence')
    parser.add_argument("--save_path", 
                        type=str, 
                        default='/home/haotiangu/catkin_ws/src/tcps_image_attack/scripts/result', 
					    help='where to save outputs as png')

    return parser.parse_known_args(sys.argv)


def convert2torch(obs):
    obs = np.expand_dims(obs, 0)
    ### Generate Attacked Image ###
    image_torch = torch.FloatTensor(obs).permute(0, 3, 1, 2)#<--- To avoid MIXED MEMORY 
    """
    X: minibatch image    [(1 x 3 x 448 x 448), ...]
    """  
    return image_torch

def calc_avg_mean_std(adv_images, size):
    mean_sum = np.array([0., 0., 0.])
    std_sum = np.array([0., 0., 0.])
    n_images = len(adv_images)
    for i in adv_images:
        img = i.transpose(1,2,0)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mean, std = cv2.meanStdDev(img)
        mean_sum += np.squeeze(mean)
        std_sum += np.squeeze(std)
    return (mean_sum / n_images, std_sum / n_images)



if __name__ == '__main__':

    # rosnode node initialization
    rospy.init_node('tangoesrgan_node')
    print('tangoesrgan_node is initialized at', os.getcwd())
    start_time = time.time()
    args, unknown = get_args()

    if not os.path.exists(args.save_path):
		   os.makedirs(args.save_path)
    logger = init_logger_test(args.save_path)

    # determine models according to model names
    args.model_name = args.model_name.split('.')[0]
    if args.model_name == 'TANGOESRGAN_x4plus':  # x4 RRDBNet model
        model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=5, num_grow_ch=32, scale=4)
        netscale = 4
        file_url = ['https://github.com/haotiangu/FPRN/releases/download/FPRN/net_g_392.pth']
    elif args.model_name == 'RealESRGAN_x4plus_anime_6B':  # x4 RRDBNet model with 6 blocks
        model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=6, num_grow_ch=32, scale=4)
        netscale = 4
        file_url = ['https://github.com/xinntao/Real-ESRGAN/releases/download/v0.2.2.4/RealESRGAN_x4plus_anime_6B.pth']
    elif args.model_name == 'RealESRGAN_x2plus':  # x2 RRDBNet model
        model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=23, num_grow_ch=32, scale=2)
        netscale = 2
        file_url = ['https://github.com/xinntao/Real-ESRGAN/releases/download/v0.2.1/RealESRGAN_x2plus.pth']
    elif args.model_name == 'realesr-animevideov3':  # x4 VGG-style model (XS size)
        model = SRVGGNetCompact(num_in_ch=3, num_out_ch=3, num_feat=64, num_conv=16, upscale=4, act_type='prelu')
        netscale = 4
        file_url = ['https://github.com/xinntao/Real-ESRGAN/releases/download/v0.2.5.0/realesr-animevideov3.pth']
    elif args.model_name == 'tangoesrgan-general-x4v3':  # x4 VGG-style model (S size)
        model = SRVGGNetCompact(num_in_ch=3, num_out_ch=3, num_feat=64, num_conv=12, upscale=4, act_type='prelu')
        netscale = 4
        file_url = ['https://github.com/haotiangu/TANGO_ESRGAN/releases/download/TANGO-ESRGAN/net_tangoesrgan_35000.pth']

    
    for p, v in zip(args.__dict__.keys(), args.__dict__.values()):
		      print('{}: {}'.format(p, v))

        # determine model paths
    if args.model_path is not None:
        model_path = args.model_path
    else:
        model_path = os.path.join('weights', args.model_name + '.pth')
        if not os.path.isfile(model_path):
            ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
            for url in file_url:
                # model_path will be updated
                model_path = load_file_from_url(
                    url=url, model_dir=os.path.join(ROOT_DIR, 'weights'), progress=True, file_name=None)

        # restorer
    dni_weight = None

    upsampler = TangoESRGANer(
        scale=netscale,
        model_path=model_path,
        dni_weight=dni_weight,
        model=model,
        tile=args.tile,
        tile_pad=args.tile_pad,
        pre_pad=args.pre_pad,
        half=not args.fp32,
        gpu_id=args.gpu_id)

    if args.face_enhance:  # Use GFPGAN for face enhancement
        file_url = [
            'https://github.com/TencentARC/GFPGAN/releases/download/v1.3.0/GFPGANv1.3.pth'
        ]
        model_path = os.path.join('weights', args.model_name + '.pth')
        if not os.path.isfile(model_path):
            ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
            for url in file_url:
                # model_path will be updated
                model_path = load_file_from_url(
                    url=url, model_dir=os.path.join(ROOT_DIR, 'weights'), progress=True, file_name=None)
        from gfpgan import GFPGANer
        face_enhancer = GFPGANer(
            model_path=model_path,
            upscale=args.outscale,
            arch='clean',
            channel_multiplier=2,
            bg_upsampler=upsampler)
    os.makedirs(args.output, exist_ok=True)

    mid_time = time.time()
    # subscriber init.
    sub_clean_image = rospy.Subscriber('/airsim_node/camera_frame', Image, fnc_clean_img_callback)
    sub_attacked_image  = rospy.Subscriber('/attack_generator_node/attacked_image', Image, fnc_img_callback)
    sub_adv_image = rospy.Subscriber('/attack_generator_node/perturbation_image', Image, fnc_adv_img_callback)
    # publishers init.
    pub_clean_image = rospy.Publisher('/fastdvdnet_node/clean_image', Image, queue_size=10)

    # Running rate
    rate=rospy.Rate(FREQ_MID_LEVEL)


    # a bridge from cv2 image to ROS image
    mybridge = CvBridge()

    error_count = 0
    n_iteration = 0

    seq_list = []

    ##############################
    ### Instructions in a loop ###
    ##############################

    while not rospy.is_shutdown():

        n_iteration += 1
        # Load the saved Model every 10 iteration
        

        # Image generation
        if IMAGE_RECEIVED is not None and ADV_IMAGE_RECEIVED is not None and IMAGE_CLEAN_RECEIVED is not None:
        # TRY THE REAL_TIME PERFORMANCE
        #if IMAGE_RECEIVED is not None:
            with torch.no_grad():
                # Get camera image
                np_clean_im = np.frombuffer(IMAGE_CLEAN_RECEIVED.data, dtype=np.uint8).reshape(IMAGE_CLEAN_RECEIVED.height, IMAGE_CLEAN_RECEIVED.width, -1)
                np_clean_im = np.array(np_clean_im)
                #print('np_clean_im',np_clean_im.shape) #448*448*3
                # Get attacked image
                np_im = np.frombuffer(IMAGE_RECEIVED.data, dtype=np.uint8).reshape(IMAGE_RECEIVED.height, IMAGE_RECEIVED.width, -1)
                np_im = np.array(np_im)
                #print('np_im',np_im.shape)

                # Get visualized noise image
                np_adv_im = np.frombuffer(ADV_IMAGE_RECEIVED.data, dtype=np.uint8).reshape(ADV_IMAGE_RECEIVED.height, ADV_IMAGE_RECEIVED.width, -1)
                np_adv_im = np.array(np_adv_im)
                #print('np_adv_im',np_adv_im.shape)#(448,448,3)

                clean_tensor = convert2torch(np_clean_im)
                attacked_tensor = convert2torch(np_im)
                adv_tensor = convert2torch(np_adv_im)

                seq, _, _ = open_sequence_denoiser(clean_tensor, np_clean_im, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                seq = torch.from_numpy(seq)

                #get attacked images seq shape is (max_num_fr_per_seq,3,448,448)
                seq_attack, _, _ = open_sequence_denoiser(attacked_tensor, np_im, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                seqn = torch.from_numpy(seq_attack)

                seq_adv, _, _ = open_sequence_denoiser(adv_tensor, np_adv_im, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                train_mean, train_std = calc_avg_mean_std(seq_adv, (448,448))
               
                noisestd_r = torch.FloatTensor([train_std[0]]) # this one has been nomalized 
                noisestd_g = torch.FloatTensor([train_std[1]]) 
                noisestd_b = torch.FloatTensor([train_std[2]]) 

                mid_time = time.time()
                if args.face_enhance:
                    _, _, output = face_enhancer.enhance(np_im, has_aligned=False, only_center_face=False, paste_back=True)
                else:
                    output, _ = upsampler.enhance(np_im, outscale=args.outscale)

                adv_frame = mybridge.cv2_to_imgmsg(output)
                pub_clean_image.publish(adv_frame)
                seq_time = time.time()
                denframes_tensor = convert2torch(output)
                denframes, _, _ = open_sequence_denoiser(denframes_tensor, output, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                denframes = torch.from_numpy(denframes)
                psnr = batch_psnr(denframes, seq, 1.)
                psnr_noisy = batch_psnr(seqn.squeeze(), seq, 1.)
                logger.info("\tPSNR noisy {:.4f}dB, PSNR result {:.4f}dB".format(psnr_noisy, psnr))
            stop_time = time.time()
            runtime = (seq_time - mid_time)
            #print('runtime',runtime) # gfpgan 2.65s # ESRGAN 2.87s # ESRNET 2.6S

        try:
            experiment_done_done = rospy.get_param('experiment_done')
        except:
            experiment_done_done = False
        if experiment_done_done and n_iteration > FREQ_MID_LEVEL*3:
            rospy.signal_shutdown('Finished 100 Episodes!')

        rate.sleep()
