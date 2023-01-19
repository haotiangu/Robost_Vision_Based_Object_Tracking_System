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


from fastdvdnet.models import FastDVDnet
from fastdvdnet.fastdvdnet import denoise_seq_fastdvdnet

from fastdvdnet.utils import batch_psnr, init_logger_test, \
				variable_to_cv2_image, remove_dataparallel_wrapper, open_sequence, close_logger


NUM_IN_FR_EXT = 5 # temporal size of patch
MC_ALGO = 'DeepFlow' # motion estimation algorithm
OUTIMGEXT = '.png' # output images format


IMAGE_RECEIVED = None
def fnc_img_callback(msg):
    global IMAGE_RECEIVED
    IMAGE_RECEIVED = msg

TARGET_RECEIVED = None
def fnc_target_callback(msg):
    global TARGET_RECEIVED
    TARGET_RECEIVED = msg

def get_args():
    """ Get arguments for individual tb3 deployment. """
    parser = argparse.ArgumentParser(
        description="Denoise a sequence with FastDVDnet"
    )

    # Required arguments
    
    parser.add_argument("--model_file", 
                       type=str,
					   default="/home/haotiangu/catkin_ws/src/tcps_image_attack/scripts/fastdvdnet/model.pth", 
					   help='path to model of the pretrained denoiser')
    parser.add_argument("--suffix", type=str, default="", help='suffix to add to output name')
    parser.add_argument("--max_num_fr_per_seq", 
                        type=int, 
                        default=25,
					    help='max number of frames to load per sequence')
    parser.add_argument("--save_path", 
                        type=str, 
                        default='./results', 
					    help='where to save outputs as png')
    parser.add_argument("--dont_save_results", 
                        action='store_true', 
                        help="don't save output images")
    parser.add_argument("--save_noisy", 
                        action='store_true',
                        help="save noisy frames")
    parser.add_argument("--no_gpu", 
                        action='store_true', 
                        help="run model on CPU")
    parser.add_argument("--gray", 
                        action='store_true',
						help='perform denoising of grayscale images instead of RGB')
    
    return parser.parse_known_args(sys.argv)



if __name__ == '__main__':

    # rosnode node initialization
    rospy.init_node('fastdvdnet_node')
    print('Fastdvdnet_node is initialized at', os.getcwd())

    args, unknown = get_args()

    args.cuda = not args.no_gpu and torch.cuda.is_available()

    for p, v in zip(args.__dict__.keys(), args.__dict__.values()):
		      print('{}: {}'.format(p, v))

    start_time = time.time()
    if not os.path.exists(args.save_path):
		   os.makedirs(args.save_path)
    logger = init_logger_test(args.save_path)

	
    if torch.cuda.is_available():
        device = torch.device('cuda')
    else:
        device = torch.device('cpu')

    print('Loading models ...')
    model_temp = FastDVDnet(num_input_frames=NUM_IN_FR_EXT)
	
    state_temp_dict = torch.load(args.model_file,map_location=device)


    if torch.cuda.is_available():
        device_ids = [0]
        model_temp = nn.DataParallel(model_temp, device_ids=device_ids).cuda()
    else:
        state_temp_dict = remove_dataparallel_wrapper(state_temp_dict)
    model_temp.load_state_dict(state_temp_dict)

    model_temp.eval() 
    # subscriber init.
    sub_image = rospy.Subscriber('/airsim_node/camera_frame', Image, fnc_img_callback)
    sub_target = rospy.Subscriber('/decision_maker_node/target', Twist, fnc_target_callback)

    # publishers init.
    # pub_attacked_image = rospy.Publisher('/attack_generator_node/attacked_image', Image, queue_size=10)
    pub_clean_image = rospy.Publisher('/fastdvdnet_node/clean_image', Image, queue_size=10)
 
    # Running rate
    rate=rospy.Rate(FREQ_MID_LEVEL)

    
    # Training agents init
    SETTING['name'] = rospy.get_param('name')
    agent = ImageAttacker(SETTING)

    # a bridge from cv2 image to ROS image
    mybridge = CvBridge()
    
    error_count = 0
    n_iteration = 0


    ##############################
    ### Instructions in a loop ###
    ##############################

    while not rospy.is_shutdown():

        n_iteration += 1
        # Load the saved Model every 10 iteration
        if n_iteration%FREQ_MID_LEVEL == 0:
            try:
                #print(os.getcwd())
                agent.load_the_model()
                error_count = 0
            except:
                error_count +=1
                if error_count > 3:
                    print('In image_attack_node, model loading failed more than 10 times!')
        


        # Image generation
        if IMAGE_RECEIVED is not None and TARGET_RECEIVED is not None:

            with torch.no_grad():
                # Get camera image
                np_im = np.frombuffer(IMAGE_RECEIVED.data, dtype=np.uint8).reshape(IMAGE_RECEIVED.height, IMAGE_RECEIVED.width, -1)
                np_im = np.array(np_im)
                # Get action
                act = np.array([TARGET_RECEIVED.linear.x, TARGET_RECEIVED.linear.y, TARGET_RECEIVED.linear.z, TARGET_RECEIVED.angular.x])
                # Get attacked image
                attacked_obs,X_Attacked_Tensor,adv_image,X_adv_tensor = agent.generate_attack(np_im, act)

                denframes = denoise_seq_fastdvdnet(seq = X_Attacked_Tensor,\
                                                                                noise_std = X_adv_tensor,\
                                                                                temp_psz=NUM_IN_FR_EXT,\
                                                                                model_temporal=model_temp)
            #attacked_obs = (attacked_obs*255).astype('uint8')
            #attacked_frame = mybridge.cv2_to_imgmsg(attacked_obs)
            #print('attacked_frame',attacked_frame)

            adv_image = (adv_image*255).astype('uint8')
            adv_frame = mybridge.cv2_to_imgmsg(adv_image)

            #attacked_frame = np.array(attacked_frame)
           # noisestd = X_adv


            # Publish messages
            # pub_attacked_image.publish(attacked_frame)
            pub_clean_image.publish(adv_frame)
        try:
            experiment_done_done = rospy.get_param('experiment_done')
        except:
            experiment_done_done = False
        if experiment_done_done and n_iteration > FREQ_MID_LEVEL*3:
            rospy.signal_shutdown('Finished 100 Episodes!')
        
        rate.sleep()


        """
                denframes = denoise_seq_fastdvdnet(seq = attacked_frame,\
                                                                                noise_std = X_adv,\
                                                                                temp_psz=NUM_IN_FR_EXT,\
                                                                                model_temporal=model_temp)
"""