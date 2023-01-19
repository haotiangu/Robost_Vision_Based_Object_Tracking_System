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
				variable_to_cv2_image, remove_dataparallel_wrapper, open_sequence_denoiser, close_logger


NUM_IN_FR_EXT = 5 # temporal size of patch
MC_ALGO = 'DeepFlow' # motion estimation algorithm
OUTIMGEXT = '.png' # output images format


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
                        default=5,
					    help='max number of frames to load per sequence')
    parser.add_argument("--save_path", 
                        type=str, 
                        default='/home/haotiangu/catkin_ws/src/tcps_image_attack/scripts/result', 
					    help='where to save outputs as png')
    parser.add_argument("--noise_sigma",
                        type=float, 
                        default=0, 
                        help='noise level used on test set')
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



def save_out_seq(seqnoisy, seqclean, save_dir, sigmaval, suffix, save_noisy):
	"""Saves the denoised and noisy sequences under save_dir
	"""
	seq_len = seqnoisy.size()[0]
	for idx in range(seq_len):
		# Build Outname
		fext = OUTIMGEXT
		noisy_name = os.path.join(save_dir,\
						('n{}_{}').format(sigmaval, idx) + fext)
		if len(suffix) == 0:
			out_name = os.path.join(save_dir,\
					('n{}_FastDVDnet_{}').format(sigmaval, idx) + fext)
		else:
			out_name = os.path.join(save_dir,\
					('n{}_FastDVDnet_{}_{}').format(sigmaval, suffix, idx) + fext)

		# Save result
		if save_noisy:
			noisyimg = variable_to_cv2_image(seqnoisy[idx].clamp(0., 1.))
			cv2.imwrite(noisy_name, noisyimg)

		outimg = variable_to_cv2_image(seqclean[idx].unsqueeze(dim=0))
		cv2.imwrite(out_name, outimg)

if __name__ == '__main__':

    # rosnode node initialization
    rospy.init_node('fastdvdnet_node')
    print('Fastdvdnet_node is initialized at', os.getcwd())
    start_time = time.time()
    args, unknown = get_args()
	
    args.cuda = not args.no_gpu and torch.cuda.is_available()
   
    for p, v in zip(args.__dict__.keys(), args.__dict__.values()):
		      print('{}: {}'.format(p, v))

    
    if not os.path.exists(args.save_path):
		   os.makedirs(args.save_path)
    logger = init_logger_test(args.save_path)

	
    if args.cuda:
        device = torch.device('cuda')
    else:
        device = torch.device('cpu')
  
    print('Loading models ...')
    model_temp = FastDVDnet(num_input_frames=NUM_IN_FR_EXT)
	
    state_temp_dict = torch.load(args.model_file, map_location=device)


    if args.cuda:
        device_ids = [0]
        model_temp = nn.DataParallel(model_temp, device_ids=device_ids).cuda()
    else:
        state_temp_dict = remove_dataparallel_wrapper(state_temp_dict)
    model_temp.load_state_dict(state_temp_dict)

    model_temp.eval() 
    mid_time = time.time()
    # subscriber init.
    #sub_image = rospy.Subscriber('/airsim_node/camera_frame', Image, fnc_img_callback)
    #sub_target = rospy.Subscriber('/decision_maker_node/target', Twist, fnc_target_callback)
    sub_clean_image = rospy.Subscriber('/airsim_node/camera_frame', Image, fnc_clean_img_callback)
    sub_attacked_image  = rospy.Subscriber('/attack_generator_node/attacked_image', Image, fnc_img_callback)
    sub_adv_image = rospy.Subscriber('/attack_generator_node/perturbation_image', Image, fnc_adv_img_callback)

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

    seq_list = []

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
        if IMAGE_RECEIVED is not None and ADV_IMAGE_RECEIVED is not None and IMAGE_CLEAN_RECEIVED is not None:

            with torch.no_grad():
                # Get camera image
                np_clean_im = np.frombuffer(IMAGE_CLEAN_RECEIVED.data, dtype=np.uint8).reshape(IMAGE_CLEAN_RECEIVED.height, IMAGE_CLEAN_RECEIVED.width, -1)
                np_clean_im = np.array(np_clean_im)          
                # Get attacked image
                np_im = np.frombuffer(IMAGE_RECEIVED.data, dtype=np.uint8).reshape(IMAGE_RECEIVED.height, IMAGE_RECEIVED.width, -1)
                np_im = np.array(np_im)
                
                # Get visualized noise image
                np_adv_im = np.frombuffer(ADV_IMAGE_RECEIVED.data, dtype=np.uint8).reshape(ADV_IMAGE_RECEIVED.height, ADV_IMAGE_RECEIVED.width, -1)
                np_adv_im = np.array(np_adv_im)
                #print(np_adv_im.shape)#(448,448,3)
                # Get clean camera image/attacked image/noise image tensor tensor
                clean_tensor = convert2torch(np_clean_im)
                attacked_tensor = convert2torch(np_im) 
                adv_tensor = convert2torch(np_adv_im)
                mid_start_time = time.time()
 
                seq, _, _ = open_sequence_denoiser(clean_tensor, np_clean_im, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                seq = torch.from_numpy(seq).to(device)

                #get attacked images seq shape is (max_num_fr_per_seq,3,448,448)
                seq_attack, _, _ = open_sequence_denoiser(attacked_tensor, np_im, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                seqn = torch.from_numpy(seq_attack).to(device)

                #get adv images in shape (max_num_fr_per_seq,3,448,448) 
                seq_adv, _, _ = open_sequence_denoiser(adv_tensor, np_adv_im, args.gray, expand_if_needed=False, max_num_fr=args.max_num_fr_per_seq)
                seq_time = time.time()
                train_mean, train_std = calc_avg_mean_std(seq_adv, (448,448))
                #the length of seq_adv = 5 max_num_fr_per_seq
                noisestd_r = torch.FloatTensor([train_std[0]]).to(device) # this one has been nomalized 
                noisestd_g = torch.FloatTensor([train_std[1]]).to(device) 
                noisestd_b = torch.FloatTensor([train_std[2]]).to(device) 

                
                denframes = denoise_seq_fastdvdnet(seq = seqn,\
                                                                                noise_std = noisestd_r,\
                                                                                temp_psz=NUM_IN_FR_EXT,\
                                                                                model_temporal=model_temp)
                print(deframes.shape)
                '''
                denframes = denoise_seq_fastdvdnet(seq = denframes,\
                                                                                noise_std = noisestd_g,\
                                                                                temp_psz=NUM_IN_FR_EXT,\
                                                                                model_temporal=model_temp)
                denframes = denoise_seq_fastdvdnet(seq = denframes,\
                                                                                noise_std = noisestd_b,\
                                                                                temp_psz=NUM_IN_FR_EXT,\
                                                                                model_temporal=model_temp)
                '''
            stop_time = time.time()
            psnr = batch_psnr(denframes, seq, 1.)
            psnr_noisy = batch_psnr(seqn.squeeze(), seq, 1.)
            runtime = (stop_time - seq_time)
            loadtime = (mid_start_time - seq_time) + (mid_time - start_time)
            seq_len = denframes.size()[0]
            logger.info("\tDenoised {} frames in {:.3f}s, loaded seq in {:.3f}s".\
				 format(seq_len, runtime, loadtime))
            logger.info("\tPSNR noisy {:.4f}dB, PSNR result {:.4f}dB".format(psnr_noisy, psnr))
            for idx in range(seq_len):
                outimg = variable_to_cv2_image(denframes[idx].unsqueeze(dim=0),conv_rgb_to_bgr=False)
                adv_frame = mybridge.cv2_to_imgmsg(outimg)
                pub_clean_image.publish(adv_frame)

            #adv_image = (adv*255).astype('uint8')
            #adv_frame = mybridge.cv2_to_imgmsg(adv_image)
            #pub_clean_image.publish(adv_frame)
            #pub_clean_image.publish(adv_frame)
        
            
        try:
            experiment_done_done = rospy.get_param('experiment_done')
        except:
            experiment_done_done = False
        if experiment_done_done and n_iteration > FREQ_MID_LEVEL*3:
            rospy.signal_shutdown('Finished 100 Episodes!')
        
        rate.sleep()

'''
	        seq_len = denframes.size()[0]
	        for idx in range(seq_len):
                outimg = variable_to_cv2_image(denframes[idx].unsqueeze(dim=0))
                print('outimg',outimg.shape)
'''