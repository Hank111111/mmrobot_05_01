#coding:utf-8
from __future__ import division
#from __future__ import unicode_literals

import random
import torch
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data
from torch.autograd import Variable 
import numpy as np
import os
import util
import dataset
from PIL import Image
import models.crnn as crnn
import keys

alphabet = keys.alphabet
#print(len(alphabet))
#raw_input('\ninput:')
converter = util.strLabelConverter(alphabet)
model = crnn.CRNN(32, 1, len(alphabet)+1, 256, 1).cuda()
path = '/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/crnn/samples/netCRNN63.pth'
model.load_state_dict(torch.load(path))
#while 1:
    #im_name = raw_input("\nplease input file name:")

def crnn_process(num):
    #print(model)
    #print num
    #char_num=[]
    char_list=[]
    sumnum = 0
    for a in range (0,num):
    	im_path =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/text-detection-ctpn/crnn/img/" + str(a) +".png"
    	image = Image.open(im_path).convert('L')
    	scale = image.size[1]*1.0 / 32
    	w = image.size[0] / scale
    	w = int(w)
    	#print(w)
	
    	transformer = dataset.resizeNormalize((w, 32))
    	image = transformer(image).cuda()
    	image = image.view(1, *image.size())
    	image = Variable(image)
    	model.eval()
    	preds = model(image)
    	_, preds = preds.max(2)
    	preds = preds.squeeze(1)
    	preds = preds.transpose(0, -1).contiguous().view(-1)
    	preds_size = Variable(torch.IntTensor([preds.size(0)]))
    	#raw_pred = converter.decode(preds.data, preds_size.data, raw=True)
    	sim_pred = converter.decode(preds.data, preds_size.data, raw=False)
    	#print('%-20s => %-20s' % (raw_pred, sim_pred))
	#char_num.append(len(sim_pred))
	char_list.append(sim_pred)
	#result_str = " ".join(char_list)
    return char_list
	

if __name__ == '__main__':
    crnn_process(1)
