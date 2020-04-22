import os
import random

trainval_percent = 0.95
train_percent = 0.95
dataset = 'sign'
xmlfilepath = 'VOC_%s/VOC2007/Annotations' % dataset
txtsavepath = 'VOC_%s/VOC2007/ImageSets/Main' % dataset
total_xml = os.listdir(xmlfilepath)

if '.DS_S' in total_xml:
    total_xml.remove('.DS_S')
if '.DS_Store' in total_xml:
    total_xml.remove('.DS_Store')

num=len(total_xml)
list=range(num)
tv=int(num*trainval_percent)
tr=int(tv*train_percent)
trainval= random.sample(list,tv)
train=random.sample(trainval,tr)

ftrainval = open('VOC_%s/VOC2007/ImageSets/Main/trainval.txt' % dataset, 'w')
ftest = open('VOC_%s/VOC2007/ImageSets/Main/test.txt' % dataset, 'w')
ftrain = open('VOC_%s/VOC2007/ImageSets/Main/train.txt' % dataset, 'w')
fval = open('VOC_%s/VOC2007/ImageSets/Main/val.txt' % dataset, 'w')

for i  in list:
    name=total_xml[i][:-4]+'\n'
    if i in trainval:
        ftrainval.write(name)
        if i in train:
            ftrain.write(name)
        else:
            fval.write(name)
    else:
        ftest.write(name)

ftrainval.close()
ftrain.close()
fval.close()
ftest .close()
