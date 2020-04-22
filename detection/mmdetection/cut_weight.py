import sys
import torch

src = sys.argv[1]
dst = sys.argv[2]

PTH = torch.load(src)
num_classes = 2
spec_class = 10

'''
List = ['bbox_head.0.fc_cls.weight', 'bbox_head.0.fc_cls.bias', 'bbox_head.1.fc_cls.weight', 'bbox_head.1.fc_cls.bias', 'bbox_head.2.fc_cls.weight', 'bbox_head.2.fc_cls.bias', \
'mask_head.0.conv_logits.weight', 'mask_head.0.conv_logits.bias', 'mask_head.1.conv_logits.weight', 'mask_head.1.conv_logits.bias', 'mask_head.2.conv_logits.weight', 'mask_head.2.conv_logits.bias']
'''


tem = PTH['state_dict']['bbox_head.0.fc_cls.weight'][10, :]
tem2 = PTH['state_dict']['bbox_head.0.fc_cls.bias'][10]
PTH['state_dict']['bbox_head.0.fc_cls.weight'].resize_(num_classes, 1024)
PTH['state_dict']['bbox_head.0.fc_cls.bias'].resize_(num_classes)
PTH['state_dict']['bbox_head.0.fc_cls.weight'] = tem
PTH['state_dict']['bbox_head.0.fc_cls.bias'] = tem2

tem = PTH['state_dict']['bbox_head.1.fc_cls.weight'][10, :]
tem2 = PTH['state_dict']['bbox_head.1.fc_cls.bias'][10]
PTH['state_dict']['bbox_head.1.fc_cls.weight'].resize_(num_classes, 1024)
PTH['state_dict']['bbox_head.1.fc_cls.bias'].resize_(num_classes)
PTH['state_dict']['bbox_head.1.fc_cls.weight'] = tem
PTH['state_dict']['bbox_head.1.fc_cls.bias'] = tem2

tem = PTH['state_dict']['bbox_head.2.fc_cls.weight'][10, :]
tem2 = PTH['state_dict']['bbox_head.2.fc_cls.bias'][10]
PTH['state_dict']['bbox_head.2.fc_cls.weight'].resize_(num_classes, 1024)
PTH['state_dict']['bbox_head.2.fc_cls.bias'].resize_(num_classes)
PTH['state_dict']['bbox_head.2.fc_cls.weight'] = tem
PTH['state_dict']['bbox_head.2.fc_cls.bias'] = tem2

tem = PTH['state_dict']['mask_head.0.conv_logits.weight'][10, :]
tem2 = PTH['state_dict']['mask_head.0.conv_logits.bias'][10, :]
PTH['state_dict']['mask_head.0.conv_logits.weight'].resize_(num_classes, 256, 1, 1)
PTH['state_dict']['mask_head.0.conv_logits.bias'].resize_(num_classes)
PTH['state_dict']['mask_head.0.conv_logits.weight'] = tem
PTH['state_dict']['mask_head.0.conv_logits.bias'] = tem2

tem = PTH['state_dict']['mask_head.1.conv_logits.weight'][10, :]
tem2 = PTH['state_dict']['mask_head.1.conv_logits.bias'][10, :]
PTH['state_dict']['mask_head.1.conv_logits.weight'].resize_(num_classes, 256, 1, 1)
PTH['state_dict']['mask_head.1.conv_logits.bias'].resize_(num_classes)
PTH['state_dict']['mask_head.1.conv_logits.weight'] = tem
PTH['state_dict']['mask_head.1.conv_logits.bias'] = tem2

tem = PTH['state_dict']['mask_head.2.conv_logits.weight'][10, :]
tem2 = PTH['state_dict']['mask_head.2.conv_logits.bias'][10, :]
PTH['state_dict']['mask_head.2.conv_logits.weight'].resize_(num_classes, 256, 1, 1)
PTH['state_dict']['mask_head.2.conv_logits.bias'].resize_(num_classes)
PTH['state_dict']['mask_head.2.conv_logits.weight'] = tem
PTH['state_dict']['mask_head.2.conv_logits.bias'] = tem2

torch.save(PTH, dst)