from .voc import VOCDataset
from .registry import DATASETS


@DATASETS.register_module
class MyDataset(VOCDataset):

    CLASSES = ('light', )