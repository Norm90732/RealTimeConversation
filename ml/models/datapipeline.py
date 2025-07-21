import datasets
import torch
from torch.utils.data import Dataset
import librosa
from datasets import load_dataset



ds = load_dataset(
    "parquet",
    data_files={"train": [
        "../data/train-00000-of-00003.parquet",
        "../data/train-00001-of-00003.parquet",
        "../data/train-00002-of-00003.parquet"]},
    split="train")
ds = ds.cast_column("audio", datasets.Audio(decode=False))
print(ds.features)

class IEMOCAPDataset(Dataset):
    '''
    Loading in audio through audio.bytes, the class tags for emotion classification,
    dimensional emotion model, speaker information, primary label, acoustic features
    in format for multi task learning through shared back bone
    '''
    def __init__(self,idx,):
        return hello