"""Model inventory is standard-library-only so doctor works before setup."""
import importlib.util
from pathlib import Path

MODEL_FILES = {'tokens':'tokens.txt', 'encoder':'encoder-epoch-99-avg-1.int8.onnx',
               'decoder':'decoder-epoch-99-avg-1.int8.onnx', 'joiner':'joiner-epoch-99-avg-1.int8.onnx'}


def model_status(model_dir):
    directory = Path(model_dir)
    missing = [name for name in MODEL_FILES.values() if not (directory/name).is_file()]
    return {'path':str(directory.resolve()), 'missing':missing, 'ready':not missing,
            'sherpa_onnx_installed':importlib.util.find_spec('sherpa_onnx') is not None}
