# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "soundfile"]
# ///

import numpy as np
import soundfile as sf

wav_path = "input.wav"
audio, sr = sf.read(wav_path)

# Mix to mono if stereo
if audio.ndim > 1:
    audio = audio.mean(axis=1)

ir = audio[:1024]
ir /= np.max(np.abs(ir))  # normalize

with open("ir_1024.h", "w") as f:
    f.write("#pragma once\n\n")
    f.write("constexpr int IR_LEN = 1024;\n\n")
    f.write("const float ir_1024[IR_LEN] = {\n")
    for i, v in enumerate(ir):
        f.write(f"  {v:.8f},")
        if (i + 1) % 6 == 0:
            f.write("\n")
    f.write("\n};\n")

