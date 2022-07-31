"""
GIF animation creating program

Author: Shisato Yano
"""

from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import os


class GifAnimation:
    """
    各プログラムのアニメーションをGIF形式で保存するクラス
    GIFとは: 複数の静止画像を組み合わせてパラパラ漫画のようにしたもの
    """

    def __init__(self, save_name_path, duration_ms):
        self.image_counter = 0
        self.images_array = []
        self.save_name_path = save_name_path
        self.duration_ms = duration_ms
    
    def save_image(self):
        plt.savefig(str(self.image_counter) + ".png")
        self.image_counter += 1
    
    def append_image(self):
        for num in range(self.image_counter):
            image = Image.open(str(num) + ".png")
            self.images_array.append(image)
    
    def remove_image(self):
        for num in range(self.image_counter):
            os.remove(str(num) + ".png")
    
    def create_gif(self):
        if self.image_counter == 0:
            print("Can not create gif because there is no image file..")
            return False
        else:
            self.append_image()
            
            print("Created " + self.save_name_path)
            self.images_array[0].save(self.save_name_path, 
                                      save_all=True, 
                                      append_images=self.images_array[1:],
                                      loop=0,
                                      duration=self.duration_ms)
            
            self.remove_image()
            
            return True
