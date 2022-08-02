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
        """
        コンストラクタ
        save_name_path: 作成したGIFファイルの保存場所とファイル名
        duration_ms: 画像の更新周期[msec]
        """

        self.image_counter = 0 # 静止画像の枚数
        self.images_array = [] # 画像ファイルのポインタを格納する配列
        self.save_name_path = save_name_path
        self.duration_ms = duration_ms
    
    def save_image(self):
        """
        各フレームの画像を保存する関数
        保存した画像の枚数をカウントアップ
        """
        
        plt.savefig(str(self.image_counter) + ".png")
        self.image_counter += 1
    
    def append_image(self):
        """
        保存した画像ファイルを開いてファイルポインタを取得し
        取得したポインタを配列に格納する関数
        """

        for num in range(self.image_counter):
            image = Image.open(str(num) + ".png")
            self.images_array.append(image)
    
    def remove_image(self):
        """
        保存した画像ファイルでGIFファイルを生成した後に
        不要な画像ファイルを全て削除する関数
        """

        for num in range(self.image_counter):
            os.remove(str(num) + ".png")
    
    def create_gif(self):
        """
        保存した全画像ファイルを繋ぎ合わせて
        GIFファイルを作成する関数
        """

        if self.image_counter == 0: # 保存された画像が無ければ終了
            print("Can not create gif because there is no image file..")
            return False
        else:
            self.append_image() # 画像ファイルポインタを配列に格納
            
            # GIFファイルを生成して保存
            print("Created " + self.save_name_path)
            self.images_array[0].save(self.save_name_path, 
                                      save_all=True, 
                                      append_images=self.images_array[1:],
                                      loop=0,
                                      duration=self.duration_ms)
            
            self.remove_image() # 不要な画像ファイルを削除
            
            return True
