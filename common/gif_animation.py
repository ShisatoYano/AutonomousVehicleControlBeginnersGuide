"""
GIF animation creating program

Author: Shisato Yano
"""


class GifAnimation:
    """
    各プログラムのアニメーションをGIF形式で保存するクラス
    GIFとは: 複数の静止画像を組み合わせてパラパラ漫画のようにしたもの
    """

    def __init__(self, save_name_path, duration_ms):
        self.image_counter = 0
        self.image_array = []
        self.save_name_path = save_name_path
        self.duration_ms = duration_ms
