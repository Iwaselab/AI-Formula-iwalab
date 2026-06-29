from ultralytics import YOLO

model = YOLO('yolo11s.pt') #ベースモデル

results = model.train(
    data='/home/iwalab/AIFMovie/crosswalk/data.yaml', #学習に使用するデータセットの設定ファイルへのパス
    epochs=150, #学習エポック数
    imgsz=640, #入力画像のサイズ
    rect=True, #矩形トレーニングを有効にするかどうか。rosbagを用いた学習の場合はTrueにすることが推奨される
    patience=30,          #改善が見られない場合に、トレーニングを早期終了するまでのエポック数
    optimizer='AdamW',    #オプティマイザー。AdamWが今の標準らしい。
    device=0,  #GPU使用
    batch=-1, #バッチサイズ
    workers=1, #データローダーのワーカー数
)